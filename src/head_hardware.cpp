#include "openarmx_head_hardware/head_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <openarmx/robstride_motor/rs_motor_control.hpp>

namespace openarmx_head_hardware {

static inline openarmx::robstride_motor::CANPacket mit_set_mode_packet(
    const openarmx::robstride_motor::Motor& motor) {
  return openarmx::robstride_motor::CanPacketEncoder::create_write_param_command(
      motor, 0x7005, 0.0f, openarmx::robstride_motor::ParamValueType::UINT8);
}

OpenArmX_HeadHW::OpenArmX_HeadHW() = default;

OpenArmX_HeadHW::~OpenArmX_HeadHW() {
  if (param_spin_thread_active_) {
    param_spin_thread_active_ = false;
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
  }
}

bool OpenArmX_HeadHW::parse_config(const hardware_interface::HardwareInfo& info) {
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can2";

  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = false;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  it = info.hardware_parameters.find("control_mode");
  if (it == info.hardware_parameters.end()) {
    control_mode_ = ControlMode::MIT;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    control_mode_ = (value == "csp") ? ControlMode::CSP : ControlMode::MIT;
  }

  it = info.hardware_parameters.find("home_on_activate");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    home_on_activate_ = (value == "true" || value == "1");
  }

  it = info.hardware_parameters.find("home_duration_sec");
  if (it != info.hardware_parameters.end()) {
    try {
      home_duration_sec_ = std::stod(it->second);
    } catch (...) {
      home_duration_sec_ = 3.0;
    }
    if (home_duration_sec_ < 0.1) home_duration_sec_ = 0.1;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "Configuration: CAN=%s, can_fd=%s, control_mode=%s, home_on_activate=%s, home_duration=%.2fs",
              can_interface_.c_str(),
              can_fd_ ? "enabled" : "disabled",
              (control_mode_ == ControlMode::MIT ? "mit" : "csp"),
              home_on_activate_ ? "true" : "false",
              home_duration_sec_);
  return true;
}

void OpenArmX_HeadHW::generate_joint_names() {
  joint_names_.clear();
  joint_names_.push_back("openarmx_head_yaw_joint");
  joint_names_.push_back("openarmx_head_pitch_joint");

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "Generated %zu joint names for head", joint_names_.size());
}

hardware_interface::CallbackReturn OpenArmX_HeadHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  generate_joint_names();

  // Initialize ROS2 node for dynamic parameters
  param_node_ = std::make_shared<rclcpp::Node>("openarmx_head_hardware_params");

  // 头部两个关节都用 RS00，但 yaw 与 pitch 的负载特性不同。
  // yaw 基本只克服摩擦和惯量，默认 KP 保持 30 即可。pitch 需要持续抗重力，
  // 如果 KP 太低，在 MIT 模式下会表现为能收到命令但基本不抬头，所以这里
  // 直接把 pitch 的默认 KP/KD 提高到 100.0 / 2.0，进一步提高抗重力能力。
  kp_values_ = {100.0, 100.0};
  kd_values_ = {10.0, 10.0};

  for (size_t i = 0; i < HEAD_DOF; ++i) {
    std::string kp_name = "kp_" + joint_names_[i];
    std::string kd_name = "kd_" + joint_names_[i];
    param_node_->declare_parameter(kp_name, kp_values_[i]);
    param_node_->declare_parameter(kd_name, kd_values_[i]);
  }

  param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      std::bind(&OpenArmX_HeadHW::parameters_callback, this, std::placeholders::_1));

  param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  param_executor_->add_node(param_node_);
  param_spin_thread_active_ = true;
  param_spin_thread_ = std::thread([this]() {
    while (param_spin_thread_active_ && rclcpp::ok()) {
      param_executor_->spin_some(std::chrono::milliseconds(10));
    }
  });

  // Initialize OpenArmX on head CAN bus
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "Initializing OpenArmX head on %s...", can_interface_.c_str());
  openarmx_ = std::make_unique<openarmx::can::socket::OpenArmX>(can_interface_, can_fd_);
  openarmx_->init_head_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS, DEFAULT_RECV_CAN_IDS);

  // Initialize state and command vectors
  pos_commands_.resize(HEAD_DOF, 0.0);
  vel_commands_.resize(HEAD_DOF, 0.0);
  tau_commands_.resize(HEAD_DOF, 0.0);
  pos_states_.resize(HEAD_DOF, 0.0);
  vel_states_.resize(HEAD_DOF, 0.0);
  tau_states_.resize(HEAD_DOF, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "OpenArmX Head HW initialized successfully");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_HeadHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  openarmx_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx_->recv_all();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmX_HeadHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmX_HeadHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmX_HeadHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"), "Activating OpenArmX Head...");
  openarmx_->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);

  using namespace openarmx::robstride_motor;
  auto& master = openarmx_->get_master_can_device_collection();
  auto& sock = master.get_can_socket();
  auto& head = openarmx_->get_head();
  auto head_motors = head.get_all_motors();
  auto head_devices = head.get_all_devices();

  // 1) Disable all motors
  if (!openarmx_->disable_all()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArmX_HeadHW"),
                "Some motors failed to disable, continuing...");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (control_mode_ == ControlMode::MIT) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                "Configuring %zu head motors for MIT mode", head_motors.size());
    for (size_t i = 0; i < head_motors.size(); ++i) {
      configure_motor_mit_mode(head_motors[i], head_devices[i], sock);
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                "Configuring %zu head motors for CSP mode", head_motors.size());
    for (size_t i = 0; i < head_motors.size(); ++i) {
      configure_motor_csp_mode(head_motors[i], head_devices[i], sock);
    }
  }

  // Wait for mode switch to settle before enabling
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  openarmx_->recv_all();

  // 2) Enable all motors
  if (!openarmx_->enable_all()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_HeadHW"),
                 "Failed to enable head motors");
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  openarmx_->refresh_all();
  openarmx_->recv_all();

  // 3) Initialize commands to current position to avoid sudden movement
  const auto direction_multipliers = get_motor_direction_multipliers();
  for (size_t i = 0; i < HEAD_DOF && i < head_motors.size(); ++i) {
    pos_commands_[i] = head_motors[i]->get_position() * direction_multipliers[i];
  }

  // 4) 启动自动缓慢回零：记录起点位置，write() 将在 home_duration_sec_ 内
  //    线性插值把 pos_commands_ 从当前位置拉到 0。
  if (home_on_activate_) {
    home_start_positions_.assign(pos_commands_.begin(), pos_commands_.end());
    home_start_time_ = std::chrono::steady_clock::now();
    homing_active_ = true;
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                "Homing armed: yaw %.3f -> 0, pitch %.3f -> 0 over %.2fs",
                home_start_positions_[0], home_start_positions_[1],
                home_duration_sec_);
  } else {
    homing_active_ = false;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"), "OpenArmX Head activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_HeadHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"), "Deactivating OpenArmX Head...");
  if (!openarmx_->disable_all()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArmX_HeadHW"),
                "Some motors failed to disable during deactivation");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"), "OpenArmX Head deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmX_HeadHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  openarmx_->refresh_all();
  openarmx_->recv_all();

  auto head_motors = openarmx_->get_head().get_motors();
  const auto direction_multipliers = get_motor_direction_multipliers();

  for (size_t i = 0; i < HEAD_DOF && i < head_motors.size(); ++i) {
    pos_states_[i] = head_motors[i]->get_position() * direction_multipliers[i];
    vel_states_[i] = head_motors[i]->get_velocity() * direction_multipliers[i];
    tau_states_[i] = head_motors[i]->get_torque() * direction_multipliers[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmX_HeadHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto direction_multipliers = get_motor_direction_multipliers();

  // 启动缓回零：覆盖控制器写入的 pos_commands_，在 home_duration_sec_
  // 内把每个关节从 home_start_positions_ 线性插值到 0。期间也把速度/力矩
  // 前馈清零，避免叠加上层指令造成抖动。
  if (homing_active_) {
    const double elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - home_start_time_).count();
    const double alpha = std::min(1.0, elapsed / home_duration_sec_);
    for (size_t i = 0; i < HEAD_DOF; ++i) {
      pos_commands_[i] = home_start_positions_[i] * (1.0 - alpha);
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
    }
    if (alpha >= 1.0) {
      homing_active_ = false;
      RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                  "Homing finished, yielding to controller commands");
    }
  }

  if (control_mode_ == ControlMode::MIT) {
    std::vector<openarmx::robstride_motor::MotionControlParam> params(HEAD_DOF);
    std::lock_guard<std::mutex> lock(kp_kd_mutex_);

    for (size_t i = 0; i < HEAD_DOF; ++i) {
      params[i].kp = kp_values_[i];
      params[i].kd = kd_values_[i];
      params[i].position = pos_commands_[i] * direction_multipliers[i];
      params[i].velocity = vel_commands_[i] * direction_multipliers[i];
      params[i].torque = tau_commands_[i] * direction_multipliers[i];
    }
    openarmx_->get_head().send_motion_control_commands(params);
  } else {
    // CSP mode: write LOC_REF for each motor
    using namespace openarmx::robstride_motor;
    auto& head = openarmx_->get_head();
    auto motors = head.get_all_motors();
    auto devices = head.get_all_devices();
    auto& sock = openarmx_->get_master_can_device_collection().get_can_socket();

    for (size_t i = 0; i < HEAD_DOF && i < motors.size(); ++i) {
      float target = static_cast<float>(pos_commands_[i] * direction_multipliers[i]);
      auto pkt = csp_set_target_position_packet(*motors[i], target);
      auto frame = devices[i]->create_can_frame(pkt.send_can_id, pkt.data);
      sock.write_can_frame(frame);
    }
  }

  openarmx_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArmX_HeadHW::configure_motor_csp_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  auto mode_pkt = csp_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  auto motor_type = motor->get_motor_type();
  size_t idx = static_cast<size_t>(motor_type);
  const auto& limits = MOTOR_LIMIT_PARAMS[idx];

  // Speed limit for head: conservative 1.0 rad/s
  auto spd_pkt = csp_set_speed_limit_packet(*motor, 1.0f);
  auto spd_frame = device->create_can_frame(spd_pkt.send_can_id, spd_pkt.data);
  socket.write_can_frame(spd_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  auto cur_pkt = csp_set_current_limit_packet(*motor, static_cast<float>(limits.tMax));
  auto cur_frame = device->create_can_frame(cur_pkt.send_can_id, cur_pkt.data);
  socket.write_can_frame(cur_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void OpenArmX_HeadHW::configure_motor_mit_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  uint32_t motor_id = motor->get_send_can_id();
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "Setting MIT mode for motor ID=%u", motor_id);

  // 与 CSP 路径不同，MIT 路径此前只切换了 RUN_MODE，没有显式放开扭矩上限。
  // 这会导致轻载的 yaw 还能动，但持续抗重力的 pitch 在高 KP 下也只能挪一点。
  auto motor_type = motor->get_motor_type();
  size_t idx = static_cast<size_t>(motor_type);
  const auto& limits = MOTOR_LIMIT_PARAMS[idx];

  // 1) Read current RUN_MODE to see what mode the motor is in
  auto query_pkt = CanPacketEncoder::create_query_param_command(*motor, ParamIndex::RUN_MODE);
  auto query_frame = device->create_can_frame(query_pkt.send_can_id, query_pkt.data);
  socket.write_can_frame(query_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  openarmx_->recv_all();

  // 2) Write RUN_MODE = 0 (MIT mode) - must be done in disabled state
  auto mode_pkt = mit_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  openarmx_->recv_all();

  // 3) Write again to be sure (some motors need a retry)
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  openarmx_->recv_all();

  // 4) In MIT mode, explicitly raise LIMIT_TORQUE to the motor-type maximum.
  //    CSP 路径已经会写限流/限扭矩，而 MIT 路径若沿用偏小的旧值，pitch 会在
  //    重力载荷下表现成“命令到了但抬不起来”。
  auto torque_limit_pkt = CanPacketEncoder::create_write_param_command(
      *motor, ParamIndex::LIMIT_TORQUE, static_cast<float>(limits.tMax), ParamValueType::FLOAT);
  auto torque_limit_frame = device->create_can_frame(
      torque_limit_pkt.send_can_id, torque_limit_pkt.data);
  socket.write_can_frame(torque_limit_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  openarmx_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
              "MIT mode set for motor ID=%u with torque_limit=%.2f",
              motor_id, limits.tMax);
}

std::vector<double> OpenArmX_HeadHW::get_motor_direction_multipliers() const {
  // Match real motor mounting to the URDF/teleop convention.
  // Current hardware requires yaw reversed while pitch already matches.
  return {-1.0, 1.0};
}

rcl_interfaces::msg::SetParametersResult OpenArmX_HeadHW::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  for (const auto& param : parameters) {
    std::string name = param.get_name();

    for (size_t i = 0; i < HEAD_DOF; ++i) {
      std::string kp_name = "kp_" + joint_names_[i];
      std::string kd_name = "kd_" + joint_names_[i];

      if (name == kp_name) {
        double old_val = kp_values_[i];
        kp_values_[i] = param.as_double();
        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                    "Updated %s: %.2f -> %.2f", name.c_str(), old_val, kp_values_[i]);
      } else if (name == kd_name) {
        double old_val = kd_values_[i];
        kd_values_[i] = param.as_double();
        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_HeadHW"),
                    "Updated %s: %.2f -> %.2f", name.c_str(), old_val, kd_values_[i]);
      }
    }
  }

  return result;
}

}  // namespace openarmx_head_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarmx_head_hardware::OpenArmX_HeadHW,
                       hardware_interface::SystemInterface)
