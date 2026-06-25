#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>
#include <openarmx/robstride_motor/rs_motor_control.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarmx_head_hardware {

class OpenArmX_HeadHW : public hardware_interface::SystemInterface {
public:
  OpenArmX_HeadHW();
  ~OpenArmX_HeadHW();

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr size_t HEAD_DOF = 2;

  // 默认电机配置: 2 x RS00
  // joint1 = yaw (左右摇头, CAN ID 0x01), joint2 = pitch (俯仰, CAN ID 0x02)
  const std::vector<openarmx::robstride_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarmx::robstride_motor::MotorType::RS00,  // yaw
      openarmx::robstride_motor::MotorType::RS00,  // pitch
  };
  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x01, 0x02};

  // Configuration
  std::string can_interface_;
  bool can_fd_;
  enum class ControlMode { MIT, CSP };
  ControlMode control_mode_ = ControlMode::MIT;

  // ROS2 node for dynamic parameter handling
  rclcpp::Node::SharedPtr param_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr param_executor_;
  std::thread param_spin_thread_;
  std::atomic<bool> param_spin_thread_active_{false};

  // Dynamic KP and KD values (2 joints)
  std::vector<double> kp_values_;
  std::vector<double> kd_values_;
  std::mutex kp_kd_mutex_;

  // OpenArmX instance (head uses its own CAN bus)
  std::unique_ptr<openarmx::can::socket::OpenArmX> openarmx_;

  // Joint names
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  void configure_motor_csp_mode(openarmx::robstride_motor::Motor* motor,
                                openarmx::robstride_motor::RSCANDevice* device,
                                openarmx::canbus::CANSocket& socket);
  void configure_motor_mit_mode(openarmx::robstride_motor::Motor* motor,
                                openarmx::robstride_motor::RSCANDevice* device,
                                openarmx::canbus::CANSocket& socket);

  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  std::vector<double> get_motor_direction_multipliers() const;

  // 启动自动缓慢回零：on_activate 记录每个关节当前位置，write() 前
  // N 秒内线性插值把目标从当前位置推到 0，避免上电后的突跳。
  bool home_on_activate_ = true;
  double home_duration_sec_ = 3.0;
  bool homing_active_ = false;
  std::vector<double> home_start_positions_;
  std::chrono::steady_clock::time_point home_start_time_;
};

}  // namespace openarmx_head_hardware
