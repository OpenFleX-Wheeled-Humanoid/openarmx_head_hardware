# openarmx_head_hardware

[English](./README.md) | 中文

---

OpenArmX 2自由度头部（偏航 + 俯仰）的 ros2_control 硬件接口插件。

## 概述

本包实现了 `hardware_interface::SystemInterface` 插件，通过 CAN 总线与两个 Robstride RS00 电机通信以控制 OpenArmX 头部。支持两种控制模式（MIT 和 CSP），为每个关节提供位置、速度和力矩的指令/状态接口。

## 插件信息

- **插件名称**：`openarmx_head_hardware/OpenArmX_HeadHW`
- **基类**：`hardware_interface::SystemInterface`
- **自由度**：2（偏航 + 俯仰）
- **电机**：2x Robstride RS00
  - 偏航（CAN ID 0x01）：左右旋转
  - 俯仰（CAN ID 0x02）：上下旋转

## 功能特性

- **双控制模式**：
  - **MIT 模式**：运动控制，支持可配置的 KP/KD 增益和力矩前馈
  - **CSP 模式**：周期同步位置模式，带速度/电流限制
- **激活时自动回零**：从当前位置平滑插值到零位，默认持续 3 秒，避免突跳
- **动态参数调节**：每个关节的 KP/KD 值可在运行时通过 ROS 2 参数实时调整
- **仿真硬件支持**：可替换为 `fake_components/GenericSystem` 用于仿真

## 硬件参数

通过 URDF ros2_control xacro 配置：

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `can_interface` | `can2` | CAN 总线接口名称 |
| `can_fd` | `false` | 启用 CAN-FD |
| `control_mode` | `mit` | `mit` 或 `csp` |
| `home_on_activate` | `true` | 激活时启用缓慢回零 |
| `home_duration_sec` | `3.0` | 回零插值持续时间（秒） |

## 接口

### 指令接口（每个关节）

- `position`（弧度）
- `velocity`（rad/s）
- `effort`（Nm）

### 状态接口（每个关节）

- `position`（弧度）
- `velocity`（rad/s）
- `effort`（Nm）

## 动态参数

在 `/openarmx_head_hardware_params` 节点上可用：

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `kp_openarmx_head_yaw_joint` | 100.0 | 偏航位置增益（MIT 模式） |
| `kd_openarmx_head_yaw_joint` | 10.0 | 偏航速度增益（MIT 模式） |
| `kp_openarmx_head_pitch_joint` | 100.0 | 俯仰位置增益（MIT 模式） |
| `kd_openarmx_head_pitch_joint` | 10.0 | 俯仰速度增益（MIT 模式） |

运行时调整：

```bash
ros2 param set /openarmx_head_hardware_params kp_openarmx_head_pitch_joint 150.0
ros2 param set /openarmx_head_hardware_params kd_openarmx_head_pitch_joint 5.0
```

## 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_hardware
source install/setup.bash
```

## 依赖

- `hardware_interface`
- `pluginlib`
- `rclcpp` / `rclcpp_lifecycle`
- `openarmx_can`（CAN 通信库）

## 注意事项

- 电机方向映射：偏航电机方向取反（-1.0）以匹配 URDF 约定；俯仰电机方向不变（1.0）。
- MIT 模式下会显式将力矩限制设为电机最大值，确保俯仰关节能够抵抗重力。
- CSP 模式使用保守的速度限制（1.0 rad/s）以确保安全。
- 硬件激活前需确保 CAN 接口已配置并启用。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
