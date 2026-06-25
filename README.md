# openarmx_head_hardware

English | [中文](./README-CN.md)

---

ros2_control hardware interface plugin for the OpenArmX 2-DOF head (yaw + pitch).

## Overview

This package implements a `hardware_interface::SystemInterface` plugin that communicates with two Robstride RS00 motors over CAN bus to control the OpenArmX head. It supports two control modes (MIT and CSP) and provides position, velocity, and effort command/state interfaces for each joint.

## Plugin Info

- **Plugin name**: `openarmx_head_hardware/OpenArmX_HeadHW`
- **Base class**: `hardware_interface::SystemInterface`
- **DOF**: 2 (yaw + pitch)
- **Motors**: 2x Robstride RS00
  - Yaw (CAN ID 0x01): left/right rotation
  - Pitch (CAN ID 0x02): up/down rotation

## Features

- **Dual control modes**:
  - **MIT mode**: Motion control with configurable KP/KD gains, torque feedforward
  - **CSP mode**: Cyclic Synchronous Position with speed/current limits
- **Homing on activation**: Smoothly interpolates from current position to zero over a configurable duration (default 3 seconds) to avoid sudden jumps
- **Dynamic parameter tuning**: KP/KD values per joint can be adjusted at runtime via ROS 2 parameters
- **Fake hardware support**: Can be replaced with `fake_components/GenericSystem` for simulation

## Hardware Parameters

Configured via the URDF ros2_control xacro:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can2` | CAN bus interface name |
| `can_fd` | `false` | Enable CAN-FD |
| `control_mode` | `mit` | `mit` or `csp` |
| `home_on_activate` | `true` | Enable slow homing on activation |
| `home_duration_sec` | `3.0` | Homing interpolation duration (seconds) |

## Interfaces

### Command Interfaces (per joint)

- `position` (radians)
- `velocity` (rad/s)
- `effort` (Nm)

### State Interfaces (per joint)

- `position` (radians)
- `velocity` (rad/s)
- `effort` (Nm)

## Dynamic Parameters

Available on the `/openarmx_head_hardware_params` node:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_openarmx_head_yaw_joint` | 100.0 | Position gain for yaw (MIT mode) |
| `kd_openarmx_head_yaw_joint` | 10.0 | Velocity gain for yaw (MIT mode) |
| `kp_openarmx_head_pitch_joint` | 100.0 | Position gain for pitch (MIT mode) |
| `kd_openarmx_head_pitch_joint` | 10.0 | Velocity gain for pitch (MIT mode) |

Adjust at runtime:

```bash
ros2 param set /openarmx_head_hardware_params kp_openarmx_head_pitch_joint 150.0
ros2 param set /openarmx_head_hardware_params kd_openarmx_head_pitch_joint 5.0
```

## Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_hardware
source install/setup.bash
```

## Dependencies

- `hardware_interface`
- `pluginlib`
- `rclcpp` / `rclcpp_lifecycle`
- `openarmx_can` (CAN communication library)

## Notes

- Motor direction mapping: yaw motor direction is inverted (-1.0) to match the URDF convention; pitch is direct (1.0).
- In MIT mode, the torque limit is explicitly set to the motor's maximum to ensure pitch can resist gravity.
- CSP mode uses a conservative speed limit of 1.0 rad/s for safety.
- The CAN interface must be configured and up before the hardware activates.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
