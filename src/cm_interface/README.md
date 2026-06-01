# cm_interface

ROS 2 interface for AK-series actuators over CAN using the MIT control mode. The package provides motor drivers, joint-space teleoperation, position unwrapping, and a PD outer loop that converts joint targets into per-tick motor position commands.

## Overview

Each motor stack (optionally in its own namespace) contains three cooperating nodes:

```text
Teleop (joystick)          joint_translator_node          motor_node_continuous
     |                              |                                |
 joint_despos  ----------------->  motor_command  ---------------->  CAN / MIT
 hold_joint                         ^                                |
 soft_mode                          |                                v
     ^                         motor_total_position  <--------  motor_state
     |                              ^
 joint_curpos  <---------------------+
                              motor_unwrapper_node
```

- **motor_node_continuous** — Opens a SocketCAN interface, sends MIT frames, publishes `motor_state`, subscribes to `motor_command` (delta position per tick) and `soft_mode`.
- **motor_unwrapper_node** — Integrates wrapped motor position into `motor_total_position` for multi-turn tracking.
- **joint_translator_node** — Runs a 200 Hz PD loop in motor space: subscribes to `joint_despos`, publishes `joint_curpos` and `motor_command`. Handles goal hold, predictive hold, and optional joint angle limits (hip).

Teleop nodes publish `joint_despos` in **joint radians** and `hold_joint` (release-to-hold). Velocity constants are scaled by `1/gear_ratio` so motor-space motion stays consistent across gear reductions.

## Supported motors

Set `motor_model` to one of:

| Parameter value | Motor   | Notes                          |
|-----------------|---------|--------------------------------|
| `ak70_10`       | AK70-10 | Default profile                |
| `ak10_9`        | AK10-9  | Used for wheel motors in stack |
| `ak80_64`       | AK80-64 | Used for knee in stack         |

Profiles in `include/cm_interface/motor_mit_profile.hpp` define CAN scaling, default MIT `kp`/`kd`, translator PD gains, and `omega_max`.

## Build

```bash
cd /path/to/your_ws
colcon build --packages-select cm_interface motor_interfaces
source install/setup.bash
```

Requires: `rclcpp`, `rclpy`, `std_msgs`, `sensor_msgs`, `motor_interfaces`, `joy` (for teleop launches).

## Launch files

| Launch file | Purpose |
|-------------|---------|
| **boom_stack.launch.py** | Full boom: knee, hip, two wheels, one `joy_node`, one `boom_joystick_control_node`. Optional rosbag logging. |
| **boom_teleop.launch.py** | Single namespaced stack + joy + boom teleop (for one motor at a time). |
| **motor_stack.launch.py** | Single stack + original `joystick_control` (deadman, right-stick absolute position). |
| **motor_node_continuous.launch.py** | Motor node only. |
| **joint_translator.launch.py** | Translator + unwrapper only (motor node launched separately). |
| **joystick_teleop.launch.py** / **joystick_control.launch.py** | Teleop nodes only. |

### Full boom stack (recommended)

```bash
ros2 launch cm_interface boom_stack.launch.py
```

Default motors:

| Namespace      | Motor model | CAN ID | Gear ratio | Joint limit |
|----------------|-------------|--------|------------|-------------|
| `knee_motor`   | ak80_64     | 4      | 1.6        | —           |
| `hip_motor`    | ak70_10     | 0*     | 30         | ±45°        |
| `wheel_motor1` | ak10_9      | 1      | 1.0        | —           |
| `wheel_motor2` | ak10_9      | 2      | 1.0        | —           |

\* Hip `can_id` in launch file may differ on your bench; override as needed.

Useful arguments:

```bash
ros2 launch cm_interface boom_stack.launch.py enable_logging:=true
ros2 launch cm_interface boom_stack.launch.py joy_dev:=0 publish_hz:=50
ros2 launch cm_interface boom_stack.launch.py bag_output_dir:=./bags
```

### Single motor (boom teleop)

```bash
ros2 launch cm_interface boom_teleop.launch.py \
  ns:=hip_motor gear_ratio:=30 motor_model:=ak70_10 can_id:=3
```

### Single motor (classic joystick stack)

```bash
ros2 launch cm_interface motor_stack.launch.py \
  ns:=my_motor gear_ratio:=10 motor_model:=ak70_10 can_id:=0
```

Requires deadman held; right stick sets absolute `joint_despos`.

## Boom teleop controls

`boom_joystick_control_node` maps inputs by **namespace name** (substring match):

| Input | Namespace contains | Action |
|-------|-------------------|--------|
| Right stick X | `knee` | `joint_despos += axis * (knee_velocity_constant / gear_ratio)` |
| Left stick X | `wheel` | Same for wheel velocity constant |
| Button 5 held (no btn 4) | `hip` | Positive increment |
| Button 4 held (no btn 5) | `hip` | Negative increment |
| Button 1 (edge) | all | Toggle `soft_mode` |
| Release input | all | `hold_joint=true`, snap `joint_despos` to `joint_curpos` |

Hip `joint_despos` is clamped to ±`hip_angle_limit_deg` (default 45°) in teleop. The translator applies the same limit and limit-hold logic for the hip namespace.

Per-motor gear ratios in **boom_stack** use `namespace_gear_ratios`, e.g. `knee_motor:1.6,hip_motor:30,...`.

## Topics (per namespace `/{ns}/`)

| Topic | Type | Description |
|-------|------|-------------|
| `motor_state` | `motor_interfaces/MotorState` | Wrapped position, velocity, torque |
| `motor_total_position` | `motor_interfaces/MotorTotalPosition` | Unwrapped cumulative position |
| `motor_command` | `motor_interfaces/MotorCommand` | MIT command: `position` = delta-P per tick, plus `kp`, `kd` |
| `joint_curpos` | `std_msgs/Float32` | Joint position (rad) |
| `joint_despos` | `std_msgs/Float32` | Desired joint position (rad) |
| `hold_joint` | `std_msgs/Bool` | `true` = translator holds (`deltaP=0`) |
| `soft_mode` | `std_msgs/Bool` | `true` = motor damping-only MIT mode |

Global: `/joy` from `joy_node`.

## Soft mode

When `soft_mode` is true:

- **motor_node_continuous** sends low-stiffness MIT (Kd-only style hold).
- **joint_translator_node** stops publishing `motor_command`.

Toggle from boom teleop (button 1). On soft-mode off, despos is re-pinned to current position to avoid chasing a stale target.

## joint_translator_node behavior

- **Gear ratio**: `motor_rad / joint_rad`; joint error is converted to motor total position error for PD.
- **Hold sources**: teleop `hold_joint`, goal reached (motor-space tolerance), crossover of desired position, predictive hold (next `deltaP` would reach goal), hip angle limit.
- **Parameters** (common):
  - `motor_model`, `gear_ratio`, `loop_hz` (default 200)
  - `motor_error_tolerance` (motor rad, default 0.001)
  - `joint_angle_limit_deg` (0 = disabled; hip stack uses 45°)
  - `omega_max` (`auto` or rad/s cap on per-tick delta)
  - `mit_kp`, `mit_kd` (override profile defaults)

## motor_node_continuous

- **Parameters**: `motor_model`, `can_id`, `max_torque` (Nm; clamps `|kp * deltaP|`).
- Expects CAN interface up (e.g. `can0`) and drives configured for MIT mode.

## Logging (boom_stack)

```bash
ros2 launch cm_interface boom_stack.launch.py enable_logging:=true
```

Records all stack `motor_state` topics. Bags auto-increment: `boom_stack_bag_0`, `boom_stack_bag_1`, … under `bag_output_dir` (default: launch working directory).

| Argument | Default | Description |
|----------|---------|-------------|
| `bag_output_uri` | `boom_stack_bag` | Base name for numbered bags |
| `bag_output_dir` | `.` | Directory to scan and write bags |
| `bag_storage_id` | `mcap` | Rosbag storage format |

## Other executables

| Node / script | Role |
|---------------|------|
| `motor_node` | Legacy/alternate motor node |
| `can_test_node` | CAN bring-up / test |
| `keyboard_command` | Numpad absolute position commands via `motor_command` |
| `terminal_motor_command` / `terminal_position_publisher` | Simple CLI publishers |
| `joystick_control_node` | Deadman teleop with right-stick absolute angle |

## Example: monitor hip

```bash
ros2 topic echo /hip_motor/joint_curpos
ros2 topic echo /hip_motor/motor_state
ros2 topic hz /hip_motor/motor_state
```

## Package layout

```text
cm_interface/
├── include/cm_interface/motor_mit_profile.hpp
├── launch/          # boom_stack, boom_teleop, motor_stack, ...
├── scripts/         # boom_joystick_control.py, joystick_control.py, keyboard_command.py
└── src/             # motor_node_continuous, joint_translator_node, motor_unwrapper_node, ...
```

## Troubleshooting

- **No `joint_curpos`**: Ensure `motor_total_position` is publishing (unwrapper needs `motor_state`).
- **Motor does not move**: Check `hold_joint`, `soft_mode`, and that `joint_despos` differs from `joint_curpos` beyond `motor_error_tolerance`.
- **Hip overshoot at limit**: Confirm `joint_angle_limit_deg` on hip translator and `hip_angle_limit_deg` on teleop; reduce `hip_velocity_constant` or increase `publish_hz`.
- **CAN errors**: Verify `can_id`, interface name in motor node source, and drive power/enable.
