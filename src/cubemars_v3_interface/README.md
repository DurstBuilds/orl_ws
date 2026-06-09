# cubemars_v3_interface

ROS 2 CAN gateway for **CubeMars AK 3.0 firmware** actuators in MIT (force-control) mode.

Unlike `cm_interface` (legacy delta-position MIT on standard CAN IDs), this package uses V3
extended-frame IDs and the V3 MIT byte layout from the AK 3.0 product manual (section 4.2).

## Supported motors

| `motor_model` | Motor   |
|---------------|---------|
| `ak60_6`      | AK60-6  |

Add profiles in `include/cubemars_v3_interface/motor_mit_profile.hpp`.

## Build

```bash
cd /path/to/orl_ws
colcon build --packages-select cubemars_v3_interface motor_interfaces
source install/setup.bash
```

## Prerequisites

1. Motor configured in **MIT / force-control mode** via the CubeMars upper computer.
2. CAN feedback set to **query-reply** (request-response) mode.
3. SocketCAN up at 1 Mbps:

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000 restart-ms 100
   sudo ip link set can0 up
   ```

## Quick start (manual control)

```bash
ros2 launch cubemars_v3_interface can_gateway.launch.py can_ids:=<YOUR_DRIVE_ID>
```

Publish an absolute-position MIT command (radians):

```bash
ros2 topic pub --once /motor/motor_command motor_interfaces/msg/MotorCommand \
  "{position: 1.0, velocity: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}"
```

Monitor feedback:

```bash
ros2 topic echo /motor/motor_state
```

`MotorCommand.position` is **absolute position in rad**, not a per-tick delta.

## Topics (per namespace)

| Topic            | Type                          | Direction |
|------------------|-------------------------------|-----------|
| `motor_command`  | `motor_interfaces/MotorCommand` | subscribe |
| `motor_state`    | `motor_interfaces/MotorState`   | publish   |

## CAN framing (V3)

- Extended 29-bit ID: `(8 << 8) | drive_id` with `CAN_EFF_FLAG` (MIT control mode = 8).
- Payload: KP/KD first, then position, velocity, torque (see `mit_can_codec.hpp`).

Verify with `candump can0` — expect extended frames like `00000801` for drive ID 1.

## Troubleshooting

- **No feedback**: Check drive ID, motor power, MIT mode, and query-reply CAN setting.
- **Wrong velocity/torque readings**: Feedback scaling constants in `mit_can_codec.hpp` may need
  tuning against your hardware; see manual section 4.2.
