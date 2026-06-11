# cubemars_v3_interface

ROS 2 CAN gateway for **CubeMars AK 3.0 firmware** actuators in MIT (force-control) mode.

Unlike `cm_interface` (legacy delta-position MIT on standard CAN IDs), this package uses V3
extended-frame IDs and the V3 MIT byte layout from the AK 3.0 product manual (sections 4.2 and 4.3.1).

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
2. CAN feedback set to **streaming** (periodic) mode with a feedback rate (e.g. 200–500 Hz).
   The gateway listens for upload frames; it does not poll for request-response replies.
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

MIT commands are sent **only when `motor_command` changes** (not retransmitted at a fixed rate).

## Topics (per namespace)

| Topic            | Type                          | Direction |
|------------------|-------------------------------|-----------|
| `motor_command`  | `motor_interfaces/MotorCommand` | subscribe |
| `motor_state`    | `motor_interfaces/MotorState`   | publish   |

## CAN framing (V3)

Commands and periodic feedback use **different** extended IDs:

| Direction | Arb ID formula | Example (drive_id=1) |
|-----------|----------------|----------------------|
| MIT command (gateway → motor) | `(0x08 << 8) \| drive_id` | `00000801` |
| Periodic feedback (motor → gateway) | `(0x29 << 8) \| drive_id` | `00002901` |

- **Command payload** (section 4.2): KP/KD first, then position, velocity, torque.
- **Feedback payload** (section 4.3.1): int16 position, velocity, current; byte temperature and error.

Verify with `candump can0`:

- Motor streams `00002901` continuously at the configured feedback rate.
- Gateway sends `00000801` only when you publish a new `motor_command`.

## Troubleshooting

- **No feedback**: Check drive ID, motor power, MIT mode, and **periodic** CAN feedback setting.
  Confirm `candump` shows `0000290N` frames from the motor.
- **MIT TX failed / ENOBUFS**: Usually bus saturation from repeated identical commands; this
  gateway deduplicates TX. Check wiring and termination if errors persist.
- **Wrong velocity/torque readings**: Feedback scaling constants in `mit_can_codec.hpp` may need
  tuning against your hardware; see manual section 4.3.1.
- **Debug unmatched frames**: `ros2 launch ... log_unmatched_frames:=true`
