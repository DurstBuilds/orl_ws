# Boom stack guide

This document covers [`boom_stack.launch.py`](../launch/boom_stack.launch.py), the full multi-motor boom bring-up with joystick teleop.

For package-wide topics, other launch files, and single-motor benches, see the [package README](../README.md).

## Prerequisites

1. Build and source the workspace:

   ```bash
   cd /path/to/your_ws
   colcon build --packages-select cm_interface motor_interfaces
   source install/setup.bash
   ```

2. Bring up SocketCAN (example):

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000 restart-ms 100
   sudo ip link set can0 up
   ```

   **`restart-ms 100`** (recommended on Raspberry Pi + MCP251x): after a bus-off, the kernel
   auto-restarts the controller ~100 ms later. Without it (`restart-ms 0`, common default),
   `can0` stays **BUS-OFF** until manual `down`/`up`, the gateway sees no feedback on any drive,
   and you get comm faults on all motors. With `restart-ms=100`, hard knee runs can recover
   invisibly on the bus while motion continues.

   Verify:

   ```bash
   ip -details link show can0 | grep -E 'restart-ms|can state'
   ```

   Optional helper (same commands):

   ```bash
   ros2 run cm_interface can0_up.sh
   ```

3. Install the `joy` package (`sudo apt install ros-${ROS_DISTRO}-joy` or from source).

4. Ensure each drive is powered, on the bus, and configured for the CAN IDs in [`launch/boom_motor_config.py`](../launch/boom_motor_config.py).

## Quick start

On Raspberry Pi / MCP251x, bring up CAN with auto-restart before launch:

```bash
ros2 run cm_interface can0_up.sh
ros2 launch cm_interface boom_stack.launch.py
```

Default mode: `use_can_gateway:=true` — one `can_gateway_node` on `can0`, plus per-namespace `motor_unwrapper_node` and `joint_translator_node`, plus `joy_node` and `boom_joystick_control_node`.

Verify:

```bash
ros2 node list
ros2 topic list | grep motor_state
ros2 topic hz /knee_motor/motor_state
```

Gateway startup logs should show each drive’s `can_id`, enable order, and `MIT feedback active`.

## Architecture

```text
                    +------------------+
                    |    joy_node      |
                    |    publishes     |
                    |      /joy        |
                    +--------+---------+
                             |
                    +--------v---------+
                    | boom_joystick_   |
                    | control_node     |
                    +--------+---------+
         joint_despos / hold_joint / soft_mode (per namespace)
                             |
    +------------------------+------------------------+
    |                        |                        |
+---v------------+   +-------v--------+   +-----------v----------+
| /knee_motor/   |   | /hip_motor/    |   | /wheel_motor1|2/     |
| translator +   |   | translator +   |   | translator +         |
| unwrapper      |   | unwrapper      |   | unwrapper              |
+---+------------+   +-------+--------+   +-----------+----------+
    | motor_command          |                        |
    +------------------------+------------------------+
                             |
                    +--------v---------+
                    | can_gateway_node |  (default)
                    |   one SocketCAN  |
                    +--------+---------+
                             |
                         CAN bus
```

| Node | Role |
|------|------|
| `can_gateway_node` | Single CAN socket; MIT TX/RX for all drives; if motors go down, full reconnect (enable + set-origin button sequence) until success |
| `motor_unwrapper_node` | Integrates wrapped position → `motor_total_position` |
| `joint_translator_node` | 200 Hz joint PD → `motor_command` deltas |
| `boom_joystick_control_node` | Joystick → `joint_despos`, `hold_joint`, `soft_mode` |
| `joy_node` | Reads Linux joystick → `/joy` |
| `motor_node_continuous` | Legacy only (`use_can_gateway:=false`); one CAN socket per node |

## Motor configuration (single source of truth)

Edit [`launch/boom_motor_config.py`](../launch/boom_motor_config.py) — **`BOOM_MOTOR_STACKS`**.

| Field | Description |
|-------|-------------|
| `ns` | ROS namespace (e.g. `knee_motor`). Teleop matches substrings `knee`, `wheel`, `hip`. |
| `gear_ratio` | Motor radians per joint radian (> 0). Must match translator and `namespace_gear_ratios`. |
| `motor_model` | `ak70_10`, `ak10_9`, or `ak80_64` |
| `can_id` | Unique standard CAN ID [0, 2047] |
| `joint_angle_limit_deg` | Translator clamp; `0` disables. Namespaces containing `hip` also use launch arg `hip_angle_limit_deg`. |
| `omega_max` | Optional per-motor default for translator speed cap: `auto` (profile) or motor rad/s. Overridable at launch via `namespace_omega_max`. |

Default stack:

| Namespace | Motor model | CAN ID | Gear ratio | Joint limit |
|-----------|-------------|--------|------------|-------------|
| `knee_motor` | ak80_64 | 4 | 1.6 | — (0 = off) |
| `hip_motor` | ak70_10 | 3 | 33.0 | ±90° (launch + entry) |
| `wheel_motor1` | ak10_9 | 1 | 1.0 | — |
| `wheel_motor2` | ak10_9 | 2 | 1.0 | — |

After editing, rebuild is **not** required for launch-only changes; re-run the launch file.

### Adding a motor

1. Append a dict to `BOOM_MOTOR_STACKS` with a **unique** `can_id`.
2. Pick a supported `motor_model` (or add a profile in `motor_mit_profile.hpp` and rebuild).
3. Name `ns` so teleop can drive it, or extend `boom_joystick_control.py`:
   - `knee` in name → right stick X
   - `wheel` in name → left stick X
   - `hip` in name → buttons 4/5
4. Set `omega_max` in the stack entry (or rely on launch `namespace_omega_max` / `omega_max` default).
5. Add `namespace_gear_ratios` and `namespace_omega_max` entries automatically via defaults, or pass at launch.
6. Wire the drive on the bench and launch.

**Important:** The `namespaces` launch argument only affects **teleop**. The gateway always uses the full `BOOM_MOTOR_STACKS` list. Overriding `namespaces:=foo` without editing the config leaves other drives on CAN but not on the joystick.

## Launch arguments

### CAN and mode

| Argument | Default | When to change |
|----------|---------|----------------|
| `use_can_gateway` | `true` | `false` for legacy per-motor nodes (debug only; multiple sockets on one bus can conflict) |
| `can_interface` | `can0` | Different SocketCAN device |

### Gateway (when `use_can_gateway:=true`)

| Argument | Default | Description |
|----------|---------|-------------|
| `gateway_loop_rate_hz` | 200.0 | MIT service loop rate |
| `gateway_startup_stagger_ms` | 200 | Delay between drive enable sequences |
| `gateway_enable_settle_ms` | 100 | Post-enable wait (non-AK80) |
| `gateway_ak80_enable_settle_ms` | 250 | Post-enable wait for AK80-64 knee |
| `gateway_startup_origin_poll_ms` | 100 | RX poll after set-origin at startup |
| `gateway_bus_warmup_ms` | 100 | Delay after CAN bind before first enable |
| `gateway_alive_check_period_ms` | 500 | How often the gateway checks that every drive has fresh MIT feedback |
| `gateway_reconnect_cooldown_ms` | 2000 | Minimum time between reconnect attempts (avoids spinning while power is off) |
| `motor_feedback_timeout_ms` | 250 | Stale feedback → comm fault Kd hold (until MIT replies resume) |
| `motor_feedback_poll_ms` | 5 | Blocking RX poll each gateway loop |

If any drive is stale at an alive-check, the gateway sets `motors_were_down` and **keeps running a full reconnect** on cooldown until that sequence succeeds (feedback alone does not clear the flag). Sequence: enable all → wait until connected → same as controller set-origin button (`soft_mode` true → wait → `soft_mode` false / set-origin+enable → `joint_despos=0` + `hold_joint=true`). If not all motors connect, set-origin is skipped and `motors_were_down` stays true.

### Translator (per namespace)

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace_omega_max` | from `boom_motor_config.py` | Per-namespace cap: `ns:auto` or `ns:<rad/s>`, comma-separated (e.g. `knee_motor:4.0,hip_motor:auto,wheel_motor1:15.0,wheel_motor2:15.0`) |
| `omega_max` | `auto` | Fallback for any namespace not listed in `namespace_omega_max` (`auto` uses each motor profile’s `omega_max`) |
| `motor_error_tolerance` | 0.001 | Motor-space goal/hold tolerance (rad), all translators |
| `hip_angle_limit_deg` | 90.0 | Hip teleop clamp and hip translator limit |

Example — slower knee, profile-default hip, faster wheels:

```bash
ros2 launch cm_interface boom_stack.launch.py \
  namespace_omega_max:="knee_motor:3.0,hip_motor:auto,wheel_motor1:20.0,wheel_motor2:20.0"
```

Check a running translator:

```bash
ros2 param get /knee_motor/joint_translator_node omega_max
```

### Teleop

| Argument | Default | Description |
|----------|---------|-------------|
| `joy_dev` | `0` | Joystick device index |
| `publish_hz` | 50.0 | Teleop timer rate |
| `namespaces` | from config | Comma-separated list for `boom_joystick_control_node` |
| `namespace_gear_ratios` | from config | `ns:ratio,...` for velocity scaling |

Velocity and axis mapping can also be changed at runtime:

```bash
ros2 param set /boom_joystick_control_node knee_velocity_constant 0.8
ros2 param set /boom_joystick_control_node stick_deadzone 0.2
```

### Legacy motors (`use_can_gateway:=false`)

| Argument | Default | Description |
|----------|---------|-------------|
| `motor_tx_rate_hz` | 200.0 | Per-motor MIT TX rate |
| `motor_startup_stagger_ms` | 800 | Delay before each motor opens CAN |

### Logging

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_IMU` | `false` | Launch `imu_accel` (`IMU_Accel`). Serial port auto-detects a Yost TSS USB sensor. |
| `enable_logging` | `false` | Launch `psu_telemetry` and record all stack `motor_state`, `motor_command`, `joint_curpos`, and `power_supply/*` topics. Also records `/IMU_Acceleration` when `enable_IMU` is true. |
| `psu_serial_port` | `/dev/ttyACM0` | PSU USB serial device (only when logging is on) |
| `bag_output_uri` | `boom_stack_bag` | Base name; auto-increments `_0`, `_1`, … |
| `bag_output_dir` | `.` | Directory for bag output |
| `bag_storage_id` | `mcap` | Rosbag storage plugin |

Example:

```bash
ros2 launch cm_interface boom_stack.launch.py enable_logging:=true bag_output_dir:=./bags
```

With IMU streaming and bag recording:

```bash
ros2 launch cm_interface boom_stack.launch.py enable_IMU:=true enable_logging:=true bag_output_dir:=./bags
```

## Teleop controls

| Input | Namespace contains | Action |
|-------|-------------------|--------|
| Right stick X | `knee` | Increment `joint_despos` |
| Left stick X | `wheel` | Increment `joint_despos` (both wheels share stick) |
| Button 5 (not 4) | `hip` | Positive increment |
| Button 4 (not 5) | `hip` | Negative increment |
| Button 1 (edge) | all listed | Toggle `soft_mode` |
| Button 2 / X (edge) | `knee` only | Toggle `soft_mode` (same re-latch as global soft stop) |
| Release input | all listed | `hold_joint=true`, snap despos to curpos |

## Topics (per namespace `/{ns}/`)

| Topic | Type | Description |
|-------|------|-------------|
| `motor_state` | `motor_interfaces/MotorState` | Wrapped position, velocity, torque |
| `motor_total_position` | `motor_interfaces/MotorTotalPosition` | Unwrapped cumulative position |
| `motor_command` | `motor_interfaces/MotorCommand` | MIT delta-P per tick + kp/kd |
| `joint_curpos` | `std_msgs/Float32` | Joint position (rad) |
| `joint_despos` | `std_msgs/Float32` | Desired joint position (rad) |
| `hold_joint` | `std_msgs/Bool` | `true` → translator holds |
| `soft_mode` | `std_msgs/Bool` | `true` → damping-only at motor; off triggers set-origin + despos latch |
| `origin_reset` | `std_msgs/Bool` | Gateway publishes `true` after set-origin; unwrapper resets total, translator re-latches despos |

Global: `/joy`. With `enable_IMU:=true`: `/IMU_Acceleration` (`imu_interface/ImuAcceleration`).

## Modes

### Gateway mode (recommended)

```bash
ros2 launch cm_interface boom_stack.launch.py
```

One CAN owner, staggered startup by `can_id`, AK80 knee enabled last.

### Legacy mode (debug)

```bash
ros2 launch cm_interface boom_stack.launch.py use_can_gateway:=false
```

Each namespace runs its own `motor_node_continuous`. Do not use multiple nodes on the same `can0` in production.

### Single-motor bench

Use [`boom_teleop.launch.py`](../launch/boom_teleop.launch.py) instead of the full stack.

### Gateway only

```bash
ros2 launch cm_interface can_gateway.launch.py
```

Defaults are imported from `boom_motor_config.py`.

## Tuning

| Goal | What to adjust |
|------|----------------|
| Faster/slower teleop | `knee_velocity_constant`, `wheel_velocity_constant`, `hip_velocity_constant` (÷ gear_ratio applied) |
| Finer position hold | Lower `motor_error_tolerance` |
| Hip travel limit | `hip_angle_limit_deg` (launch) |
| Bus timing / enable issues | `gateway_*_ms` arguments |
| Translator speed cap (per motor) | `namespace_omega_max` at launch, or `omega_max` in `BOOM_MOTOR_STACKS` / profile in `motor_mit_profile.hpp` |
| MIT stiffness | Profile `mit_kp`/`mit_kd` or translator overrides |

## CAN bus-off and auto-restart (Pi / MCP251x)

Under high knee torque and impacts, the adapter can enter **BUS-OFF** (`can state BUS-OFF` in
`ip -details link show can0`). The ROS gateway does not run `ip link` for you.

| Setup | Behavior |
|-------|----------|
| `restart-ms 0` | Bus stays off; manual `sudo ip link set can0 down/up` required; all motors comm-fault |
| `restart-ms 100` | Kernel auto-restarts from bus-off; often no visible issue; motors keep running |

Monitor auto-restarts:

```bash
watch -n0.5 'grep -E "restarts|busoff" /proc/net/can/stats'
```

The **`restarts`** counter increments on each automatic recovery. **`busoff`** counts bus-off events.
If both climb quickly during testing, fix termination, ground, and power — do not rely on restart alone.

Live error frames: `candump -ta -e can0` (second terminal while running).

## Troubleshooting

| Symptom | Likely cause | What to do |
|---------|--------------|------------|
| `can state BUS-OFF`, all motors comm fault | Host adapter left bus; `restart-ms 0` | `restart-ms 100` on `can0`; check wiring/power; `candump -e can0` |
| Gateway exits immediately | CAN interface down or wrong name | Check `ip link`; set `can_interface:=...` |
| `Motor initilization failed, check power and CAN wiring` | No MIT feedback from any drive after startup | Power, CAN bitrate, termination, `can_interface`, drive IDs; gateway will retry after `gateway_reconnect_cooldown_ms` |
| `Duplicate can_id` fatal | Config error | Fix `BOOM_MOTOR_STACKS` |
| `comm fault` on one drive | No feedback, wrong ID, cable, or motor power loss | Check `can_id`, power, termination; gateway reconnects all drives if the drive stays stale |
| `motor_command` deltas publish but no motion after reconnect | Drive stuck in comm fault (gateway ignored commands); often after MIT starvation during reinit | Check for `comm fault` logs; soft-mode / set-origin pulse recovers by sending Kd MIT. Rebuild if still sticky after this fix |
| `[RECONNECT] Motors were down` / `motors_were_down=true` | Feedback lost or init incomplete | Full reconnect keeps running on cooldown until set-origin button sequence succeeds; pressing Back manually does the same soft_mode pulse |
| `comm fault` immediately after `All motors successfully initiated` | Service loop was checking feedback before polling; refresh timestamps aged during a long poll | Rebuild `cm_interface` (gateway polls RX before each service tick; startup ends with a final MIT ping + short poll) |
| Motor moves but wrong joint | CAN ID collision or mismatch | Verify unique IDs and wiring |
| No joystick motion on new motor | Namespace lacks `knee`/`wheel`/`hip` | Rename or extend teleop script |
| `No motor_total_position` warn at start | Startup race | Wait for gateway feedback; usually clears |
| Teleop moves, CAN silent | `soft_mode` on or `hold_joint` true | Release sticks; toggle soft mode off |
| Hip stops at limit | Expected | `hip_angle_limit_deg`; move opposite direction |
| Bag empty / wrong topics | Logging off or wrong cwd | `enable_logging:=true`; set `bag_output_dir` |

### Soft mode sequence

1. Button 1 toggles `soft_mode` on all namespaces; button X toggles `soft_mode` on knee only.
2. Gateway sends low Kd; translator stops `motor_command`.
3. On toggle off: gateway set-origin, unwrapper resets total, translator re-latches despos.

## Related files

| File | Purpose |
|------|---------|
| [`launch/boom_motor_config.py`](../launch/boom_motor_config.py) | Motor list and derived defaults |
| [`launch/boom_stack.launch.py`](../launch/boom_stack.launch.py) | Full stack launch |
| [`launch/can_gateway.launch.py`](../launch/can_gateway.launch.py) | Gateway-only launch |
| [`scripts/boom_joystick_control.py`](../scripts/boom_joystick_control.py) | Teleop node |
| [`src/can_gateway_node.cpp`](../src/can_gateway_node.cpp) | CAN gateway implementation |
| [`include/cm_interface/motor_mit_profile.hpp`](../include/cm_interface/motor_mit_profile.hpp) | Motor profiles |
