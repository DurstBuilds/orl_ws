# cm_interface

ROS 2 interface for AK-series actuators over CAN using the MIT control mode. The package provides motor drivers, joint-space teleoperation, position unwrapping, and a PD outer loop that converts joint targets into per-tick motor position commands.

## Overview

Each motor stack (optionally in its own namespace) contains three cooperating nodes:

```
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

- **can_gateway_node** — (boom stack default) Single SocketCAN socket for all drives; routes RX/TX by `can_id`; same per-namespace topics as `motor_node_continuous`.
- **motor_node_continuous** — One drive per process (legacy / single-motor benches); opens its own SocketCAN interface.
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
| **boom_stack.launch.py** | Full boom: knee, hip, two wheels, one `joy_node`, one `boom_joystick_control_node`. Optional rosbag logging and joint position test sequence. |
| **boom_teleop.launch.py** | Single namespaced stack + joy + boom teleop (for one motor at a time). |
| **motor_stack.launch.py** | Single stack + original `joystick_control` (deadman, right-stick absolute position). |
| **motor_node_continuous.launch.py** | Motor node only. |
| **can_gateway.launch.py** | CAN gateway only (boom drive list). |
| **joint_translator.launch.py** | Translator + unwrapper only (motor node launched separately). |
| **joystick_teleop.launch.py** | Teleop nodes only (deadman + right-stick absolute position). |
| **joint_position_sequence.launch.py** | Joystick-triggered preset joint waypoint runner |

### Full boom stack (recommended)

```bash
ros2 launch cm_interface boom_stack.launch.py
```

**See [docs/BOOM_STACK.md](docs/BOOM_STACK.md)** for architecture, adding motors (`launch/boom_motor_config.py`), launch arguments, teleop controls, tuning, and troubleshooting.

On Raspberry Pi / MCP251x, bring up CAN with auto-restart before launch: `ros2 run cm_interface can0_up.sh` (sets `restart-ms 100`).

Default: `use_can_gateway:=true` (one gateway on `can0`) and four drives from `boom_motor_config.py` (knee, hip, two wheels). Legacy per-motor CAN: `use_can_gateway:=false`.

Optional joint test sequence: `enable_joint_sequence:=true` (see below).

### Joint position test sequence

Runs preset `joint_despos` waypoints when you press **Start** (button 7) on the gamepad. Between waypoints the node waits until all listed joints settle within tolerance, then applies the per-step `delay_sec` from the YAML file.

```bash
ros2 launch cm_interface boom_stack.launch.py \
  enable_joint_sequence:=true \
  joint_sequence:=KneeTumble
```

Or launch the sequence node separately after `boom_stack`:

```bash
ros2 launch cm_interface joint_position_sequence.launch.py
```

Edit presets in `config/joint_sequence_presets.yaml` (installed under `share/cm_interface/config/`). Select one with `joint_sequence:=<PresetName>` (e.g. `KneeTumble`, `HipSuperman`). Positions are in **degrees** per namespace (`knee_motor`, `hip_motor`, `wheel_motor1`, `wheel_motor2`). While a sequence runs, `boom_joystick_control` pauses `joint_despos` / `hold_joint` publishing (`/joint_sequence/active` is true). Press Start again to abort. The sequence will not start if any joint is in `soft_mode`.

Launch args (when using `boom_stack`): `sequence_file`, `joint_sequence`, `joint_sequence_start_button` (default 7), `joint_sequence_loop`.

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
| Button 2 / X (edge) | `knee` | Toggle `soft_mode` (knee only; same re-latch as global soft stop) |
| Release input | all | `hold_joint=true`, snap `joint_despos` to `joint_curpos` |

Hip `joint_despos` is clamped to ±`hip_angle_limit_deg` (default 90°) in teleop. The translator applies the same limit and limit-hold logic for the hip namespace.

Per-motor gear ratios in **boom_stack** use `namespace_gear_ratios`, e.g. `knee_motor:1.6,hip_motor:33,...` (defaults from `boom_motor_config.py`).

## Topics (per namespace `/{ns}/`)

| Topic | Type | Description |
|-------|------|-------------|
| `motor_state` | `motor_interfaces/MotorState` | Wrapped position, velocity, torque |
| `motor_total_position` | `motor_interfaces/MotorTotalPosition` | Unwrapped cumulative position |
| `motor_command` | `motor_interfaces/MotorCommand` | MIT command: `position` = delta-P per tick, plus `kp`, `kd` |
| `joint_curpos` | `std_msgs/Float32` | Joint position (rad) |
| `joint_despos` | `std_msgs/Float32` | Desired joint position (rad) |
| `hold_joint` | `std_msgs/Bool` | `true` = translator holds (`deltaP=0`) |
| `soft_mode` | `std_msgs/Bool` | `true` = motor damping-only MIT mode (set-origin + despos latch on off) |

Global: `/joy` from `joy_node`.

## Soft mode

When `soft_mode` is true:

- **motor_node_continuous** sends low-stiffness MIT (Kd-only style hold).
- **joint_translator_node** stops publishing `motor_command`.

Toggle from boom teleop (button 1 for all joints, button X for knee only). On soft-mode off, despos is re-pinned to current position after gateway set-origin and unwrapper reset.

## joint_translator_node behavior

- **Gear ratio**: `motor_rad / joint_rad`; joint error is converted to motor total position error for PD.
- **Hold sources**: teleop `hold_joint`, goal reached (motor-space tolerance), crossover of desired position, predictive hold (next `deltaP` would reach goal), hip angle limit.
- **Parameters** (common):
  - `motor_model`, `gear_ratio`, `loop_hz` (default 200)
  - `motor_error_tolerance` (motor rad, default 0.001)
  - `joint_angle_limit_deg` (0 = disabled; hip stack uses 90°)
  - `omega_max` (`auto` or rad/s cap on per-tick delta). In **boom_stack**, use `namespace_omega_max` for per-namespace values (see [docs/BOOM_STACK.md](docs/BOOM_STACK.md)).
  - `mit_kp`, `mit_kd` (override profile defaults)

## motor_node_continuous

- **Parameters**: `motor_model`, `can_id`, `can_interface` (default `can0`), `tx_rate_hz` (default 200), `feedback_timeout_ms` (default 250), `feedback_poll_ms` (default 5).
- Opens one SocketCAN socket per process; optional kernel filters for this drive’s ID.
- TX timer sends the latest `motor_command` each tick; each TX is followed by a blocking feedback poll (capped at the TX period).
- On comm fault (stale feedback), commands are forced to zero until feedback recovers.
- Expects CAN interface up and drives configured for MIT mode.

## Logging (boom_stack)

See [docs/BOOM_STACK.md](docs/BOOM_STACK.md#logging) for `enable_logging`, bag paths, and recorded topics.

## Other executables

| Node / script | Role |
|---------------|------|
| `keyboard_command` | Numpad absolute position commands via `motor_command` |
| `joystick_control_node` | Deadman teleop with right-stick absolute angle |
| `joint_position_sequence_node` | Start-button preset joint waypoint test sequence |

Manual keyboard teleop (remap namespace as needed):

```bash
ros2 run cm_interface keyboard_command --ros-args -r __ns:=/hip_motor
```

## Example: monitor hip

```bash
ros2 topic echo /hip_motor/joint_curpos
ros2 topic echo /hip_motor/motor_state
ros2 topic hz /hip_motor/motor_state
```

## Package layout

```text
cm_interface/
├── config/          # joint_sequence_presets.yaml
├── docs/BOOM_STACK.md
├── include/cm_interface/motor_mit_profile.hpp
├── launch/          # boom_stack, boom_motor_config, boom_teleop, ...
├── scripts/         # boom_joystick_control.py, joint_position_sequence.py, ...
└── src/             # can_gateway_node, joint_translator_node, motor_unwrapper_node, ...
```

## Troubleshooting

- **No `joint_curpos`**: Ensure `motor_total_position` is publishing (unwrapper needs `motor_state`).
- **Motor does not move**: Check `hold_joint`, `soft_mode`, and that `joint_despos` differs from `joint_curpos` beyond `motor_error_tolerance`.
- **Hip overshoot at limit**: Confirm `joint_angle_limit_deg` on hip translator and `hip_angle_limit_deg` on teleop; reduce `hip_velocity_constant` or increase `publish_hz`.
- **CAN errors**: Verify `can_id`, interface name, drive power/enable, and that `can0` has `restart-ms 100` (see `can0_up.sh` and [docs/BOOM_STACK.md](docs/BOOM_STACK.md)).
- **Motors powered after Pi boot**: `can_gateway_node` stays alive in standby and retries connect every `gateway_standby_retry_ms` (default 5 s). Toggle soft_mode off (button 1) once all drives report feedback.

## Raspberry Pi boot setup

Use this section to bring up SocketCAN and `boom_stack.launch.py` automatically when the Pi powers on. Boot order matters: **CAN first, then the ROS stack**.

The gateway defaults to **standby reconnect** (`gateway_standby_retry_ms:=5000`) and **soft_mode on all drives** (`start_in_soft_mode:=true`) so the stack can start before motor power is applied. See [docs/BOOM_STACK.md](docs/BOOM_STACK.md) for tuning and troubleshooting.

### 1. MCP251x device tree (one-time)

Edit `/boot/firmware/config.txt` (Bookworm) or `/boot/config.txt` (older images) and enable your CAN HAT overlay. Example for a common MCP2515 on SPI:

```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

Reboot, then confirm the interface exists (it may still be down):

```bash
ip link show can0
```

Adjust `oscillator` and `interrupt` to match your HAT wiring.

### 2. Passwordless `sudo` for `ip link` (recommended)

[`scripts/can0_up.sh`](scripts/can0_up.sh) runs `sudo ip link` to set bitrate and `restart-ms`. For systemd at boot, allow your ROS user to run it without a password:

```bash
sudo visudo -f /etc/sudoers.d/can0-up
```

Add **one line only** — use your Linux username with **no** angle brackets. Example for user `cubemarspi`:

```text
cubemarspi ALL=(ALL) NOPASSWD: /sbin/ip
```

Wrong (causes `syntax error`): `<cubemarspi> ALL=(ALL) NOPASSWD: /sbin/ip`

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X` in nano). Validate:

```bash
sudo visudo -c -f /etc/sudoers.d/can0-up
```

You should see `parsed OK`. If you already broke sudo, fix as root:

```bash
sudo rm /etc/sudoers.d/can0-up
sudo visudo -f /etc/sudoers.d/can0-up
# paste the correct line (no <>), save, then re-run visudo -c above
```

### 3. systemd: bring up CAN at boot

Use a **simple root service** with direct `ip` commands (no ROS workspace required). System unit files must be created with `sudo`:

```bash
sudo tee /etc/systemd/system/can0-up.service > /dev/null <<'EOF'
[Unit]
Description=Bring up SocketCAN can0 (1 Mbit/s, restart-ms 100)
After=network-pre.target
Before=boom-stack.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '/sbin/ip link set can0 down 2>/dev/null || true; /sbin/ip link set can0 type can bitrate 1000000 restart-ms 100; /sbin/ip link set can0 up'
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
EOF
```

Manual equivalent (what the unit runs):

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

Enable and test:

```bash
sudo systemctl daemon-reload
sudo systemctl enable can0-up.service
sudo systemctl start can0-up.service
systemctl status can0-up.service
ip -details link show can0 | grep -E 'state|can state|restart-ms|bitrate'
```

Success looks like `state UP` and `can state ERROR-ACTIVE` (or `ERROR-PASSIVE`), not `STOPPED`.

**If `can0-up.service` fails**, read the log:

```bash
journalctl -xeu can0-up.service --no-pager
```

Common causes:

| Log message | Fix |
|-------------|-----|
| `No such file or directory` on `source .../install/setup.bash` | Old unit still used `ros2 run`; replace with the simple unit above |
| Literal `<WORKSPACE>` or `<USER>` in the unit | Re-create the file with `sudo tee`; do not leave `<>` placeholders |
| `Cannot find device "can0"` | Enable MCP251x overlay in `config.txt` and reboot |
| `ip: command not found` | Use full path `/sbin/ip` (already in unit above) |
| `RTNETLINK answers: Operation not supported` | Wrong overlay / oscillator / SPI wiring |

Optional: run the installed helper instead of raw `ip` (requires built workspace):

```bash
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/cubemarspi/orl_ws/install/setup.bash && ros2 run cm_interface can0_up.sh can0 1000000 100'
```

Use real paths; run as `User=cubemarspi` only if passwordless sudo for `/sbin/ip` is configured (section 2).

### 4. systemd: launch boom stack at boot

`WorkingDirectory` and all `source` paths must be **full absolute paths** (start with `/`). Wrong: `orl_ws`, `<WORKSPACE>`. Right: `/home/cubemarspi/orl_ws`.

Find your workspace path:

```bash
echo "$HOME/orl_ws"
ls "$HOME/orl_ws/install/setup.bash"
```

Create the unit (edit paths if your workspace or ROS distro differs):

```bash
sudo tee /etc/systemd/system/boom-stack.service > /dev/null <<'EOF'
[Unit]
Description=ROS 2 boom stack (can_gateway + teleop)
After=network-online.target can0-up.service
Wants=network-online.target can0-up.service

[Service]
Type=simple
User=cubemarspi
WorkingDirectory=/home/cubemarspi/orl_ws
Environment=ROS_DISTRO=jazzy
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=20
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/cubemarspi/orl_ws/install/setup.bash && exec ros2 launch cm_interface boom_stack.launch.py enable_logging:=false'
ExecStop=/home/cubemarspi/orl_ws/install/cm_interface/lib/cm_interface/boom_stack_stop.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF
```

Use **`exec ros2 launch`** so `ros2 launch` is the service main process (not `bash`). **`ExecStop`** runs [`boom_stack_stop.sh`](scripts/boom_stack_stop.sh) because C++ nodes (`can_gateway_node`, translators, unwrappers) often keep running after launch exits under systemd (`Unit process … remains running after unit stopped`).

After updating the unit, install the stop script, kill orphans once, then test stop:

```bash
cd ~/orl_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cm_interface
chmod +x install/cm_interface/lib/cm_interface/boom_stack_stop.sh

pkill -f 'ros2 launch cm_interface boom_stack' || true
pkill -x can_gateway_node || true
pkill -x joint_translator_node || true
pkill -x motor_unwrapper_node || true

sudo systemctl daemon-reload
sudo systemctl start boom-stack.service
sudo systemctl stop boom-stack.service
ros2 node list   # should be empty
```

Validate before enabling:

```bash
systemd-analyze verify /etc/systemd/system/boom-stack.service
```

Adjust launch arguments as needed (`joy_dev:=0`, `gateway_standby_retry_ms:=5000`, `start_in_soft_mode:=true` are defaults).

Enable and test:

```bash
sudo systemctl daemon-reload
sudo systemctl enable boom-stack.service
sudo systemctl start boom-stack.service
journalctl -u boom-stack -f
```

Useful checks after boot:

```bash
ros2 topic echo /knee_motor/soft_mode --once
ros2 topic hz /knee_motor/motor_state
systemctl status can0-up boom-stack
```

If the gateway logs `[STANDBY] CAN interface ... not available`, fix `can0-up.service` first. If CAN is up but motors are off, expect `[STANDBY] Connecting drives ...` every few seconds until power is applied; then press button 1 to exit soft_mode when ready to move.
