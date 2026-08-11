# imu_interface

ROS 2 driver for the Yost Labs **TSS-DL3** (3-Space Data Logger v3).

The `IMU_Accel` node streams corrected primary accelerometer data over USB and
publishes `imu_interface/ImuAcceleration` on topic `IMU_Acceleration` at a
configurable rate. Sensor values are converted from **g** to **m/s²**.

Uses the official [YostLabs Python API](https://github.com/YostLabs/3SpacePythonPackage).

## Dependencies

ROS packages are pulled in by `package.xml`. The vendor SDK is installed separately.

On Ubuntu 24.04 / Debian (PEP 668), plain `pip install` is blocked. Use one of:

**Option A — system install (simplest for ROS on the Pi):**

```bash
python3 -m pip install --break-system-packages yostlabs
```

**Option B — venv with system site packages (keeps ROS `rclpy` visible):**

```bash
python3 -m venv ~/venvs/yostlabs --system-site-packages
source ~/venvs/yostlabs/bin/activate
python3 -m pip install yostlabs
# activate this venv whenever you run the IMU node
```

Also requires `python3-serial` if not already pulled in by `yostlabs`.

## Build

```bash
cd /path/to/orl_ws
source /opt/ros/$ROS_DISTRO/setup.bash   # or your Jazzy underlay
colcon build --packages-select imu_interface
source install/setup.bash
```

## Run

Auto-detect USB sensor at 100 Hz:

```bash
ros2 launch imu_interface imu_accel.launch.py
```

Custom rate and port:

```bash
ros2 launch imu_interface imu_accel.launch.py rate_hz:=200.0 serial_port:=/dev/ttyACM0
```

Or run the executable directly:

```bash
ros2 run imu_interface imu_accel --ros-args -p rate_hz:=100.0
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate_hz` | `100.0` | Stream / publish frequency (Hz), max 2000 |
| `topic` | `IMU_Acceleration` | Output topic name |
| `frame_id` | `imu_link` | Header frame id |
| `serial_port` | `""` | Device path; empty = auto-detect |

## Message

`imu_interface/ImuAcceleration`:

```
std_msgs/Header header
float64 x   # m/s^2
float64 y
float64 z
```

## Inspect

Source the workspace first (`source ~/orl_ws/install/setup.bash`), then:

```bash
ros2 interface show imu_interface/msg/ImuAcceleration
ros2 topic echo /IMU_Acceleration
ros2 topic hz /IMU_Acceleration
```

If echo reports `The message type 'imu_interface/msg/ImuAcceleration' is invalid`, the package
is missing from that shell’s environment — rebuild and source as in **Build** above.
