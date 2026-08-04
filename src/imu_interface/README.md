# imu_interface

ROS 2 driver for the Yost Labs **TSS-DL3** (3-Space Data Logger v3).

The `IMU_Accel` node streams corrected primary accelerometer data over USB and
publishes `imu_interface/ImuAcceleration` on topic `IMU_Acceleration` at a
configurable rate. Sensor values are converted from **g** to **m/s²**.

Uses the official [YostLabs Python API](https://github.com/YostLabs/3SpacePythonPackage).

## Dependencies

ROS packages are pulled in by `package.xml`. The vendor SDK is installed separately:

```bash
python3 -m pip install yostlabs
```

Also requires `python3-serial` (pulled in by `yostlabs` / system packages).

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

```bash
ros2 topic echo /IMU_Acceleration
ros2 topic hz /IMU_Acceleration
```
