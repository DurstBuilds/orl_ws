# encoder_interface

ROS 2 SPI driver for the Vishay **RAIK060I11318FB693** multi-turn absolute encoder (SPI simple frame, read-only).

## Build

```bash
cd /path/to/orl_ws
colcon build --packages-select encoder_interface
source install/setup.bash
```

## Raspberry Pi SPI bring-up

1. Enable SPI: `sudo raspi-config` → Interface Options → SPI, or add `dtparam=spi=on` to `/boot/firmware/config.txt`.
2. Wire encoder `/CS` to **CE1** (spidev0.**1**), plus SCLK, MOSI, MISO, GND. Supply **5 V ± 0.25 V** on VCC.
3. Verify the device node exists:

   ```bash
   ls -l /dev/spidev0.1
   ```

4. Optional bus test:

   ```bash
   spidev_test -D /dev/spidev0.1 -s 1000000
   ```

Perform encoder self-calibration after mechanical mounting (push-button on the sensor). Reads work at reduced accuracy until calibrated.

## Run

```bash
ros2 launch encoder_interface encoder.launch.py
```

Parameters: `spi_device`, `spi_speed_hz` (default 1 MHz), `poll_rate_hz` (default 100), `frame_id`, `cs_delay_us` (default 10).

## Verify

```bash
ros2 topic echo /encoder/state
ros2 topic hz /encoder/state
```

Rotate the shaft and confirm `position_counts`, `turn_count`, and `total_angle_rad` change. On good reads, `crc_valid` should be `true`. If CRC fails consistently, increase `cs_delay_us` or lower `spi_speed_hz`.

## Topic

| Topic | Type | Description |
|-------|------|-------------|
| `/encoder/state` | `encoder_interface/msg/EncoderState` | Raw counts, radians, status flags |
