# encoder_interface

ROS 2 SPI driver for the Vishay **RAIK060I11318FB693** multi-turn absolute encoder.

Reads position via SPI simple frame. At startup (default), sends SPI command **0x24**
(“Apply Zero position” offset) so the current shaft angle becomes origin; the offset is
stored in the encoder NVM.

## Build

```bash
cd /path/to/orl_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select encoder_interface
source install/setup.bash
```

## Raspberry Pi wiring (SPI1)

Default configuration matches this wiring:

| Encoder pin | Signal | Pi connection |
|-------------|--------|---------------|
| 1 | VCC | Pin 4 (5 V) |
| 2 | SCLK | GPIO21 (SPI1 SCLK) |
| 3 | `/CS` | GPIO16 (SPI1 CE2) |
| 4 | MOSI | GPIO20 (SPI1 MOSI) |
| 5 | MISO | GPIO19 (SPI1 MISO) |
| 6 | GND | Pin 6 (GND) |

**GPIO16 works as chip select** — it is SPI1 CE2, exposed as `/dev/spidev1.2`. You do not need to move `/CS` unless you prefer a different CE line (GPIO18 = CE0 → `spidev1.0`, GPIO17 = CE1 → `spidev1.1`).

## Device tree (required for SPI1)

SPI1 is disabled by default. Add to `/boot/firmware/config.txt`:

```text
dtparam=spi=on
dtoverlay=spi1-3cs
```

`spi1-3cs` is required for GPIO16 (CE2). Reboot, then verify:

```bash
ls -l /dev/spidev1.2
```

Optional bus test:

```bash
spidev_test -D /dev/spidev1.2 -s 1000000
```

Perform encoder self-calibration after mechanical mounting (push-button on the sensor). Reads work at reduced accuracy until calibrated.

## Run

```bash
ros2 launch encoder_interface encoder.launch.py
```

Parameters: `spi_device` (default `/dev/spidev1.2`), `spi_speed_hz` (default 500 kHz), `poll_rate_hz` (default 100), `frame_id`, `cs_delay_us` (default 20), `apply_zero_position_on_startup` (default true — SPI cmd `0x24`), `log_raw_frames` (default false).

Skip zeroing:

```bash
ros2 launch encoder_interface encoder.launch.py apply_zero_position_on_startup:=false
```

## Troubleshooting CRC errors

If you see `Encoder CRC invalid (turn=0 pos=0)`:

1. Rebuild and redeploy — an earlier version clocked 8 spurious bits before the frame read.
2. Confirm SPI1 is enabled and the device exists: `ls -l /dev/spidev1.2`
3. Launch with raw SPI logging:

   ```bash
   ros2 launch encoder_interface encoder.launch.py log_raw_frames:=true
   ```

   All-zero raw bytes (`00 00 00 00 00 00`) usually means MISO is not connected or the wrong spidev device.
4. Try slower SPI and longer CS delay:

   ```bash
   ros2 launch encoder_interface encoder.launch.py spi_speed_hz:=250000 cs_delay_us:=50
   ```

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
