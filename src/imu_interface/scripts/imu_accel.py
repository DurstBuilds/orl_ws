#!/usr/bin/env python3
"""Stream corrected acceleration from a Yost Labs TSS-DL3 and publish as ImuAcceleration."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from imu_interface.msg import ImuAcceleration

try:
    from yostlabs.tss3 import ThreespaceSensor, StreamableCommands
    from yostlabs.communication.serial import ThreespaceSerialComClass
    from yostlabs.tss3.utils.streaming import (
        ThreespaceStreamingManager,
        ThreespaceStreamingStatus,
    )
    from yostlabs.tss3.errors import DiscoveryError, SensorConnectionError, ThreespaceError
except ImportError as exc:
    raise SystemExit(
        'yostlabs is required. Install with: python3 -m pip install yostlabs'
    ) from exc

_G_TO_MS2 = 9.80665
_MAX_STREAM_HZ = 2000.0


def _accel_components(value) -> tuple[float, float, float] | None:
    """Normalize yostlabs accel payloads to (x, y, z) floats in g."""
    if value is None:
        return None
    if hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
        return float(value.x), float(value.y), float(value.z)
    try:
        x, y, z = value
        return float(x), float(y), float(z)
    except (TypeError, ValueError):
        return None


class ImuAccelNode(Node):
    """ROS node: TSS-DL3 corrected accel in, ImuAcceleration out."""

    def __init__(self) -> None:
        super().__init__('IMU_Accel')

        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('topic', 'IMU_Acceleration')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('serial_port', '')

        rate_hz = float(self.get_parameter('rate_hz').value)
        topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        serial_port = str(self.get_parameter('serial_port').value).strip()

        if rate_hz <= 0.0 or rate_hz > _MAX_STREAM_HZ:
            self.get_logger().error(
                f'Parameter "rate_hz" must be in (0, {_MAX_STREAM_HZ:.0f}].'
            )
            raise SystemExit(1)

        stream_hz = max(1, int(round(rate_hz)))
        self._sensor = None
        self._manager = None
        self._owner = object()

        try:
            if serial_port:
                self._sensor = ThreespaceSensor(serial_port)
                port_desc = serial_port
            else:
                self._sensor = ThreespaceSensor(ThreespaceSerialComClass)
                port_desc = getattr(self._sensor.com, 'name', 'auto-detect')
        except (DiscoveryError, SensorConnectionError, ThreespaceError, OSError, ValueError) as exc:
            self.get_logger().error(f'Failed to open TSS-DL3: {exc}')
            raise SystemExit(1) from exc

        self._publisher = self.create_publisher(ImuAcceleration, topic, 10)

        self._manager = ThreespaceStreamingManager(self._sensor)
        if not self._manager.register_command(
            self._owner, StreamableCommands.GetPrimaryCorrectedAccelVec
        ):
            self.get_logger().error('Failed to register GetPrimaryCorrectedAccelVec stream slot.')
            self._cleanup_sensor()
            raise SystemExit(1)

        self._manager.register_callback(self._on_stream, hz=stream_hz)
        self._manager.enable()

        # Drain serial often enough to keep up with the configured stream rate.
        update_hz = min(max(rate_hz * 4.0, 50.0), 1000.0)
        self._timer = self.create_timer(1.0 / update_hz, self._poll_streaming)

        self.get_logger().info(
            f'Publishing corrected accel at {stream_hz} Hz on {topic} '
            f'(frame_id={self._frame_id}, port={port_desc})'
        )

    def _on_stream(self, status: ThreespaceStreamingStatus) -> None:
        if status != ThreespaceStreamingStatus.Data:
            return
        if self._manager is None:
            return

        accel_g = _accel_components(
            self._manager.get_value(StreamableCommands.GetPrimaryCorrectedAccelVec)
        )
        if accel_g is None:
            return

        msg = ImuAcceleration()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.x = accel_g[0] * _G_TO_MS2
        msg.y = accel_g[1] * _G_TO_MS2
        msg.z = accel_g[2] * _G_TO_MS2
        self._publisher.publish(msg)

    def _poll_streaming(self) -> None:
        if self._manager is None:
            return
        try:
            self._manager.update()
        except ThreespaceError as exc:
            self.get_logger().warn(f'Streaming update failed: {exc}')

    def _cleanup_sensor(self) -> None:
        if self._manager is not None:
            try:
                self._manager.disable()
            except ThreespaceError as exc:
                self.get_logger().warn(f'Failed to disable streaming manager: {exc}')
            self._manager = None
        if self._sensor is not None:
            try:
                self._sensor.cleanup()
            except ThreespaceError as exc:
                self.get_logger().warn(f'Failed to cleanup sensor: {exc}')
            self._sensor = None

    def destroy_node(self) -> bool:
        self._cleanup_sensor()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ImuAccelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
