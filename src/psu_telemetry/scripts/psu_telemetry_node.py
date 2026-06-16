#!/usr/bin/env python3
# Reads output current from a TTI QPX600DP over USB serial and publishes at a fixed rate.

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        'pyserial is required. Install with: sudo apt install python3-serial'
    ) from exc

_RESPONSE_TERMINATOR = b'\r\n'
_CURRENT_RESPONSE_RE = re.compile(r'^([+-]?(?:\d+\.?\d*|\.\d+))A\s*$', re.IGNORECASE)


def _parse_current_response(raw: bytes) -> float:
    text = raw.decode('ascii', errors='replace').strip()
    match = _CURRENT_RESPONSE_RE.match(text)
    if match is None:
        raise ValueError(f'unexpected current response: {text!r}')
    return float(match.group(1))


class PsuTelemetryNode(Node):
    """ROS node: QPX600DP I<n>O? query in, Float32 current (A) out."""

    def __init__(self) -> None:
        super().__init__('psu_telemetry_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('output_index', 1)
        self.declare_parameter('topic', 'power_supply/current')
        self.declare_parameter('serial_timeout_s', 0.5)
        self.declare_parameter('identify_on_startup', True)

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        output_index = int(self.get_parameter('output_index').value)
        topic = str(self.get_parameter('topic').value)
        serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        identify_on_startup = bool(self.get_parameter('identify_on_startup').value)

        if output_index not in (1, 2):
            self.get_logger().error('Parameter "output_index" must be 1 or 2.')
            raise SystemExit(1)
        if publish_rate_hz <= 0.0:
            self.get_logger().error('Parameter "publish_rate_hz" must be > 0.')
            raise SystemExit(1)

        self._current_query = f'I{output_index}O?\n'

        try:
            self._serial = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=serial_timeout_s,
            )
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open serial port {serial_port}: {exc}')
            raise SystemExit(1) from exc

        self._publisher = self.create_publisher(Float32, topic, 10)
        timer_period_s = 1.0 / publish_rate_hz
        self._timer = self.create_timer(timer_period_s, self._poll_current)

        self.get_logger().info(
            f'Publishing current from output {output_index} on {topic} at {publish_rate_hz:.1f} Hz '
            f'via {serial_port}'
        )

        if identify_on_startup:
            self._identify_device()

    def _write_query(self, command: str) -> bytes:
        self._serial.reset_input_buffer()
        self._serial.write(command.encode('ascii'))
        self._serial.flush()
        return self._serial.read_until(_RESPONSE_TERMINATOR)

    def _identify_device(self) -> None:
        try:
            response = self._write_query('*IDN?\n')
            if not response:
                self.get_logger().warn('*IDN? returned no response')
                return
            text = response.decode('ascii', errors='replace').strip()
            self.get_logger().info(f'Power supply identity: {text}')
        except (serial.SerialException, UnicodeDecodeError) as exc:
            self.get_logger().warn(f'*IDN? failed: {exc}')

    def _poll_current(self) -> None:
        try:
            response = self._write_query(self._current_query)
            if not response:
                self.get_logger().warn(f'{self._current_query.strip()} returned no response')
                return
            current_a = _parse_current_response(response)
        except (serial.SerialException, ValueError, UnicodeDecodeError) as exc:
            self.get_logger().warn(f'Current read failed: {exc}')
            return

        msg = Float32()
        msg.data = float(current_a)
        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PsuTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
