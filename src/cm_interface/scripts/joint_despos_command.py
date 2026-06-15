#!/usr/bin/env python3
# Publishes joint_despos from terminal input (degrees). One motor namespace per process.

import math
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def _topic_for_namespace(namespace: str, suffix: str) -> str:
    if namespace:
        return f'/{namespace}/{suffix}'
    return suffix


def _open_terminal_stream(mode: str):
    """Use controlling terminal when launch/ros2 run redirects stdin/stdout."""
    stream = sys.stdin if mode == 'r' and sys.stdin.isatty() else sys.stdout if mode == 'w' and sys.stdout.isatty() else None
    if stream is not None:
        return stream, False
    try:
        return open('/dev/tty', mode), True
    except OSError:
        return None, False


class JointDesposCommand(Node):
    """ROS node: stdin in, namespaced joint_despos / hold_joint out."""

    def __init__(self) -> None:
        super().__init__('joint_despos_command')

        self.declare_parameter('namespace', '')
        namespace = str(self.get_parameter('namespace').value).strip().lstrip('/')
        if not namespace:
            self.get_logger().error('Parameter "namespace" is required (e.g. hip_motor).')
            raise SystemExit(1)

        self._namespace = namespace
        despos_topic = _topic_for_namespace(namespace, 'joint_despos')
        hold_joint_topic = _topic_for_namespace(namespace, 'hold_joint')

        self._despos_publisher = self.create_publisher(Float32, despos_topic, 10)
        self._hold_joint_publisher = self.create_publisher(Bool, hold_joint_topic, 10)

        self._input_stream, self._close_input = _open_terminal_stream('r')
        self._output_stream, self._close_output = _open_terminal_stream('w')
        if self._input_stream is None or self._output_stream is None:
            self.get_logger().error(
                'No interactive terminal available. Run from a TTY or use: '
                'ros2 run cm_interface joint_despos_command --ros-args -p namespace:=<motor>'
            )
            raise SystemExit(1)

        self._stdin_running = True
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        self.get_logger().info(
            f'Publishing joint_despos to {despos_topic} (enter degrees; q to quit).'
        )

    def _publish_despos_deg(self, degrees: float) -> None:
        radians = math.radians(degrees)

        hold_msg = Bool()
        hold_msg.data = False
        self._hold_joint_publisher.publish(hold_msg)

        despos_msg = Float32()
        despos_msg.data = float(radians)
        self._despos_publisher.publish(despos_msg)

        self.get_logger().info(
            f'Published {degrees:.4f} deg ({radians:.4f} rad) to '
            f'{self._despos_publisher.topic_name}'
        )

    def _read_line(self, prompt: str) -> str:
        self._output_stream.write(prompt)
        self._output_stream.flush()
        return self._input_stream.readline()

    def _stdin_loop(self) -> None:
        prompt = f'{self._namespace} joint_despos (deg): '
        while self._stdin_running and rclpy.ok():
            try:
                line = self._read_line(prompt)
            except EOFError:
                break

            stripped = line.strip()
            if not stripped:
                continue
            if stripped.lower() in ('q', 'quit'):
                self.get_logger().info('Quit requested.')
                break

            try:
                degrees = float(stripped)
            except ValueError:
                self.get_logger().warn(f'Invalid number: {stripped!r}')
                continue

            self._publish_despos_deg(degrees)

        self._stdin_running = False
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self) -> None:
        self._stdin_running = False
        self._stdin_thread.join(timeout=0.5)
        if self._close_input:
            self._input_stream.close()
        if self._close_output:
            self._output_stream.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = JointDesposCommand()
    except SystemExit:
        rclpy.shutdown()
        raise SystemExit(1)

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
