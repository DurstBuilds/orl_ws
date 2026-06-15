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

    def _stdin_loop(self) -> None:
        prompt = f'{self._namespace} joint_despos (deg): '
        while self._stdin_running and rclpy.ok():
            try:
                line = input(prompt)
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
        super().destroy_node()


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('joint_despos_command requires an interactive terminal (TTY).', file=sys.stderr)
        raise SystemExit(1)

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
