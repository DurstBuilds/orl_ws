#!/usr/bin/env python3
# Publishes joint_despos increments from joystick X axes by namespace role:
# - right stick X drives namespaces containing "knee"
# - left stick X drives namespaces containing "wheel"
# Also publishes soft_mode toggle (button index configurable, default 1).

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32


class JoyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._right_x = 0.0
        self._left_x = 0.0
        self._soft_mode = False
        self._prev_soft_mode_button_pressed = False

    def update(self, msg: Joy, right_x_axis: int, left_x_axis: int, soft_mode_button_index: int) -> None:
        right_x = self._read_axis(msg, right_x_axis) or 0.0
        left_x = self._read_axis(msg, left_x_axis) or 0.0
        soft_mode_button_pressed = (
            soft_mode_button_index < len(msg.buttons) and
            msg.buttons[soft_mode_button_index] == 1
        )

        with self._lock:
            self._right_x = right_x
            self._left_x = left_x
            if soft_mode_button_pressed and not self._prev_soft_mode_button_pressed:
                self._soft_mode = not self._soft_mode
            self._prev_soft_mode_button_pressed = soft_mode_button_pressed

    @staticmethod
    def _read_axis(msg: Joy, axis_index: int) -> Optional[float]:
        if axis_index < len(msg.axes):
            return float(msg.axes[axis_index])
        return None

    def get_state(self) -> tuple[float, float, bool]:
        with self._lock:
            return self._right_x, self._left_x, self._soft_mode


class NamespaceTarget:
    """Per-namespace publishers + curpos state."""

    def __init__(self, node: Node, namespace: str) -> None:
        self.namespace = namespace
        self.namespace_lower = namespace.lower()
        self.lock = threading.Lock()
        self.curpos = 0.0
        self.has_curpos = False
        self.warned_no_curpos = False

        if namespace:
            despos_topic = f'/{namespace}/joint_despos'
            curpos_topic = f'/{namespace}/joint_curpos'
            soft_mode_topic = f'/{namespace}/soft_mode'
        else:
            despos_topic = 'joint_despos'
            curpos_topic = 'joint_curpos'
            soft_mode_topic = 'soft_mode'

        self.despos_publisher = node.create_publisher(Float32, despos_topic, 10)
        self.soft_mode_publisher = node.create_publisher(Bool, soft_mode_topic, 10)
        node.create_subscription(Float32, curpos_topic, self._curpos_callback, 10)

    def _curpos_callback(self, msg: Float32) -> None:
        with self.lock:
            self.curpos = msg.data
            self.has_curpos = True

    def get_curpos(self) -> tuple[bool, float]:
        with self.lock:
            return self.has_curpos, self.curpos

    def publish_despos(self, despos: float) -> None:
        msg = Float32()
        msg.data = float(despos)
        self.despos_publisher.publish(msg)


class BoomJoystickControl(Node):
    def __init__(self) -> None:
        super().__init__('boom_joystick_control_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('right_stick_x_axis', 3)
        self.declare_parameter('left_stick_x_axis', 0)
        self.declare_parameter('soft_mode_button_index', 1)
        self.declare_parameter('velocity_constant', 4.0)
        self.declare_parameter('stick_deadzone', 0.15)
        self.declare_parameter('namespaces', '')

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        self._right_x_axis = (
            self.get_parameter('right_stick_x_axis').get_parameter_value().integer_value
        )
        self._left_x_axis = (
            self.get_parameter('left_stick_x_axis').get_parameter_value().integer_value
        )
        self._soft_mode_button_index = (
            self.get_parameter('soft_mode_button_index').get_parameter_value().integer_value
        )
        self._velocity_constant = (
            self.get_parameter('velocity_constant').get_parameter_value().double_value
        )
        self._stick_deadzone = (
            self.get_parameter('stick_deadzone').get_parameter_value().double_value
        )

        ns_param = self.get_parameter('namespaces').get_parameter_value().string_value
        ns_list = [s.strip() for s in ns_param.split(',') if s.strip()] if ns_param.strip() else ['']

        self._state = JoyState()
        self._targets = [NamespaceTarget(self, ns) for ns in ns_list]
        self._last_soft_mode = False

        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / publish_hz, self._publish_timer_callback)

        despos_topics = ', '.join(t.despos_publisher.topic_name for t in self._targets)
        soft_mode_topics = ', '.join(t.soft_mode_publisher.topic_name for t in self._targets)
        self.get_logger().info(
            f'Subscribed to {joy_topic}; publishing to [{despos_topics}] at {publish_hz:.0f} Hz.\n'
            f'  soft_mode topics: [{soft_mode_topics}]\n'
            f'  Right X axis [{self._right_x_axis}] -> namespaces containing "knee"\n'
            f'  Left X axis [{self._left_x_axis}] -> namespaces containing "wheel"\n'
            f'  despos increment: curpos + axis * {self._velocity_constant:.3f}\n'
            f'  Button[{self._soft_mode_button_index}] toggles soft_mode')

    def _joy_callback(self, msg: Joy) -> None:
        self._state.update(
            msg,
            self._right_x_axis,
            self._left_x_axis,
            self._soft_mode_button_index,
        )

    def _publish_timer_callback(self) -> None:
        right_x, left_x, soft_mode = self._state.get_state()

        soft_msg = Bool()
        soft_msg.data = soft_mode
        for target in self._targets:
            target.soft_mode_publisher.publish(soft_msg)
        if soft_mode != self._last_soft_mode:
            self.get_logger().info(f'soft_mode={soft_mode}')
            self._last_soft_mode = soft_mode

        for target in self._targets:
            has_curpos, curpos = target.get_curpos()

            axis = 0.0
            if 'knee' in target.namespace_lower:
                axis = right_x
            elif 'wheel' in target.namespace_lower:
                axis = left_x
            else:
                continue

            if abs(axis) <= self._stick_deadzone:
                continue

            if not has_curpos and not target.warned_no_curpos:
                topic_name = target.despos_publisher.topic_name
                self.get_logger().warn(
                    f'No joint_curpos received yet for {topic_name}; using 0.0 fallback'
                )
                target.warned_no_curpos = True

            despos = curpos + axis * self._velocity_constant
            target.publish_despos(despos)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoomJoystickControl()
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
