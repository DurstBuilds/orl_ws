#!/usr/bin/env python3
# Publishes joint_despos from gamepad when deadman bumper is held.
# Right stick: absolute position. Left stick X (axis 6): velocity offset from curpos.
# Supports multiple namespaces: publishes to each /{ns}/joint_despos simultaneously.

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


def stick_to_joint_angle(x: float, y: float) -> float:
    """Map right-stick deflection to joint angle (rad): down=0, right=pi/2, up=pi, left=-pi/2."""
    return math.atan2(x, -y)


class JoyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._deadman_active = False
        self._position_angle: Optional[float] = None
        self._velocity_axis_value: Optional[float] = None
        self._both_centered = False

    def update(
        self,
        msg: Joy,
        deadman_index: int,
        right_x_axis: int,
        right_y_axis: int,
        left_x_axis: int,
        left_y_axis: int,
        velocity_axis: int,
        stick_deadzone: float,
    ) -> None:
        deadman = (
            deadman_index < len(msg.buttons) and msg.buttons[deadman_index] == 1
        )
        position_angle: Optional[float] = None
        velocity_axis_value: Optional[float] = None
        both_centered = False

        if deadman:
            right_x = self._read_axis(msg, right_x_axis)
            right_y = self._read_axis(msg, right_y_axis)
            left_x = self._read_axis(msg, left_x_axis)
            left_y = self._read_axis(msg, left_y_axis)
            vel_axis = self._read_axis(msg, velocity_axis)

            if right_x is not None and right_y is not None:
                right_centered = math.hypot(right_x, right_y) <= stick_deadzone
                left_centered = True
                if left_x is not None and left_y is not None:
                    left_centered = math.hypot(left_x, left_y) <= stick_deadzone

                both_centered = right_centered and left_centered

                if not right_centered:
                    position_angle = stick_to_joint_angle(right_x, right_y)
                elif vel_axis is not None and abs(vel_axis) > stick_deadzone:
                    velocity_axis_value = vel_axis
                elif both_centered:
                    position_angle = 0.0

        with self._lock:
            self._deadman_active = deadman
            self._position_angle = position_angle
            self._velocity_axis_value = velocity_axis_value
            self._both_centered = both_centered

    @staticmethod
    def _read_axis(msg: Joy, axis_index: int) -> Optional[float]:
        if axis_index < len(msg.axes):
            return float(msg.axes[axis_index])
        return None

    def get_control_state(
        self,
    ) -> tuple[bool, Optional[float], Optional[float], bool]:
        with self._lock:
            return (
                self._deadman_active,
                self._position_angle,
                self._velocity_axis_value,
                self._both_centered,
            )


class NamespaceTarget:
    """Per-namespace publisher + curpos state."""

    def __init__(self, node: Node, namespace: str) -> None:
        self.namespace = namespace
        self.lock = threading.Lock()
        self.curpos = 0.0
        self.has_curpos = False

        if namespace:
            despos_topic = f'/{namespace}/joint_despos'
            curpos_topic = f'/{namespace}/joint_curpos'
        else:
            despos_topic = 'joint_despos'
            curpos_topic = 'joint_curpos'

        self.publisher = node.create_publisher(Float32, despos_topic, 10)
        node.create_subscription(
            Float32, curpos_topic, self._curpos_callback, 10)

    def _curpos_callback(self, msg: Float32) -> None:
        with self.lock:
            self.curpos = msg.data
            self.has_curpos = True

    def get_curpos(self) -> tuple[bool, float]:
        with self.lock:
            return self.has_curpos, self.curpos

    def publish(self, despos: float) -> None:
        msg = Float32()
        msg.data = float(despos)
        self.publisher.publish(msg)


class JoystickControl(Node):
    def __init__(self) -> None:
        super().__init__('joystick_control_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('deadman_button_index', 5)
        self.declare_parameter('right_stick_x_axis', 3)
        self.declare_parameter('right_stick_y_axis', 4)
        self.declare_parameter('left_stick_x_axis', 0)
        self.declare_parameter('left_stick_y_axis', 1)
        self.declare_parameter('velocity_axis', 6)
        self.declare_parameter('velocity_constant', 4.0)
        self.declare_parameter('stick_deadzone', 0.15)
        self.declare_parameter('namespaces', '')

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        self._deadman_index = (
            self.get_parameter('deadman_button_index').get_parameter_value().integer_value
        )
        self._right_x_axis = (
            self.get_parameter('right_stick_x_axis').get_parameter_value().integer_value
        )
        self._right_y_axis = (
            self.get_parameter('right_stick_y_axis').get_parameter_value().integer_value
        )
        self._left_x_axis = (
            self.get_parameter('left_stick_x_axis').get_parameter_value().integer_value
        )
        self._left_y_axis = (
            self.get_parameter('left_stick_y_axis').get_parameter_value().integer_value
        )
        self._velocity_axis = (
            self.get_parameter('velocity_axis').get_parameter_value().integer_value
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

        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / publish_hz, self._publish_timer_callback)

        target_topics = ', '.join(t.publisher.topic_name for t in self._targets)
        self.get_logger().info(
            f'Subscribed to {joy_topic}; publishing to [{target_topics}] at '
            f'{publish_hz:.0f} Hz when button[{self._deadman_index}] held.\n'
            f'  Right stick [{self._right_x_axis}, {self._right_y_axis}]: position\n'
            f'  Velocity axis [{self._velocity_axis}]: '
            f'despos = curpos + axis * {self._velocity_constant:.3f}\n'
            f'  Both sticks centered (right + left [{self._left_x_axis}, '
            f'{self._left_y_axis}]): despos=0')

    def _joy_callback(self, msg: Joy) -> None:
        self._state.update(
            msg,
            self._deadman_index,
            self._right_x_axis,
            self._right_y_axis,
            self._left_x_axis,
            self._left_y_axis,
            self._velocity_axis,
            self._stick_deadzone,
        )

    def _publish_timer_callback(self) -> None:
        deadman, position_angle, velocity_axis, both_centered = (
            self._state.get_control_state()
        )
        if not deadman:
            return

        for target in self._targets:
            despos: Optional[float] = None

            if position_angle is not None:
                despos = position_angle
            elif velocity_axis is not None:
                has_curpos, curpos = target.get_curpos()
                if has_curpos:
                    despos = curpos + velocity_axis * self._velocity_constant
            elif both_centered:
                despos = 0.0

            if despos is not None:
                target.publish(despos)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickControl()
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
