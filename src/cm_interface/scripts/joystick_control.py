#!/usr/bin/env python3
# Publishes joint_despos from gamepad right stick when deadman bumper is held.

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


def stick_to_joint_angle(x: float, y: float) -> float:
    """Map right-stick deflection to joint angle (rad): down=0, right=pi/2, up=pi, left=-pi/2."""
    return math.atan2(y, x)

class JoyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._deadman_active = False
        self._stick_angle: Optional[float] = None

    def update(self, msg: Joy, deadman_index: int, x_axis: int, y_axis: int,
               stick_deadzone: float) -> None:
        deadman = (
            deadman_index < len(msg.buttons) and msg.buttons[deadman_index] == 1
        )
        stick_angle: Optional[float] = None

        if deadman and x_axis < len(msg.axes) and y_axis < len(msg.axes):
            x = -float(msg.axes[x_axis])
            y = float(msg.axes[y_axis])
            if math.hypot(x, y) > stick_deadzone:
                stick_angle = stick_to_joint_angle(x, y)

        with self._lock:
            self._deadman_active = deadman
            if stick_angle is not None:
                self._stick_angle = stick_angle

    def get_publish_state(self) -> tuple[bool, Optional[float]]:
        with self._lock:
            if not self._deadman_active:
                return False, None
            return True, self._stick_angle


class JoystickControl(Node):
    def __init__(self) -> None:
        super().__init__('joystick_control_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('deadman_button_index', 5)
        self.declare_parameter('right_stick_x_axis', 4)
        self.declare_parameter('right_stick_y_axis', 5)
        self.declare_parameter('stick_deadzone', 0.15)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        self._deadman_index = (
            self.get_parameter('deadman_button_index').get_parameter_value().integer_value
        )
        self._x_axis = (
            self.get_parameter('right_stick_x_axis').get_parameter_value().integer_value
        )
        self._y_axis = (
            self.get_parameter('right_stick_y_axis').get_parameter_value().integer_value
        )
        self._stick_deadzone = (
            self.get_parameter('stick_deadzone').get_parameter_value().double_value
        )

        self._state = JoyState()
        self._publisher = self.create_publisher(Float32, 'joint_despos', 10)
        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / publish_hz, self._publish_timer_callback)

        self.get_logger().info(
            f'Subscribed to {joy_topic}; publishing joint_despos at {publish_hz:.0f} Hz '
            f'when button[{self._deadman_index}] held. '
            f'Stick axes [{self._x_axis}, {self._y_axis}] (x inverted), '
            f'map: down=0, right=pi/2, up=pi')

    def _joy_callback(self, msg: Joy) -> None:
        self._state.update(
            msg,
            self._deadman_index,
            self._x_axis,
            self._y_axis,
            self._stick_deadzone,
        )

    def _publish_timer_callback(self) -> None:
        deadman_active, stick_angle = self._state.get_publish_state()
        if not deadman_active or stick_angle is None:
            return

        msg = Float32()
        msg.data = float(stick_angle)
        self._publisher.publish(msg)


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
