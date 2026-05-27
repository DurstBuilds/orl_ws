#!/usr/bin/env python3
# Publishes joint_despos increments from joystick by namespace role:
# - right stick X drives namespaces containing "knee"
# - left stick X drives namespaces containing "wheel"
# - button 4 (held) negative / button 5 (held) positive for namespaces containing "hip"
# Also publishes soft_mode toggle and hold_joint (false while controlling, true on release).

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
        self._hip_neg_pressed = False
        self._hip_pos_pressed = False

    def update(
        self,
        msg: Joy,
        right_x_axis: int,
        left_x_axis: int,
        soft_mode_button_index: int,
        hip_neg_button_index: int,
        hip_pos_button_index: int,
    ) -> None:
        right_x = -self._read_axis(msg, right_x_axis) or 0.0
        left_x = -self._read_axis(msg, left_x_axis) or 0.0
        soft_mode_button_pressed = (
            soft_mode_button_index < len(msg.buttons) and
            msg.buttons[soft_mode_button_index] == 1
        )
        hip_neg_pressed = (
            hip_neg_button_index < len(msg.buttons) and
            msg.buttons[hip_neg_button_index] == 1
        )
        hip_pos_pressed = (
            hip_pos_button_index < len(msg.buttons) and
            msg.buttons[hip_pos_button_index] == 1
        )

        with self._lock:
            self._right_x = right_x
            self._left_x = left_x
            self._hip_neg_pressed = hip_neg_pressed
            self._hip_pos_pressed = hip_pos_pressed
            if soft_mode_button_pressed and not self._prev_soft_mode_button_pressed:
                self._soft_mode = not self._soft_mode
            self._prev_soft_mode_button_pressed = soft_mode_button_pressed

    @staticmethod
    def _read_axis(msg: Joy, axis_index: int) -> Optional[float]:
        if axis_index < len(msg.axes):
            return float(msg.axes[axis_index])
        return None

    def get_state(self) -> tuple[float, float, bool, bool, bool]:
        with self._lock:
            return (
                self._right_x,
                self._left_x,
                self._soft_mode,
                self._hip_neg_pressed,
                self._hip_pos_pressed,
            )


class NamespaceTarget:
    """Per-namespace publishers + curpos state."""

    def __init__(self, node: Node, namespace: str) -> None:
        self.namespace = namespace
        self.namespace_lower = namespace.lower()
        self.lock = threading.Lock()
        self.curpos = 0.0
        self.has_curpos = False
        self.warned_no_curpos = False
        self.control_was_active = False

        if namespace:
            despos_topic = f'/{namespace}/joint_despos'
            curpos_topic = f'/{namespace}/joint_curpos'
            soft_mode_topic = f'/{namespace}/soft_mode'
            hold_joint_topic = f'/{namespace}/hold_joint'
        else:
            despos_topic = 'joint_despos'
            curpos_topic = 'joint_curpos'
            soft_mode_topic = 'soft_mode'
            hold_joint_topic = 'hold_joint'

        self.despos_publisher = node.create_publisher(Float32, despos_topic, 10)
        self.soft_mode_publisher = node.create_publisher(Bool, soft_mode_topic, 10)
        self.hold_joint_publisher = node.create_publisher(Bool, hold_joint_topic, 10)
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
        self.declare_parameter('hip_neg_button_index', 5)
        self.declare_parameter('hip_pos_button_index', 4)
        self.declare_parameter('knee_velocity_constant', 2.0)
        self.declare_parameter('wheel_velocity_constant', 2.0)
        self.declare_parameter('hip_velocity_constant', 1.0)
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
        self._hip_neg_button_index = (
            self.get_parameter('hip_neg_button_index').get_parameter_value().integer_value
        )
        self._hip_pos_button_index = (
            self.get_parameter('hip_pos_button_index').get_parameter_value().integer_value
        )
        self._knee_velocity_constant = (
            self.get_parameter('knee_velocity_constant').get_parameter_value().double_value
        )
        self._wheel_velocity_constant = (
            self.get_parameter('wheel_velocity_constant').get_parameter_value().double_value
        )
        self._hip_velocity_constant = (
            self.get_parameter('hip_velocity_constant').get_parameter_value().double_value
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
        hold_joint_topics = ', '.join(t.hold_joint_publisher.topic_name for t in self._targets)
        self.get_logger().info(
            f'Subscribed to {joy_topic}; publishing to [{despos_topics}] at {publish_hz:.0f} Hz.\n'
            f'  soft_mode topics: [{soft_mode_topics}]\n'
            f'  hold_joint topics: [{hold_joint_topics}]\n'
            f'  Right X axis [{self._right_x_axis}] -> knee: curpos + axis * {self._knee_velocity_constant:.3f}\n'
            f'  Left X axis [{self._left_x_axis}] -> wheel: curpos + axis * {self._wheel_velocity_constant:.3f}\n'
            f'  Button[{self._hip_neg_button_index}] held -> hip: curpos - {self._hip_velocity_constant:.3f}\n'
            f'  Button[{self._hip_pos_button_index}] held -> hip: curpos + {self._hip_velocity_constant:.3f}\n'
            f'  Button[{self._soft_mode_button_index}] toggles soft_mode')

    def _joy_callback(self, msg: Joy) -> None:
        self._state.update(
            msg,
            self._right_x_axis,
            self._left_x_axis,
            self._soft_mode_button_index,
            self._hip_neg_button_index,
            self._hip_pos_button_index,
        )

    def _publish_timer_callback(self) -> None:
        right_x, left_x, soft_mode, hip_neg, hip_pos = self._state.get_state()

        soft_msg = Bool()
        soft_msg.data = soft_mode
        for target in self._targets:
            target.soft_mode_publisher.publish(soft_msg)
        if soft_mode != self._last_soft_mode:
            self.get_logger().info(f'soft_mode={soft_mode}')
            self._last_soft_mode = soft_mode

        hold_joint_msg = Bool()

        for target in self._targets:
            has_curpos, curpos = target.get_curpos()

            control_active = False
            delta = 0.0

            if 'hip' in target.namespace_lower:
                if hip_neg and not hip_pos:
                    control_active = True
                    delta = -self._hip_velocity_constant
                elif hip_pos and not hip_neg:
                    control_active = True
                    delta = self._hip_velocity_constant
            elif 'knee' in target.namespace_lower:
                axis = right_x
                if abs(axis) > self._stick_deadzone:
                    control_active = True
                    delta = axis * self._knee_velocity_constant
            elif 'wheel' in target.namespace_lower:
                axis = left_x
                if abs(axis) > self._stick_deadzone:
                    control_active = True
                    delta = axis * self._wheel_velocity_constant

            hold_joint_msg.data = not control_active
            target.hold_joint_publisher.publish(hold_joint_msg)

            if not control_active:
                if target.control_was_active and has_curpos:
                    target.publish_despos(curpos)
                target.control_was_active = False
                continue

            if not has_curpos and not target.warned_no_curpos:
                topic_name = target.despos_publisher.topic_name
                self.get_logger().warn(
                    f'No joint_curpos received yet for {topic_name}; using 0.0 fallback'
                )
                target.warned_no_curpos = True
            target.publish_despos(curpos + delta)
            target.control_was_active = True


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
