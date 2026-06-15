#!/usr/bin/env python3
"""Boom teleop: incremental joint_despos from joystick, per namespace.

Namespace substring rules (case-insensitive):
  'knee'  — right stick X
  'wheel' — left stick X (both wheel motors share the same stick)
  'hip'   — buttons 4/5 (neg/pos); joint_despos clamped to hip_angle_limit_deg

All listed namespaces receive soft_mode (button 1 edge). hold_joint is published once
at startup (true), on rising edge when stick control begins (false), and on falling
edge when control returns to neutral (true, with despos snapped to curpos).

TWEAK at runtime via ros2 param (see BoomJoystickControl.__init__):
  knee_velocity_constant, wheel_velocity_constant, hip_velocity_constant
  right_stick_x_axis, left_stick_x_axis, stick_deadzone
  soft_mode_button_index, hip_neg_button_index, hip_pos_button_index
  namespaces, namespace_gear_ratios, publish_hz, hip_angle_limit_deg
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32


def clamp_hip_despos(despos: float, limit_rad: float) -> float:
    """Clamp desired hip position to ±limit_rad."""
    return max(-limit_rad, min(limit_rad, despos))


class JoyState:
    """Thread-safe cache of latest joystick axes and button edges."""
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
        right_x = self._read_axis(msg, right_x_axis) or 0.0
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


def parse_namespace_gear_ratios(
    param: str,
    default_gear_ratio: float,
) -> dict[str, float]:
    """Parse 'ns1:1.6,ns2:30' into a namespace -> gear_ratio map."""
    ratios: dict[str, float] = {}
    if not param.strip():
        return ratios
    for entry in param.split(','):
        item = entry.strip()
        if not item:
            continue
        if ':' not in item:
            raise ValueError(
                f"namespace_gear_ratios entry '{item}' must be namespace:gear_ratio"
            )
        ns, ratio_str = item.split(':', 1)
        ns = ns.strip()
        ratio = float(ratio_str.strip())
        if not ns:
            raise ValueError('namespace_gear_ratios namespace must be non-empty')
        if ratio <= 0.0:
            raise ValueError(f'gear_ratio for {ns} must be > 0')
        ratios[ns] = ratio
    return ratios


class NamespaceTarget:
    """Per-namespace publishers + curpos state."""

    def __init__(
        self,
        node: Node,
        namespace: str,
        *,
        knee_velocity: float,
        wheel_velocity: float,
        hip_velocity: float,
    ) -> None:
        self.namespace = namespace
        self.namespace_lower = namespace.lower()
        self.knee_velocity = knee_velocity
        self.wheel_velocity = wheel_velocity
        self.hip_velocity = hip_velocity
        self.lock = threading.Lock()
        self.curpos = 0.0
        self.has_curpos = True
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

    def publish_hold_joint(self, hold: bool) -> None:
        msg = Bool()
        msg.data = hold
        self.hold_joint_publisher.publish(msg)


class BoomJoystickControl(Node):
    """ROS node: /joy in, per-namespace joint_despos / hold_joint / soft_mode out."""

    def __init__(self) -> None:
        super().__init__('boom_joystick_control_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_hz', 50.0)  # TWEAK: teleop timer rate
        self.declare_parameter('gear_ratio', 1.0)  # TWEAK: fallback if ns not in namespace_gear_ratios
        self.declare_parameter('right_stick_x_axis', 3)  # TWEAK: knee axis index
        self.declare_parameter('left_stick_x_axis', 0)  # TWEAK: wheel axis index
        self.declare_parameter('soft_mode_button_index', 1)  # TWEAK: toggle on rising edge
        self.declare_parameter('hip_neg_button_index', 5)  # TWEAK
        self.declare_parameter('hip_pos_button_index', 4)  # TWEAK
        self.declare_parameter('knee_velocity_constant', 1.0)  # TWEAK: rad per tick before /gear_ratio
        self.declare_parameter('wheel_velocity_constant', 1.0)  # TWEAK
        self.declare_parameter('hip_velocity_constant', 1.0)  # TWEAK
        self.declare_parameter('hip_angle_limit_deg', 45.0)  # TWEAK: teleop clamp (translator uses launch)
        self.declare_parameter('stick_deadzone', 0.15)  # TWEAK
        self.declare_parameter('namespaces', '')
        self.declare_parameter('namespace_gear_ratios', '')

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be > 0')
        default_gear_ratio = (
            self.get_parameter('gear_ratio').get_parameter_value().double_value
        )
        if default_gear_ratio <= 0.0:
            raise ValueError('gear_ratio must be > 0')
        namespace_gear_ratios_param = (
            self.get_parameter('namespace_gear_ratios').get_parameter_value().string_value
        )
        self._namespace_gear_ratios = parse_namespace_gear_ratios(
            namespace_gear_ratios_param, default_gear_ratio
        )
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
        knee_vel = (
            self.get_parameter('knee_velocity_constant').get_parameter_value().double_value
        )
        wheel_vel = (
            self.get_parameter('wheel_velocity_constant').get_parameter_value().double_value
        )
        hip_vel = (
            self.get_parameter('hip_velocity_constant').get_parameter_value().double_value
        )
        self._knee_velocity_base = knee_vel
        self._wheel_velocity_base = wheel_vel
        self._hip_velocity_base = hip_vel
        hip_angle_limit_deg = (
            self.get_parameter('hip_angle_limit_deg').get_parameter_value().double_value
        )
        if hip_angle_limit_deg < 0.0:
            raise ValueError('hip_angle_limit_deg must be >= 0')
        self._hip_angle_limit_rad = math.radians(hip_angle_limit_deg)
        self._stick_deadzone = (
            self.get_parameter('stick_deadzone').get_parameter_value().double_value
        )

        ns_param = self.get_parameter('namespaces').get_parameter_value().string_value
        ns_list = [s.strip() for s in ns_param.split(',') if s.strip()] if ns_param.strip() else ['']

        self._state = JoyState()
        self._targets = []
        for ns in ns_list:
            gear_ratio = self._namespace_gear_ratios.get(ns, default_gear_ratio)
            velocity_scale = 1.0 / gear_ratio
            self._targets.append(NamespaceTarget(
                self,
                ns,
                knee_velocity=knee_vel * velocity_scale,
                wheel_velocity=wheel_vel * velocity_scale,
                hip_velocity=hip_vel * velocity_scale,
            ))
        self._last_soft_mode = False

        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / publish_hz, self._publish_timer_callback)

        for target in self._targets:
            target.publish_hold_joint(True)

        despos_topics = ', '.join(t.despos_publisher.topic_name for t in self._targets)
        soft_mode_topics = ', '.join(t.soft_mode_publisher.topic_name for t in self._targets)
        hold_joint_topics = ', '.join(t.hold_joint_publisher.topic_name for t in self._targets)
        self.get_logger().info(
            f'Subscribed to {joy_topic}; publishing to [{despos_topics}] at {publish_hz:.0f} Hz.\n'
            f'  default gear_ratio={default_gear_ratio:.4f}; '
            f'per-ns gear ratios={self._namespace_gear_ratios or "(default)"}\n'
            f'  soft_mode topics: [{soft_mode_topics}]\n'
            f'  hold_joint topics: [{hold_joint_topics}]\n'
            f'  Right X axis [{self._right_x_axis}] -> knee (base {self._knee_velocity_base:.4f} / gear_ratio)\n'
            f'  Left X axis [{self._left_x_axis}] -> wheel (base {self._wheel_velocity_base:.4f} / gear_ratio)\n'
            f'  Hip buttons -> delta/tick (base {self._hip_velocity_base:.4f} / gear_ratio)\n'
            f'  Hip joint_despos clamped to +/-{hip_angle_limit_deg:.1f} deg\n'
            f'  Button[{self._soft_mode_button_index}] toggles soft_mode')

    def _publish_despos(self, target: NamespaceTarget, despos: float) -> None:
        if 'hip' in target.namespace_lower:
            despos = clamp_hip_despos(despos, self._hip_angle_limit_rad)
        target.publish_despos(despos)

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
        """Map cached joy state to despos deltas; hold_joint on control edges only."""
        right_x, left_x, soft_mode, hip_neg, hip_pos = self._state.get_state()

        soft_msg = Bool()
        soft_msg.data = soft_mode
        for target in self._targets:
            target.soft_mode_publisher.publish(soft_msg)
        soft_mode_toggled_off = self._last_soft_mode and not soft_mode
        if soft_mode != self._last_soft_mode:
            self.get_logger().info(f'soft_mode={soft_mode}')
            self._last_soft_mode = soft_mode

        for target in self._targets:
            _, curpos = target.get_curpos()

            if soft_mode_toggled_off:
                self._publish_despos(target, curpos)

            control_active = False
            delta = 0.0

            if 'hip' in target.namespace_lower:
                if hip_neg and not hip_pos:
                    control_active = True
                    delta = -target.hip_velocity
                elif hip_pos and not hip_neg:
                    control_active = True
                    delta = target.hip_velocity
            elif 'knee' in target.namespace_lower:
                axis = right_x
                if abs(axis) > self._stick_deadzone:
                    control_active = True
                    delta = axis * target.knee_velocity
            elif 'wheel' in target.namespace_lower:
                axis = left_x
                if abs(axis) > self._stick_deadzone:
                    control_active = True
                    delta = axis * target.wheel_velocity

            if control_active:
                if not target.control_was_active:
                    target.publish_hold_joint(False)
                self._publish_despos(target, curpos + delta)
                target.control_was_active = True
                continue

            if target.control_was_active:
                target.publish_hold_joint(True)
                self._publish_despos(target, curpos)
            target.control_was_active = False


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
