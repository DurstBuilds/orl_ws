#!/usr/bin/env python3
# Publishes motor_command at 100 Hz from keyboard input over SSH (stdin/termios).
# Numpad keys command absolute positions using feedback from motor_state.

import math
import select
import sys
import termios
import threading
import time
import tty
from typing import Callable, Optional

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import MotorCommand, MotorState

KP = 2.0
KD = 0.02
DEFAULT_ARROW_DELTA = 0.1
ARROW_DELTA_MIN = 0.001
ARROW_DELTA_MAX = 0.5
ARROW_DELTA_STEP = 0.001
PUBLISH_HZ = 100.0
ARROW_HOLD_TIMEOUT_S = 0.2
MAX_DELTA = math.pi

NUMPAD_TARGETS = {
    '2': 0.0,
    '4': math.pi / 2.0,
    '6': -math.pi / 2.0,
    '8': math.pi,
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def shortest_delta(current: float, target: float) -> float:
    """Shortest angular delta from current to target, in [-pi, pi]."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def delta_to_pm_pi(current: float) -> float:
    """Shortest delta to pi; -pi is the same physical angle on the motor."""
    d_to_pi = shortest_delta(current, math.pi)
    d_to_neg_pi = shortest_delta(current, -math.pi)
    return d_to_pi if abs(d_to_pi) <= abs(d_to_neg_pi) else d_to_neg_pi


class KeyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._current_position = 0.0
        self._state_received = False
        self._pending_delta: Optional[float] = None
        self._arrow_delta = DEFAULT_ARROW_DELTA
        self._last_right_time = 0.0
        self._last_left_time = 0.0
        self._last_up_time = 0.0
        self._last_down_time = 0.0
        self._was_up_held = False
        self._was_down_held = False

    def update_motor_state(self, position: float) -> None:
        with self._lock:
            self._current_position = position
            self._state_received = True

    def set_numpad_target(
        self, key: str, target: float, log_fn: Callable[[str], None]
    ) -> bool:
        with self._lock:
            if not self._state_received:
                return False
            current = self._current_position
            if key == '8':
                delta = delta_to_pm_pi(current)
                des_label = 'pi'
            else:
                delta = shortest_delta(current, target)
                des_label = f'{target:.4f}'
            delta = clamp(delta, -MAX_DELTA, MAX_DELTA)
            self._pending_delta = delta
        log_fn(
            f'Numpad {key} -> des {des_label} rad, cur {current:.4f} rad, '
            f'dP {delta:.4f} rad (one shot)')
        return True

    def touch_right(self) -> None:
        with self._lock:
            self._last_right_time = time.monotonic()
            self._last_left_time = 0.0

    def touch_left(self) -> None:
        with self._lock:
            self._last_left_time = time.monotonic()
            self._last_right_time = 0.0

    def touch_up(self) -> None:
        with self._lock:
            self._last_up_time = time.monotonic()
            self._last_down_time = 0.0

    def touch_down(self) -> None:
        with self._lock:
            self._last_down_time = time.monotonic()
            self._last_up_time = 0.0

    def adjust_arrow_delta_if_held(self, log_fn: Callable[[str], None]) -> None:
        now = time.monotonic()
        with self._lock:
            up_held = (now - self._last_up_time) < ARROW_HOLD_TIMEOUT_S
            down_held = (now - self._last_down_time) < ARROW_HOLD_TIMEOUT_S

            if up_held:
                self._arrow_delta = clamp(
                    self._arrow_delta + ARROW_DELTA_STEP,
                    ARROW_DELTA_MIN, ARROW_DELTA_MAX)
            elif down_held:
                self._arrow_delta = clamp(
                    self._arrow_delta - ARROW_DELTA_STEP,
                    ARROW_DELTA_MIN, ARROW_DELTA_MAX)

            if self._was_up_held and not up_held:
                log_fn(f'arrow_delta = {self._arrow_delta:.4f}')
            if self._was_down_held and not down_held:
                log_fn(f'arrow_delta = {self._arrow_delta:.4f}')

            self._was_up_held = up_held
            self._was_down_held = down_held

    def get_position(self) -> float:
        with self._lock:
            if self._pending_delta is not None:
                delta = self._pending_delta
                self._pending_delta = None
                return delta

            now = time.monotonic()
            right = (now - self._last_right_time) < ARROW_HOLD_TIMEOUT_S
            left = (now - self._last_left_time) < ARROW_HOLD_TIMEOUT_S

            if right and not left:
                return self._arrow_delta
            if left and not right:
                return -self._arrow_delta
            return 0.0


class KeyboardCommand(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_command')

        if not sys.stdin.isatty():
            self.get_logger().fatal(
                'stdin is not a TTY. Run in an interactive SSH terminal.')
            raise RuntimeError('stdin is not a TTY')

        self._publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.create_subscription(MotorState, 'motor_state', self._motor_state_callback, 10)
        self._state = KeyState()
        self._stdin_running = True
        self._stdin_term_attrs = None
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_timer)

        self.get_logger().info(
            f'Publishing motor_command at {PUBLISH_HZ:.0f} Hz. Focus this terminal.\n'
            f'  Right/Left: +/- arrow_delta rad (hold); Up/Down: adjust delta '
            f'[{ARROW_DELTA_MIN}, {ARROW_DELTA_MAX}]\n'
            f'  Keys 2/4/6/8: one-shot dP to absolute 0, pi/2, -pi/2, pi (via motor_state)\n'
            f'  Kp={KP:.1f}  Kd={KD:.2f}  arrow_delta={DEFAULT_ARROW_DELTA:.3f}')

    def _motor_state_callback(self, msg: MotorState) -> None:
        self._state.update_motor_state(msg.position)

    def _stdin_loop(self) -> None:
        fd = sys.stdin.fileno()
        self._stdin_term_attrs = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._stdin_running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                key = self._read_key()
                if key is None:
                    continue
                if key == 'RIGHT':
                    self._state.touch_right()
                elif key == 'LEFT':
                    self._state.touch_left()
                elif key == 'UP':
                    self._state.touch_up()
                elif key == 'DOWN':
                    self._state.touch_down()
                elif key in NUMPAD_TARGETS:
                    if not self._state.set_numpad_target(
                            key, NUMPAD_TARGETS[key], self.get_logger().info):
                        self.get_logger().warn(
                            'No motor_state yet; wait for motor_node_continuous.')

        finally:
            if self._stdin_term_attrs is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._stdin_term_attrs)

    def _read_key(self) -> Optional[str]:
        ch = sys.stdin.read(1)
        if ch != '\x1b':
            return ch if len(ch) == 1 else None

        seq = ''
        for _ in range(16):
            if not select.select([sys.stdin], [], [], 0.03)[0]:
                break
            seq += sys.stdin.read(1)
            if seq and seq[-1] in 'ABCD':
                break

        if not seq:
            return None
        if seq[-1] == 'C':
            return 'RIGHT'
        if seq[-1] == 'D':
            return 'LEFT'
        if seq[-1] == 'A':
            return 'UP'
        if seq[-1] == 'B':
            return 'DOWN'
        return None

    def _on_timer(self) -> None:
        self._state.adjust_arrow_delta_if_held(self.get_logger().info)
        msg = MotorCommand()
        msg.position = float(self._state.get_position())
        msg.velocity = 0.0
        msg.kp = KP
        msg.kd = KD
        msg.torque = 0.0
        self._publisher.publish(msg)

    def destroy_node(self) -> None:
        self._stdin_running = False
        self._stdin_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardCommand()
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
