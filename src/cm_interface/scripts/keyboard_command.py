#!/usr/bin/env python3
# Publishes motor_command at 100 Hz from keyboard input over SSH (stdin/termios).

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

from motor_interfaces.msg import MotorCommand

KP = 5.0
KD = 0.02
ARROW_DELTA = 0.1
PUBLISH_HZ = 100.0
ARROW_HOLD_TIMEOUT_S = 0.4

NUMPAD_POSITIONS = {
    '2': 0.0,
    '4': math.pi / 2.0,
    '6': -math.pi / 2.0,
    '8': math.pi,
}


class KeyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending_position: Optional[float] = None
        self._last_right_time = 0.0
        self._last_left_time = 0.0

    def touch_right(self) -> None:
        with self._lock:
            self._last_right_time = time.monotonic()
            self._last_left_time = 0.0

    def touch_left(self) -> None:
        with self._lock:
            self._last_left_time = time.monotonic()
            self._last_right_time = 0.0

    def set_pending(self, position: float, label: str, log_fn: Callable[[str], None]) -> None:
        with self._lock:
            self._pending_position = position
        log_fn(f'Key {label} -> position {position:.4f} rad (one shot)')

    def get_position(self) -> float:
        with self._lock:
            if self._pending_position is not None:
                position = self._pending_position
                self._pending_position = None
                return position

            now = time.monotonic()
            right = (now - self._last_right_time) < ARROW_HOLD_TIMEOUT_S
            left = (now - self._last_left_time) < ARROW_HOLD_TIMEOUT_S

            if right and not left:
                return ARROW_DELTA
            if left and not right:
                return -ARROW_DELTA
            return 0.0


class KeyboardCommand(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_command')

        if not sys.stdin.isatty():
            self.get_logger().fatal(
                'stdin is not a TTY. Run in an interactive SSH terminal.')
            raise RuntimeError('stdin is not a TTY')

        self._publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self._state = KeyState()
        self._stdin_running = True
        self._stdin_term_attrs = None
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_timer)

        self.get_logger().info(
            f'Publishing motor_command at {PUBLISH_HZ:.0f} Hz. Focus this terminal.\n'
            f'  Right/Left arrow: +{ARROW_DELTA:.1f} / -{ARROW_DELTA:.1f} rad (hold)\n'
            f'  Keys 2/4/6/8: 0, pi/2, -pi/2, pi (single delta on press)\n'
            f'  Kp={KP:.1f}  Kd={KD:.2f}  (no keys -> position=0)')

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
                elif key in NUMPAD_POSITIONS:
                    self._state.set_pending(
                        NUMPAD_POSITIONS[key], key, self.get_logger().info)
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
        return None

    def _on_timer(self) -> None:
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
