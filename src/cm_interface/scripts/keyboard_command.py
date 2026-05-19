#!/usr/bin/env python3
# Publishes motor_command at 100 Hz from keyboard input.
#
# Preferred: sudo apt install python3-pynput python3-evdev
# Fallback:  stdin/termios (works over SSH when this terminal has focus)

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
ARROW_HOLD_TIMEOUT_S = 0.15

NUMPAD_POSITIONS = {
    '2': 0.0,
    '4': math.pi / 2.0,
    '6': -math.pi / 2.0,
    '8': math.pi,
}

PYNPUT_IMPORT_ERROR: Optional[Exception] = None
try:
    from pynput import keyboard as pynput_keyboard
except ImportError as exc:
    pynput_keyboard = None  # type: ignore
    PYNPUT_IMPORT_ERROR = exc


class KeyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._right_held = False
        self._left_held = False
        self._pending_position: Optional[float] = None
        self._last_right_time = 0.0
        self._last_left_time = 0.0

    def set_right(self, held: bool) -> None:
        with self._lock:
            self._right_held = held
            if held:
                self._last_right_time = time.monotonic()

    def set_left(self, held: bool) -> None:
        with self._lock:
            self._left_held = held
            if held:
                self._last_left_time = time.monotonic()

    def touch_right(self) -> None:
        with self._lock:
            self._last_right_time = time.monotonic()

    def touch_left(self) -> None:
        with self._lock:
            self._last_left_time = time.monotonic()

    def set_pending(self, position: float, label: str, log_fn: Callable[[str], None]) -> None:
        with self._lock:
            self._pending_position = position
        log_fn(f'Key {label} -> position {position:.4f} rad (one shot)')

    def get_position(self, stdin_hold: bool) -> float:
        with self._lock:
            if self._pending_position is not None:
                position = self._pending_position
                self._pending_position = None
                return position

            if stdin_hold:
                now = time.monotonic()
                right = (now - self._last_right_time) < ARROW_HOLD_TIMEOUT_S
                left = (now - self._last_left_time) < ARROW_HOLD_TIMEOUT_S
            else:
                right = self._right_held
                left = self._left_held

            if right and not left:
                return ARROW_DELTA
            if left and not right:
                return -ARROW_DELTA
            return 0.0


class KeyboardCommand(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_command')

        self._publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self._state = KeyState()
        self._stdin_hold = False
        self._listener = None
        self._stdin_thread: Optional[threading.Thread] = None
        self._stdin_running = False
        self._stdin_term_attrs = None

        if pynput_keyboard is not None:
            self._start_pynput()
        elif sys.stdin.isatty():
            self._start_stdin()
        else:
            self._log_pynput_failure(fatal=True)
            raise RuntimeError('No keyboard backend available')

        period = 1.0 / PUBLISH_HZ
        self._timer = self.create_timer(period, self._on_timer)

        backend = 'pynput' if not self._stdin_hold else 'stdin (SSH-compatible)'
        self.get_logger().info(
            f'Keyboard backend: {backend}\n'
            f'Publishing motor_command at {PUBLISH_HZ:.0f} Hz. Focus this terminal.\n'
            f'  Right/Left arrow: +{ARROW_DELTA:.1f} / -{ARROW_DELTA:.1f} rad (hold)\n'
            f'  Keys 2/4/6/8: 0, pi/2, -pi/2, pi (single delta on press)\n'
            f'  Kp={KP:.1f}  Kd={KD:.2f}  (no keys -> position=0)')

    def _log_pynput_failure(self, fatal: bool = False) -> None:
        msg = (
            f'pynput import failed for {sys.executable}: {PYNPUT_IMPORT_ERROR}\n'
            '  Try: sudo apt install python3-pynput python3-evdev\n'
            '  Or run in an interactive SSH terminal (stdin fallback).'
        )
        if fatal:
            self.get_logger().fatal(msg)
        else:
            self.get_logger().warn(msg)

    def _start_pynput(self) -> None:
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_pynput_press,
            on_release=self._on_pynput_release,
        )
        self._listener.start()

    def _on_pynput_press(self, key) -> None:
        if key == pynput_keyboard.Key.right:
            self._state.set_right(True)
        elif key == pynput_keyboard.Key.left:
            self._state.set_left(True)
        elif hasattr(key, 'char') and key.char in NUMPAD_POSITIONS:
            self._state.set_pending(
                NUMPAD_POSITIONS[key.char], key.char, self.get_logger().info)

    def _on_pynput_release(self, key) -> None:
        if key == pynput_keyboard.Key.right:
            self._state.set_right(False)
        elif key == pynput_keyboard.Key.left:
            self._state.set_left(False)

    def _start_stdin(self) -> None:
        self._log_pynput_failure(fatal=False)
        self._stdin_hold = True
        self._stdin_running = True
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

    def _stdin_loop(self) -> None:
        fd = sys.stdin.fileno()
        self._stdin_term_attrs = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._stdin_running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                key = self._read_stdin_key()
                if key is None:
                    continue
                self._handle_stdin_key(key)
        finally:
            if self._stdin_term_attrs is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._stdin_term_attrs)

    def _read_stdin_key(self) -> Optional[str]:
        ch = sys.stdin.read(1)
        if ch != '\x1b':
            return ch
        if not select.select([sys.stdin], [], [], 0.01)[0]:
            return ch
        ch2 = sys.stdin.read(1)
        if ch2 != '[':
            return ch
        if not select.select([sys.stdin], [], [], 0.01)[0]:
            return ch
        ch3 = sys.stdin.read(1)
        if ch3 == 'C':
            return 'RIGHT'
        if ch3 == 'D':
            return 'LEFT'
        return None

    def _handle_stdin_key(self, key: str) -> None:
        if key == 'RIGHT':
            self._state.touch_right()
        elif key == 'LEFT':
            self._state.touch_left()
        elif key in NUMPAD_POSITIONS:
            self._state.set_pending(
                NUMPAD_POSITIONS[key], key, self.get_logger().info)

    def _on_timer(self) -> None:
        position = self._state.get_position(stdin_hold=self._stdin_hold)

        msg = MotorCommand()
        msg.position = float(position)
        msg.velocity = 0.0
        msg.kp = KP
        msg.kd = KD
        msg.torque = 0.0
        self._publisher.publish(msg)

    def destroy_node(self) -> None:
        self._stdin_running = False
        if self._stdin_thread is not None:
            self._stdin_thread.join(timeout=0.5)
        if self._listener is not None:
            self._listener.stop()
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
