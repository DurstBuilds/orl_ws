#!/usr/bin/env python3
# Publishes motor_command at 100 Hz from keyboard input.
# Requires: sudo apt install python3-pynput  OR  pip install pynput

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import MotorCommand

try:
    from pynput import keyboard
except ImportError:
    keyboard = None  # type: ignore

KP = 5.0
KD = 0.02
ARROW_DELTA = 0.1
PUBLISH_HZ = 100.0

NUMPAD_POSITIONS = {
    '2': 0.0,
    '4': math.pi / 2.0,
    '6': -math.pi / 2.0,
    '8': math.pi,
}


class KeyboardCommand(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_command')

        if keyboard is None:
            self.get_logger().fatal(
                'pynput not installed. Run: pip install pynput')
            raise RuntimeError('pynput not installed')

        self._publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self._lock = threading.Lock()
        self._right_held = False
        self._left_held = False
        self._pending_position: Optional[float] = None

        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

        period = 1.0 / PUBLISH_HZ
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Publishing motor_command at {PUBLISH_HZ:.0f} Hz. Focus this terminal.\n'
            f'  Right/Left arrow: +{ARROW_DELTA:.1f} / -{ARROW_DELTA:.1f} rad (hold)\n'
            f'  Numpad 2/4/8: 0, pi/2, pi (single delta on press)\n'
            f'  Kp={KP:.1f}  Kd={KD:.2f}  (no keys -> position=0)')

    def _on_press(self, key) -> None:
        with self._lock:
            if key == keyboard.Key.right:
                self._right_held = True
            elif key == keyboard.Key.left:
                self._left_held = True
            elif hasattr(key, 'char') and key.char in NUMPAD_POSITIONS:
                self._pending_position = NUMPAD_POSITIONS[key.char]
                self.get_logger().info(
                    f'Numpad {key.char} -> position {self._pending_position:.4f} rad (one shot)')

    def _on_release(self, key) -> None:
        with self._lock:
            if key == keyboard.Key.right:
                self._right_held = False
            elif key == keyboard.Key.left:
                self._left_held = False

    def _arrow_position(self) -> float:
        if self._right_held and not self._left_held:
            return ARROW_DELTA
        if self._left_held and not self._right_held:
            return -ARROW_DELTA
        return 0.0

    def _on_timer(self) -> None:
        with self._lock:
            if self._pending_position is not None:
                position = self._pending_position
                self._pending_position = None
            else:
                position = self._arrow_position()

        msg = MotorCommand()
        msg.position = float(position)
        msg.velocity = 0.0
        msg.kp = KP
        msg.kd = KD
        msg.torque = 0.0
        self._publisher.publish(msg)

    def destroy_node(self) -> None:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
