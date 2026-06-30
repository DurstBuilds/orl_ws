#!/usr/bin/env python3
"""Run preset joint_despos waypoints on joystick button press (default: Start / button 7).

Loads waypoints from YAML (positions in degrees). For each waypoint: command poses,
wait until joint_curpos settles within tolerance, then optional delay_sec.
Publishes /joint_sequence/active while running so boom_joystick_control pauses teleop.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32


def clamp_hip_despos(despos: float, limit_rad: float) -> float:
    """Clamp desired hip position to ±limit_rad."""
    return max(-limit_rad, min(limit_rad, despos))


@dataclass
class Waypoint:
  delay_sec: float
  positions_deg: dict[str, float] = field(default_factory=dict)


@dataclass
class SequenceConfig:
  position_tolerance_deg: float
  settle_timeout_sec: float
  waypoints: list[Waypoint]


def load_sequence_config(path: str) -> SequenceConfig:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f'sequence_file not found: {config_path}')

    with config_path.open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)

    if not isinstance(raw, dict):
        raise ValueError('sequence_file root must be a mapping')

    tolerance_deg = float(raw.get('position_tolerance_deg', 2.0))
    settle_timeout_sec = float(raw.get('settle_timeout_sec', 30.0))
    if tolerance_deg <= 0.0:
        raise ValueError('position_tolerance_deg must be > 0')
    if settle_timeout_sec <= 0.0:
        raise ValueError('settle_timeout_sec must be > 0')

    waypoints_raw = raw.get('waypoints')
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
        raise ValueError('sequence_file must contain a non-empty waypoints list')

    waypoints: list[Waypoint] = []
    for index, entry in enumerate(waypoints_raw):
        if not isinstance(entry, dict):
            raise ValueError(f'waypoints[{index}] must be a mapping')
        delay_sec = float(entry.get('delay_sec', 0.0))
        if delay_sec < 0.0:
            raise ValueError(f'waypoints[{index}].delay_sec must be >= 0')
        positions_raw = entry.get('positions')
        if not isinstance(positions_raw, dict) or not positions_raw:
            raise ValueError(f'waypoints[{index}].positions must be a non-empty mapping')
        positions_deg = {str(ns).strip(): float(deg) for ns, deg in positions_raw.items()}
        waypoints.append(Waypoint(delay_sec=delay_sec, positions_deg=positions_deg))

    return SequenceConfig(
        position_tolerance_deg=tolerance_deg,
        settle_timeout_sec=settle_timeout_sec,
        waypoints=waypoints,
    )


class NamespaceHandle:
    """Per-namespace publishers and latest feedback."""

    def __init__(self, node: Node, namespace: str) -> None:
        self.namespace = namespace
        self.namespace_lower = namespace.lower()
        prefix = f'/{namespace}'
        self.despos_publisher = node.create_publisher(Float32, f'{prefix}/joint_despos', 10)
        self.hold_joint_publisher = node.create_publisher(Bool, f'{prefix}/hold_joint', 10)
        self._lock = threading.Lock()
        self.curpos_rad = 0.0
        self.has_curpos = False
        self.soft_mode = False
        self.has_soft_mode = False
        node.create_subscription(
            Float32, f'{prefix}/joint_curpos', self._curpos_callback, 10
        )
        node.create_subscription(
            Bool, f'{prefix}/soft_mode', self._soft_mode_callback, 10
        )

    def _curpos_callback(self, msg: Float32) -> None:
        with self._lock:
            self.curpos_rad = msg.data
            self.has_curpos = True

    def _soft_mode_callback(self, msg: Bool) -> None:
        with self._lock:
            self.soft_mode = msg.data
            self.has_soft_mode = True

    def get_state(self) -> tuple[bool, float, bool, bool]:
        with self._lock:
            return self.has_curpos, self.curpos_rad, self.has_soft_mode, self.soft_mode

    def publish_hold_joint(self, hold: bool) -> None:
        msg = Bool()
        msg.data = hold
        self.hold_joint_publisher.publish(msg)

    def publish_despos_rad(self, despos_rad: float) -> None:
        msg = Float32()
        msg.data = float(despos_rad)
        self.despos_publisher.publish(msg)


class JointPositionSequence(Node):
    """Joystick-triggered joint position waypoint runner."""

    def __init__(self) -> None:
        super().__init__('joint_position_sequence_node')

        default_sequence_file = str(
            Path(get_package_share_directory('cm_interface'))
            / 'config'
            / 'joint_sequence_presets.yaml'
        )

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('sequence_file', default_sequence_file)
        self.declare_parameter('start_button_index', 7)
        self.declare_parameter('hip_angle_limit_deg', 45.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('active_publish_hz', 10.0)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        sequence_file = self.get_parameter('sequence_file').get_parameter_value().string_value.strip()
        if not sequence_file:
            sequence_file = default_sequence_file
        self._start_button_index = (
            self.get_parameter('start_button_index').get_parameter_value().integer_value
        )
        hip_angle_limit_deg = (
            self.get_parameter('hip_angle_limit_deg').get_parameter_value().double_value
        )
        if hip_angle_limit_deg < 0.0:
            raise ValueError('hip_angle_limit_deg must be >= 0')
        self._hip_angle_limit_rad = math.radians(hip_angle_limit_deg)
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value
        active_publish_hz = (
            self.get_parameter('active_publish_hz').get_parameter_value().double_value
        )
        if active_publish_hz <= 0.0:
            raise ValueError('active_publish_hz must be > 0')

        self._config = load_sequence_config(sequence_file)
        self._tolerance_rad = math.radians(self._config.position_tolerance_deg)

        namespaces: list[str] = []
        seen: set[str] = set()
        for waypoint in self._config.waypoints:
            for ns in waypoint.positions_deg:
                if ns not in seen:
                    seen.add(ns)
                    namespaces.append(ns)

        self._handles = {ns: NamespaceHandle(self, ns) for ns in namespaces}

        self._active_publisher = self.create_publisher(Bool, '/joint_sequence/active', 10)
        self._sequence_active = False
        self._abort_requested = False
        self._prev_start_pressed = False
        self._sequence_lock = threading.Lock()
        self._sequence_thread: Optional[threading.Thread] = None

        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / active_publish_hz, self._active_timer_callback)

        self.get_logger().info(
            f'Loaded {len(self._config.waypoints)} waypoints from {sequence_file}.\n'
            f'  Press button[{self._start_button_index}] to start (again to abort); '
            f'tolerance={self._config.position_tolerance_deg:.2f} deg, '
            f'settle_timeout={self._config.settle_timeout_sec:.1f} s, loop={self._loop}.\n'
            f'  Namespaces: {", ".join(namespaces)}'
        )

    def _set_active(self, active: bool) -> None:
        self._sequence_active = active
        msg = Bool()
        msg.data = active
        self._active_publisher.publish(msg)

    def _active_timer_callback(self) -> None:
        if self._sequence_active:
            msg = Bool()
            msg.data = True
            self._active_publisher.publish(msg)

    def _joy_callback(self, msg: Joy) -> None:
        start_pressed = (
            self._start_button_index < len(msg.buttons)
            and msg.buttons[self._start_button_index] == 1
        )
        rising_edge = start_pressed and not self._prev_start_pressed
        self._prev_start_pressed = start_pressed

        if not rising_edge:
            return

        with self._sequence_lock:
            if self._sequence_thread is not None and self._sequence_thread.is_alive():
                self._abort_requested = True
                self.get_logger().info('Abort requested.')
                return
            self._abort_requested = False
            self._sequence_thread = threading.Thread(
                target=self._run_sequence, daemon=True
            )
            self._sequence_thread.start()

    def _any_soft_mode(self) -> bool:
        for handle in self._handles.values():
            has_soft_mode, soft_mode = handle.get_state()[2:]
            if has_soft_mode and soft_mode:
                return True
        return False

    def _target_rad_for_namespace(
        self, namespace: str, target_deg: float
    ) -> float:
        target_rad = math.radians(target_deg)
        if 'hip' in namespace.lower():
            target_rad = clamp_hip_despos(target_rad, self._hip_angle_limit_rad)
        return target_rad

    def _publish_waypoint(self, waypoint: Waypoint) -> dict[str, float]:
        targets_rad: dict[str, float] = {}
        for namespace, target_deg in waypoint.positions_deg.items():
            handle = self._handles.get(namespace)
            if handle is None:
                self.get_logger().warn(f'Unknown namespace in waypoint: {namespace}')
                continue
            target_rad = self._target_rad_for_namespace(namespace, target_deg)
            targets_rad[namespace] = target_rad
            handle.publish_hold_joint(False)
            handle.publish_despos_rad(target_rad)
        return targets_rad

    def _all_settled(self, targets_rad: dict[str, float]) -> bool:
        for namespace, target_rad in targets_rad.items():
            handle = self._handles[namespace]
            has_curpos, curpos_rad, _, _ = handle.get_state()
            if not has_curpos:
                return False
            if abs(curpos_rad - target_rad) > self._tolerance_rad:
                return False
        return True

    def _wait_for_settle(self, targets_rad: dict[str, float], waypoint_index: int) -> bool:
        deadline = time.monotonic() + self._config.settle_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._abort_requested:
                return False
            if self._all_settled(targets_rad):
                return True
            time.sleep(0.05)
        self.get_logger().warn(
            f'Waypoint {waypoint_index} did not settle within '
            f'{self._config.settle_timeout_sec:.1f} s.'
        )
        return False

    def _hold_all_at_curpos(self) -> None:
        for handle in self._handles.values():
            has_curpos, curpos_rad, _, _ = handle.get_state()
            if has_curpos:
                handle.publish_despos_rad(curpos_rad)
            handle.publish_hold_joint(True)

    def _run_sequence(self) -> None:
        self._set_active(True)
        self.get_logger().info('Joint sequence started.')

        try:
            if self._any_soft_mode():
                self.get_logger().warn('Sequence refused: soft_mode is active on one or more joints.')
                return

            while rclpy.ok():
                for index, waypoint in enumerate(self._config.waypoints):
                    if self._abort_requested:
                        self.get_logger().info('Sequence aborted.')
                        return
                    if self._any_soft_mode():
                        self.get_logger().warn('Sequence stopped: soft_mode became active.')
                        return

                    self.get_logger().info(f'Waypoint {index + 1}/{len(self._config.waypoints)}')
                    targets_rad = self._publish_waypoint(waypoint)
                    if not targets_rad:
                        continue

                    if not self._wait_for_settle(targets_rad, index + 1):
                        if self._abort_requested:
                            self.get_logger().info('Sequence aborted.')
                        return

                    if waypoint.delay_sec > 0.0:
                        delay_end = time.monotonic() + waypoint.delay_sec
                        while rclpy.ok() and time.monotonic() < delay_end:
                            if self._abort_requested:
                                self.get_logger().info('Sequence aborted.')
                                return
                            time.sleep(0.05)

                if not self._loop:
                    break
                self.get_logger().info('Looping sequence.')

            self.get_logger().info('Joint sequence completed.')
        finally:
            self._hold_all_at_curpos()
            self._set_active(False)
            with self._sequence_lock:
                self._abort_requested = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointPositionSequence()
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
