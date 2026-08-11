#!/usr/bin/env python3
"""Run preset joint_despos waypoints on joystick button press (default: Start / button 7).

Loads waypoints from YAML (positions in degrees). For each waypoint: command poses,
wait until joint_curpos settles within tolerance, then optional delay_sec.
Publishes /joint_sequence/active while running so boom_joystick_control pauses teleop.

D-pad axis scrolls through presets; back button sets origin on all origin_namespaces.
Per-preset omega_max overrides joint_translator speed caps during sequence execution.

TWEAK controller mapping in module constants below (not launch parameters).
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

# TWEAK: joystick mapping for this node (edit here, not in launch files).
START_BUTTON_INDEX = 7
BACK_BUTTON_INDEX = 6
BACK_BUTTON_GRACE_SEC = 10.0  # ignore set-origin until joy/spurious edges settle
BACK_BUTTON_HOLD_SEC = 0.5  # require sustained back press before set-origin
JOY_GAP_SEC = 1.0  # resync button edges after joy dropout/reconnect
DPAD_VERTICAL_AXIS = 7
DPAD_AXIS_THRESHOLD = 0.5
ORIGIN_NAMESPACES = ''  # comma-separated; empty = all namespaces from presets


def clamp_hip_despos(despos: float, limit_rad: float) -> float:
    """Clamp desired hip position to ±limit_rad."""
    return max(-limit_rad, min(limit_rad, despos))


def parse_namespace_list(param: str) -> list[str]:
    """Parse comma-separated namespace list."""
    namespaces: list[str] = []
    for entry in param.split(','):
        ns = entry.strip()
        if ns:
            namespaces.append(ns)
    return namespaces


@dataclass
class Waypoint:
    delay_sec: float
    positions_deg: dict[str, float] = field(default_factory=dict)


@dataclass
class SequenceConfig:
    name: str
    position_tolerance_deg: float
    settle_timeout_sec: float
    waypoints: list[Waypoint]
    omega_max: dict[str, float] = field(default_factory=dict)


def _parse_sequence_config(raw: dict, *, source: str, name: str) -> SequenceConfig:
    if not isinstance(raw, dict):
        raise ValueError(f'{source} must be a mapping')

    tolerance_deg = float(raw.get('position_tolerance_deg', 2.0))
    settle_timeout_sec = float(raw.get('settle_timeout_sec', 30.0))
    if tolerance_deg <= 0.0:
        raise ValueError(f'{source}.position_tolerance_deg must be > 0')
    if settle_timeout_sec <= 0.0:
        raise ValueError(f'{source}.settle_timeout_sec must be > 0')

    omega_max: dict[str, float] = {}
    omega_raw = raw.get('omega_max')
    if omega_raw is not None:
        if not isinstance(omega_raw, dict):
            raise ValueError(f'{source}.omega_max must be a mapping')
        for ns, value in omega_raw.items():
            omega = float(value)
            if omega <= 0.0:
                raise ValueError(f'{source}.omega_max[{ns}] must be > 0')
            omega_max[str(ns).strip()] = omega

    waypoints_raw = raw.get('waypoints')
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
        raise ValueError(f'{source} must contain a non-empty waypoints list')

    waypoints: list[Waypoint] = []
    for index, entry in enumerate(waypoints_raw):
        if not isinstance(entry, dict):
            raise ValueError(f'{source}.waypoints[{index}] must be a mapping')
        delay_sec = float(entry.get('delay_sec', 0.0))
        if delay_sec < 0.0:
            raise ValueError(f'{source}.waypoints[{index}].delay_sec must be >= 0')
        positions_raw = entry.get('positions')
        if not isinstance(positions_raw, dict) or not positions_raw:
            raise ValueError(
                f'{source}.waypoints[{index}].positions must be a non-empty mapping')
        positions_deg = {str(ns).strip(): float(deg) for ns, deg in positions_raw.items()}
        waypoints.append(Waypoint(delay_sec=delay_sec, positions_deg=positions_deg))

    return SequenceConfig(
        name=name,
        position_tolerance_deg=tolerance_deg,
        settle_timeout_sec=settle_timeout_sec,
        waypoints=waypoints,
        omega_max=omega_max,
    )


def _load_yaml_root(path: str) -> dict:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f'sequence_file not found: {config_path}')

    with config_path.open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)

    if not isinstance(raw, dict):
        raise ValueError('sequence_file root must be a mapping')
    return raw


def load_presets_file(path: str) -> tuple[list[str], dict[str, SequenceConfig]]:
    """Load all presets; returns (ordered names, name -> config)."""
    raw = _load_yaml_root(path)
    presets = raw.get('presets')
    if presets is None:
        raise ValueError('sequence_file must contain a presets mapping')

    if not isinstance(presets, dict) or not presets:
        raise ValueError('sequence_file.presets must be a non-empty mapping')

    names = [str(name) for name in presets.keys()]
    configs = {
        name: _parse_sequence_config(
            presets[name],
            source=f"sequence_file.presets['{name}']",
            name=name,
        )
        for name in names
    }
    return names, configs


def load_sequence_config(path: str, sequence_name: str) -> SequenceConfig:
    raw = _load_yaml_root(path)

    presets = raw.get('presets')
    if presets is None:
        # Backward-compatibility: legacy single-sequence file.
        return _parse_sequence_config(raw, source='sequence_file', name=sequence_name)

    _, configs = load_presets_file(path)
    requested = sequence_name.strip()
    if not requested:
        raise ValueError('joint_sequence must be non-empty')

    config = configs.get(requested)
    if config is None:
        available = ', '.join(sorted(configs.keys()))
        raise ValueError(
            f"joint_sequence '{requested}' not found in sequence_file presets. "
            f'Available: {available}')
    return config


def collect_namespaces_from_configs(configs: dict[str, SequenceConfig]) -> list[str]:
    """Union of all namespaces referenced in any preset waypoint."""
    namespaces: list[str] = []
    seen: set[str] = set()
    for config in configs.values():
        for waypoint in config.waypoints:
            for ns in waypoint.positions_deg:
                if ns not in seen:
                    seen.add(ns)
                    namespaces.append(ns)
    return namespaces


class NamespaceHandle:
    """Per-namespace publishers and latest feedback."""

    def __init__(self, node: Node, namespace: str) -> None:
        self.namespace = namespace
        self.namespace_lower = namespace.lower()
        prefix = f'/{namespace}'
        self.despos_publisher = node.create_publisher(Float32, f'{prefix}/joint_despos', 10)
        self.hold_joint_publisher = node.create_publisher(Bool, f'{prefix}/hold_joint', 10)
        self.soft_mode_publisher = node.create_publisher(Bool, f'{prefix}/soft_mode', 10)
        self.sequence_omega_max_publisher = node.create_publisher(
            Float32, f'{prefix}/sequence_omega_max', 10
        )
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

    def publish_soft_mode(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.soft_mode_publisher.publish(msg)

    def publish_sequence_omega_max(self, omega_max: float) -> None:
        msg = Float32()
        msg.data = float(omega_max)
        self.sequence_omega_max_publisher.publish(msg)


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
        self.declare_parameter('joint_sequence', 'KneeTumble')
        self.declare_parameter('hip_angle_limit_deg', 90.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('active_publish_hz', 10.0)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        sequence_file = self.get_parameter('sequence_file').get_parameter_value().string_value.strip()
        if not sequence_file:
            sequence_file = default_sequence_file
        sequence_name = (
            self.get_parameter('joint_sequence').get_parameter_value().string_value.strip()
        )
        self._start_button_index = START_BUTTON_INDEX
        self._back_button_index = BACK_BUTTON_INDEX
        self._dpad_vertical_axis = DPAD_VERTICAL_AXIS
        self._dpad_axis_threshold = DPAD_AXIS_THRESHOLD
        if self._dpad_axis_threshold <= 0.0:
            raise ValueError('DPAD_AXIS_THRESHOLD must be > 0')
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

        self._preset_names, self._preset_configs = load_presets_file(sequence_file)
        if sequence_name not in self._preset_configs:
            available = ', '.join(sorted(self._preset_names))
            raise ValueError(
                f"joint_sequence '{sequence_name}' not found in sequence_file presets. "
                f'Available: {available}')
        self._selected_preset_index = self._preset_names.index(sequence_name)
        self._config = self._preset_configs[sequence_name]
        self._tolerance_rad = math.radians(self._config.position_tolerance_deg)

        origin_namespaces_param = ORIGIN_NAMESPACES
        self._origin_namespaces = parse_namespace_list(origin_namespaces_param)
        if not self._origin_namespaces:
            self._origin_namespaces = collect_namespaces_from_configs(self._preset_configs)

        all_namespaces = list(self._origin_namespaces)
        seen = set(all_namespaces)
        for ns in collect_namespaces_from_configs(self._preset_configs):
            if ns not in seen:
                seen.add(ns)
                all_namespaces.append(ns)

        self._handles = {ns: NamespaceHandle(self, ns) for ns in all_namespaces}

        self._active_publisher = self.create_publisher(Bool, '/joint_sequence/active', 10)
        self._sequence_active = False
        self._abort_requested = False
        self._prev_start_pressed = False
        self._prev_back_pressed = False
        self._back_button_enabled_at = time.monotonic() + BACK_BUTTON_GRACE_SEC
        self._back_hold_started_at: Optional[float] = None
        self._back_origin_done_for_press = False
        self._last_joy_time = time.monotonic()
        self._prev_dpad_direction = 0
        self._sequence_lock = threading.Lock()
        self._sequence_thread: Optional[threading.Thread] = None
        self._stack_ready = False

        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.create_subscription(Bool, '/boom_stack/ready', self._stack_ready_callback, 10)
        self.create_timer(1.0 / active_publish_hz, self._active_timer_callback)

        self.get_logger().info(
            f'Loaded {len(self._preset_names)} presets from {sequence_file}.\n'
            f'  Selected joint sequence: {self._config.name}\n'
            f'  Press button[{self._start_button_index}] to start (again to abort); '
            f'button[{self._back_button_index}] (hold {BACK_BUTTON_HOLD_SEC:.1f}s) to set origin; '
            f'axis[{self._dpad_vertical_axis}] to scroll presets.\n'
            f'  tolerance={self._config.position_tolerance_deg:.2f} deg, '
            f'settle_timeout={self._config.settle_timeout_sec:.1f} s, loop={self._loop}.\n'
            f'  Origin namespaces: {", ".join(self._origin_namespaces)}\n'
            f'  Waypoint namespaces: {", ".join(all_namespaces)}'
        )

    def _stack_ready_callback(self, msg: Bool) -> None:
        self._stack_ready = msg.data

    def _is_sequence_running(self) -> bool:
        with self._sequence_lock:
            return self._sequence_thread is not None and self._sequence_thread.is_alive()

    def _select_preset(self, index: int) -> None:
        self._selected_preset_index = index % len(self._preset_names)
        name = self._preset_names[self._selected_preset_index]
        self._config = self._preset_configs[name]
        self._tolerance_rad = math.radians(self._config.position_tolerance_deg)
        self.get_logger().info(f'Selected joint sequence: {name}')

    def _read_dpad_direction(self, msg: Joy) -> int:
        if self._dpad_vertical_axis >= len(msg.axes):
            return 0
        axis_value = float(msg.axes[self._dpad_vertical_axis])
        if axis_value > self._dpad_axis_threshold:
            return 1
        if axis_value < -self._dpad_axis_threshold:
            return -1
        return 0

    def _handle_dpad_scroll(self, msg: Joy) -> None:
        if self._is_sequence_running():
            return

        direction = self._read_dpad_direction(msg)
        if direction == 0 or direction == self._prev_dpad_direction:
            self._prev_dpad_direction = direction
            return

        self._prev_dpad_direction = direction
        if direction < 0:
            self._select_preset(self._selected_preset_index - 1)
        else:
            self._select_preset(self._selected_preset_index + 1)

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
        now = time.monotonic()
        joy_gap = now - self._last_joy_time > JOY_GAP_SEC
        self._last_joy_time = now

        back_pressed = (
            self._back_button_index < len(msg.buttons)
            and msg.buttons[self._back_button_index] == 1
        )
        start_pressed = (
            self._start_button_index < len(msg.buttons)
            and msg.buttons[self._start_button_index] == 1
        )

        if joy_gap:
            # Resync edges after controller reconnect; spurious rising edges are common.
            self._prev_back_pressed = back_pressed
            self._prev_start_pressed = start_pressed
            self._back_button_enabled_at = now + BACK_BUTTON_GRACE_SEC
            self._back_hold_started_at = None
            self._back_origin_done_for_press = False
            self._handle_dpad_scroll(msg)
            return

        self._handle_dpad_scroll(msg)

        if back_pressed:
            if self._back_hold_started_at is None:
                self._back_hold_started_at = now
            elif (
                not self._back_origin_done_for_press
                and now - self._back_hold_started_at >= BACK_BUTTON_HOLD_SEC
                and now >= self._back_button_enabled_at
            ):
                self._back_origin_done_for_press = True
                self._handle_set_origin()
        else:
            self._back_hold_started_at = None
            self._back_origin_done_for_press = False
        self._prev_back_pressed = back_pressed

        rising_edge = start_pressed and not self._prev_start_pressed
        self._prev_start_pressed = start_pressed

        if not rising_edge:
            return

        if not self._stack_ready:
            self.get_logger().warn('Sequence refused: stack is not ready.')
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

    def _handle_set_origin(self) -> None:
        if not self._stack_ready:
            self.get_logger().warn('Set origin refused: stack is not ready.')
            return
        if self._is_sequence_running():
            self.get_logger().warn('Set origin refused: joint sequence is running.')
            return

        for namespace in self._origin_namespaces:
            handle = self._handles.get(namespace)
            if handle is None:
                self.get_logger().warn(f'Set origin skipped unknown namespace: {namespace}')
                continue
            handle.publish_soft_mode(True)

        time.sleep(0.1)

        for namespace in self._origin_namespaces:
            handle = self._handles.get(namespace)
            if handle is None:
                continue
            handle.publish_soft_mode(False)

        time.sleep(0.1)

        for namespace in self._origin_namespaces:
            handle = self._handles.get(namespace)
            if handle is None:
                continue
            has_curpos, curpos_rad, _, _ = handle.get_state()
            if has_curpos:
                handle.publish_despos_rad(curpos_rad)
            handle.publish_hold_joint(True)

        self.get_logger().info(
            f'Set origin at current position for: {", ".join(self._origin_namespaces)}'
        )

    def _apply_preset_omega_max(self) -> set[str]:
        applied: set[str] = set()
        for namespace, omega in self._config.omega_max.items():
            handle = self._handles.get(namespace)
            if handle is None:
                self.get_logger().warn(f'No handle for omega_max namespace: {namespace}')
                continue
            handle.publish_sequence_omega_max(omega)
            applied.add(namespace)
            self.get_logger().info(
                f'Sequence omega_max for {namespace}: {omega:.3f} motor rad/s'
            )
        return applied

    def _restore_omega_max(self, namespaces: set[str]) -> None:
        for namespace in namespaces:
            handle = self._handles.get(namespace)
            if handle is None:
                continue
            handle.publish_sequence_omega_max(0.0)
            self.get_logger().info(f'Restored omega_max for {namespace}')

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

    def _out_of_tolerance_errors(
        self, targets_rad: dict[str, float]
    ) -> list[tuple[str, Optional[float], float, Optional[float]]]:
        """Return (namespace, curpos_deg, target_deg, error_deg) for motors outside tolerance."""
        errors: list[tuple[str, Optional[float], float, Optional[float]]] = []
        for namespace, target_rad in targets_rad.items():
            handle = self._handles[namespace]
            has_curpos, curpos_rad, _, _ = handle.get_state()
            target_deg = math.degrees(target_rad)
            if not has_curpos:
                errors.append((namespace, None, target_deg, None))
                continue
            error_rad = curpos_rad - target_rad
            if abs(error_rad) > self._tolerance_rad:
                errors.append((
                    namespace,
                    math.degrees(curpos_rad),
                    target_deg,
                    math.degrees(error_rad),
                ))
        return errors

    def _log_settle_timeout_errors(
        self, targets_rad: dict[str, float], waypoint_index: int
    ) -> None:
        self.get_logger().warn(
            f'Waypoint {waypoint_index} did not settle within '
            f'{self._config.settle_timeout_sec:.1f} s.'
        )
        tolerance_deg = self._config.position_tolerance_deg
        for namespace, curpos_deg, target_deg, error_deg in self._out_of_tolerance_errors(
            targets_rad
        ):
            if curpos_deg is None or error_deg is None:
                self.get_logger().error(
                    f'  {namespace}: no curpos feedback (target={target_deg:.2f} deg)'
                )
                continue
            self.get_logger().error(
                f'  {namespace}: curpos={curpos_deg:.2f} deg, '
                f'target={target_deg:.2f} deg, error={error_deg:+.2f} deg '
                f'(tolerance=±{tolerance_deg:.2f} deg)'
            )

    def _wait_for_settle(self, targets_rad: dict[str, float], waypoint_index: int) -> bool:
        deadline = time.monotonic() + self._config.settle_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._abort_requested:
                return False
            if self._all_settled(targets_rad):
                return True
            time.sleep(0.05)
        self._log_settle_timeout_errors(targets_rad, waypoint_index)
        return False

    def _hold_all_at_curpos(self) -> None:
        for handle in self._handles.values():
            has_curpos, curpos_rad, _, _ = handle.get_state()
            if has_curpos:
                handle.publish_despos_rad(curpos_rad)
            handle.publish_hold_joint(True)

    def _run_sequence(self) -> None:
        self._set_active(True)
        self.get_logger().info(f"Joint sequence started: {self._config.name}")
        applied_omega_max: set[str] = set()

        try:
            if not self._stack_ready:
                self.get_logger().warn('Sequence refused: stack is not ready.')
                return
            if self._any_soft_mode():
                self.get_logger().warn('Sequence refused: soft_mode is active on one or more joints.')
                return

            applied_omega_max = self._apply_preset_omega_max()
            time.sleep(0.05)

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
            self._restore_omega_max(applied_omega_max)
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
