"""Full boom stack: CAN gateway (or legacy per-motor nodes) + translator pipelines + teleop.

Motor list lives in boom_motor_config.BOOM_MOTOR_STACKS (single source of truth).
Edit that file to add/remove drives; defaults for gateway, teleop, bag topics, and
namespace_omega_max are derived from it automatically.

Modes:
  use_can_gateway:=true  (default) — one can_gateway_node on can_interface; per-ns
                           motor_unwrapper_node + joint_translator_node.
  use_can_gateway:=false — legacy motor_node_continuous per namespace (debug only;
                           multiple sockets on one bus can conflict).
"""

import re
import sys
from pathlib import Path

# Allow importing boom_motor_config from this directory when installed under share/.
_launch_dir = Path(__file__).resolve().parent
if str(_launch_dir) not in sys.path:
    sys.path.insert(0, str(_launch_dir))

from boom_motor_config import (  # noqa: E402
    BOOM_MOTOR_STACKS,
    DEFAULT_CAN_IDS,
    DEFAULT_GATEWAY_ALIVE_CHECK_PERIOD_MS,
    DEFAULT_GATEWAY_RECONNECT_COOLDOWN_MS,
    DEFAULT_MOTOR_MODELS,
    DEFAULT_NAMESPACES,
    DEFAULT_NAMESPACE_GEAR_RATIOS,
    DEFAULT_NAMESPACE_OMEGA_MAX,
    JOINT_CURPOS_TOPICS,
    MOTOR_COMMAND_TOPICS,
    MOTOR_STATE_TOPICS,
    omega_max_for_namespace,
    origin_jump_threshold_for_stack,
    parse_namespace_omega_max,
)

IMU_ACCEL_TOPIC = '/IMU_Acceleration'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _logging_enabled(context) -> bool:
    """True when enable_logging launch arg is true/1/yes."""
    value = LaunchConfiguration('enable_logging').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _imu_enabled(context) -> bool:
    """True when enable_IMU launch arg is true/1/yes."""
    value = LaunchConfiguration('enable_IMU').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _joint_sequence_enabled(context) -> bool:
    """True when enable_joint_sequence launch arg is true/1/yes."""
    value = LaunchConfiguration('enable_joint_sequence').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _use_can_gateway(context) -> bool:
    """True when use_can_gateway launch arg is true/1/yes."""
    value = LaunchConfiguration('use_can_gateway').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _parse_namespace_omega_max(context) -> dict[str, str]:
    """Parse namespace_omega_max launch arg; omega_max is fallback for unlisted namespaces."""
    default = LaunchConfiguration('omega_max').perform(context).strip()
    param = LaunchConfiguration('namespace_omega_max').perform(context).strip()
    try:
        return parse_namespace_omega_max(param, default)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc


def _parse_hip_angle_limit_deg(context) -> float:
    """Parse and validate hip_angle_limit_deg (must be >= 0)."""
    raw = LaunchConfiguration('hip_angle_limit_deg').perform(context).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise RuntimeError(
            f"hip_angle_limit_deg must be a number, got '{raw}'"
        ) from exc
    if value < 0.0:
        raise RuntimeError(f'hip_angle_limit_deg must be >= 0, got {value}')
    return value


def _collect_bag_indices(base: str, bag_dir: Path) -> list[int]:
    """Find existing bag index suffixes for auto-increment naming."""
    indices: list[int] = []
    escaped = re.escape(base)

    for path in bag_dir.glob(f'{base}_*.mcap'):
        match = re.match(rf'^{escaped}_(\d+)\.mcap$', path.name)
        if match:
            indices.append(int(match.group(1)))

    if bag_dir.is_dir():
        for path in bag_dir.iterdir():
            if not path.is_dir():
                continue
            match = re.match(rf'^{escaped}_(\d+)$', path.name)
            if match:
                indices.append(int(match.group(1)))

    legacy_dir = bag_dir / base
    if legacy_dir.is_dir():
        for path in legacy_dir.glob(f'{base}_*.mcap'):
            match = re.match(rf'^{escaped}_(\d+)\.mcap$', path.name)
            if match:
                indices.append(int(match.group(1)))

    return indices


def _next_bag_output_uri(base: str, bag_dir: str) -> str:
    """Return next bag folder name base_N under bag_dir."""
    root = Path(bag_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    indices = _collect_bag_indices(base, root)
    next_index = max(indices) + 1 if indices else 0
    return f'{base}_{next_index}'


def _joint_angle_limit_for_stack(stack: dict, hip_angle_limit_deg: float) -> float:
    """Joint limit (deg) for joint_translator_node; hip namespaces use launch hip limit."""
    if 'hip' in stack['ns']:
        return hip_angle_limit_deg
    return stack['joint_angle_limit_deg']


def _translator_stack_group(
    stack: dict, hip_angle_limit_deg: float, omega_max: str
) -> GroupAction:
    """Per-motor pipeline when can_gateway owns CAN: unwrapper + joint translator."""
    joint_angle_limit_deg = _joint_angle_limit_for_stack(stack, hip_angle_limit_deg)
    return GroupAction([
        PushRosNamespace(stack['ns']),
        Node(
            package='cm_interface',
            executable='motor_unwrapper_node',
            name='motor_unwrapper_node',
            parameters=[{
                'origin_jump_threshold': origin_jump_threshold_for_stack(stack),
            }],
        ),
        Node(
            package='cm_interface',
            executable='joint_translator_node',
            name='joint_translator_node',
            parameters=[{
                'motor_model': stack['motor_model'],
                'gear_ratio': stack['gear_ratio'],
                'omega_max': omega_max,
                'joint_angle_limit_deg': joint_angle_limit_deg,
                'motor_error_tolerance': ParameterValue(
                    LaunchConfiguration('motor_error_tolerance'), value_type=float
                ),
            }],
        ),
    ])


def _legacy_motor_stack_group(
    stack: dict,
    motor_startup_delay_ms: int,
    hip_angle_limit_deg: float,
    omega_max: str,
) -> GroupAction:
    """Per-motor pipeline in legacy mode: motor_node_continuous + unwrapper + translator."""
    joint_angle_limit_deg = _joint_angle_limit_for_stack(stack, hip_angle_limit_deg)
    return GroupAction([
        PushRosNamespace(stack['ns']),
        Node(
            package='cm_interface',
            executable='motor_node_continuous',
            name='motor_node_continuous',
            parameters=[{
                'motor_model': stack['motor_model'],
                'can_id': stack['can_id'],
                'can_interface': LaunchConfiguration('can_interface'),
                'tx_rate_hz': ParameterValue(
                    LaunchConfiguration('motor_tx_rate_hz'), value_type=float
                ),
                'feedback_timeout_ms': ParameterValue(
                    LaunchConfiguration('motor_feedback_timeout_ms'), value_type=int
                ),
                'feedback_poll_ms': ParameterValue(
                    LaunchConfiguration('motor_feedback_poll_ms'), value_type=int
                ),
                'startup_delay_ms': motor_startup_delay_ms,
                'use_can_filters': True,
            }],
        ),
        Node(
            package='cm_interface',
            executable='motor_unwrapper_node',
            name='motor_unwrapper_node',
            parameters=[{
                'origin_jump_threshold': origin_jump_threshold_for_stack(stack),
            }],
        ),
        Node(
            package='cm_interface',
            executable='joint_translator_node',
            name='joint_translator_node',
            parameters=[{
                'motor_model': stack['motor_model'],
                'gear_ratio': stack['gear_ratio'],
                'omega_max': omega_max,
                'joint_angle_limit_deg': joint_angle_limit_deg,
                'motor_error_tolerance': ParameterValue(
                    LaunchConfiguration('motor_error_tolerance'), value_type=float
                ),
            }],
        ),
    ])


def _launch_setup(context, *args, **kwargs):
    """Build node list from BOOM_MOTOR_STACKS and launch arguments."""
    bag_dir = LaunchConfiguration('bag_output_dir').perform(context)
    hip_angle_limit_deg = _parse_hip_angle_limit_deg(context)
    omega_max_default = LaunchConfiguration('omega_max').perform(context).strip()
    namespace_omega_max = _parse_namespace_omega_max(context)

    def _stack_omega_max(stack: dict) -> str:
        return omega_max_for_namespace(stack['ns'], namespace_omega_max, omega_max_default)

    actions = []

    if _use_can_gateway(context):
        actions.append(
            Node(
                package='cm_interface',
                executable='can_gateway_node',
                name='can_gateway_node',
                output='screen',
                parameters=[{
                    'can_interface': LaunchConfiguration('can_interface'),
                    'namespaces': DEFAULT_NAMESPACES,
                    'motor_models': DEFAULT_MOTOR_MODELS,
                    'can_ids': DEFAULT_CAN_IDS,
                    'loop_rate_hz': ParameterValue(
                        LaunchConfiguration('gateway_loop_rate_hz'), value_type=float
                    ),
                    'feedback_timeout_ms': ParameterValue(
                        LaunchConfiguration('motor_feedback_timeout_ms'), value_type=int
                    ),
                    'feedback_poll_ms': ParameterValue(
                        LaunchConfiguration('motor_feedback_poll_ms'), value_type=int
                    ),
                    'startup_stagger_ms': ParameterValue(
                        LaunchConfiguration('gateway_startup_stagger_ms'), value_type=int
                    ),
                    'enable_settle_ms': ParameterValue(
                        LaunchConfiguration('gateway_enable_settle_ms'), value_type=int
                    ),
                    'ak80_enable_settle_ms': ParameterValue(
                        LaunchConfiguration('gateway_ak80_enable_settle_ms'), value_type=int
                    ),
                    'startup_origin_poll_ms': ParameterValue(
                        LaunchConfiguration('gateway_startup_origin_poll_ms'), value_type=int
                    ),
                    'bus_warmup_ms': ParameterValue(
                        LaunchConfiguration('gateway_bus_warmup_ms'), value_type=int
                    ),
                    'alive_check_period_ms': ParameterValue(
                        LaunchConfiguration('gateway_alive_check_period_ms'), value_type=int
                    ),
                    'reconnect_cooldown_ms': ParameterValue(
                        LaunchConfiguration('gateway_reconnect_cooldown_ms'), value_type=int
                    ),
                }],
            )
        )
        actions.extend(
            _translator_stack_group(stack, hip_angle_limit_deg, _stack_omega_max(stack))
            for stack in BOOM_MOTOR_STACKS
        )
    else:
        stagger_ms = int(LaunchConfiguration('motor_startup_stagger_ms').perform(context))
        if stagger_ms < 0:
            stagger_ms = 0
        actions.extend(
            _legacy_motor_stack_group(
                stack, index * stagger_ms, hip_angle_limit_deg, _stack_omega_max(stack)
            )
            for index, stack in enumerate(BOOM_MOTOR_STACKS)
        )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    boom_joystick_control_node = Node(
        package='cm_interface',
        executable='boom_joystick_control_node',
        name='boom_joystick_control_node',
        parameters=[{
            'namespaces': LaunchConfiguration('namespaces'),
            'namespace_gear_ratios': LaunchConfiguration('namespace_gear_ratios'),
            'publish_hz': ParameterValue(LaunchConfiguration('publish_hz'), value_type=float),
            'hip_angle_limit_deg': hip_angle_limit_deg,
            'test_kp': ParameterValue(LaunchConfiguration('test_kp'), value_type=float),
            'knee_translator_node': 'knee_motor/joint_translator_node',
        }],
    )

    actions.extend([joy_node, boom_joystick_control_node])

    if _joint_sequence_enabled(context):
        actions.append(
            Node(
                package='cm_interface',
                executable='joint_position_sequence_node',
                name='joint_position_sequence_node',
                parameters=[{
                    'sequence_file': LaunchConfiguration('sequence_file'),
                    'joint_sequence': LaunchConfiguration('joint_sequence'),
                    'hip_angle_limit_deg': hip_angle_limit_deg,
                    'loop': ParameterValue(
                        LaunchConfiguration('joint_sequence_loop'), value_type=bool
                    ),
                }],
            )
        )

    if _imu_enabled(context):
        actions.append(
            Node(
                package='imu_interface',
                executable='imu_accel',
                name='IMU_Accel',
                output='screen',
                parameters=[{
                    'rate_hz': 500.0,
                    'topic': 'IMU_Acceleration',
                    'frame_id': 'imu_link',
                    'serial_port': '',
                    'accel_range_g': 16,
                }],
            )
        )

    if _logging_enabled(context):
        base = LaunchConfiguration('bag_output_uri').perform(context)
        storage = LaunchConfiguration('bag_storage_id').perform(context)
        bag_uri = _next_bag_output_uri(base, bag_dir)
        bag_root = str(Path(bag_dir).expanduser().resolve())
        bag_topics = list(MOTOR_STATE_TOPICS)
        bag_topics.extend(MOTOR_COMMAND_TOPICS)
        bag_topics.extend(JOINT_CURPOS_TOPICS)
        if _imu_enabled(context):
            bag_topics.append(IMU_ACCEL_TOPIC)
        print(
            f'[boom_stack] enable_logging: recording to {bag_root}/{bag_uri} '
            f'({bag_uri}_0.mcap inside bag directory when using mcap)'
        )
        actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '-o',
                    bag_uri,
                    '-s',
                    storage,
                    *bag_topics,
                ],
                cwd=bag_root,
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_can_gateway',
            default_value='true',
            description='If true, one can_gateway_node owns CAN; if false, per-motor motor_node_continuous.',
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN interface name for gateway and legacy motor nodes.',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='0',
            description='Joystick device index for joy_node.',
        ),
        DeclareLaunchArgument(
            'omega_max',
            default_value='auto',
            description=(
                'Default joint_translator omega_max for namespaces not listed in '
                'namespace_omega_max (auto = motor profile rad/s cap).'
            ),
        ),
        DeclareLaunchArgument(
            'namespace_omega_max',
            default_value=DEFAULT_NAMESPACE_OMEGA_MAX,
            description='Per-namespace omega_max for joint_translator (ns:auto|rad_per_s,...).',
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='50.0',
            description='boom_joystick_control publish rate (Hz).',
        ),
        DeclareLaunchArgument(
            'test_kp',
            default_value='5.0',
            description=(
                'Knee MIT Kp when Xbox X toggles test mode '
                '(standard is joint_translator mit_kp at startup).'
            ),
        ),
        DeclareLaunchArgument(
            'hip_angle_limit_deg',
            default_value='90.0',
            description='Hip joint limit (deg) for teleop and hip joint_translator_node.',
        ),
        DeclareLaunchArgument(
            'motor_error_tolerance',
            default_value='0.001',
            description='Motor-space goal/hold tolerance (rad) for joint_translator_node.',
        ),
        DeclareLaunchArgument(
            'gateway_loop_rate_hz',
            default_value='200.0',
            description='can_gateway_node service loop rate (Hz).',
        ),
        DeclareLaunchArgument(
            'gateway_startup_stagger_ms',
            default_value='200',
            description='Delay between each drive enable sequence in can_gateway_node (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_enable_settle_ms',
            default_value='100',
            description='Post-enable delay for hip/wheel drives (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_ak80_enable_settle_ms',
            default_value='250',
            description='Post-enable delay for knee AK80-64 (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_startup_origin_poll_ms',
            default_value='100',
            description='Feedback poll after set-origin during startup (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_bus_warmup_ms',
            default_value='100',
            description='Delay after CAN bind before first enable (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_alive_check_period_ms',
            default_value=DEFAULT_GATEWAY_ALIVE_CHECK_PERIOD_MS,
            description='How often can_gateway_node checks that every drive has fresh MIT feedback (ms).',
        ),
        DeclareLaunchArgument(
            'gateway_reconnect_cooldown_ms',
            default_value=DEFAULT_GATEWAY_RECONNECT_COOLDOWN_MS,
            description='Minimum time between motor reconnect attempts (ms).',
        ),
        DeclareLaunchArgument(
            'motor_tx_rate_hz',
            default_value='200.0',
            description='Legacy motor_node_continuous TX rate when use_can_gateway:=false.',
        ),
        DeclareLaunchArgument(
            'motor_feedback_timeout_ms',
            default_value='250',
            description='No fresh feedback for this long (ms) triggers comm fault hold.',
        ),
        DeclareLaunchArgument(
            'motor_feedback_poll_ms',
            default_value='5',
            description='Blocking RX poll after each gateway loop (ms).',
        ),
        DeclareLaunchArgument(
            'motor_startup_stagger_ms',
            default_value='800',
            description='Legacy per-motor startup delay when use_can_gateway:=false.',
        ),
        DeclareLaunchArgument(
            'namespaces',
            default_value=DEFAULT_NAMESPACES,
            description='Comma-separated namespaces for boom_joystick_control (gateway list is always BOOM_MOTOR_STACKS).',
        ),
        DeclareLaunchArgument(
            'namespace_gear_ratios',
            default_value=DEFAULT_NAMESPACE_GEAR_RATIOS,
            description='Per-namespace gear ratios for teleop scaling (ns:ratio,...).',
        ),
        DeclareLaunchArgument(
            'enable_joint_sequence',
            default_value='true',
            description=(
                'If true, run joint_position_sequence_node (Start button runs YAML waypoints).'
            ),
        ),
        DeclareLaunchArgument(
            'sequence_file',
            default_value='',
            description='Waypoint YAML for joint sequence; empty uses installed preset.',
        ),
        DeclareLaunchArgument(
            'joint_sequence',
            default_value='KneeTumble',
            description='Preset name in sequence_file when using presets mapping.',
        ),
        DeclareLaunchArgument(
            'joint_sequence_loop',
            default_value='false',
            description='Repeat joint sequence waypoints after the last step.',
        ),
        DeclareLaunchArgument(
            'enable_IMU',
            default_value='false',
            description=(
                'If true, launch imu_accel (IMU_Accel). When enable_logging is also true, '
                'record /IMU_Acceleration in the bag.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='false',
            description=(
                'If true, run ros2 bag record on all stack motor_state, '
                'motor_command, and joint_curpos topics. '
                'Also records /IMU_Acceleration when enable_IMU is true.'
            ),
        ),
        DeclareLaunchArgument(
            'bag_output_uri',
            default_value='boom_stack_bag',
            description='Base name for bags; auto-increments when prior bags exist.',
        ),
        DeclareLaunchArgument(
            'bag_output_dir',
            default_value='.',
            description='Directory for bag folders and .mcap files (default: launch cwd).',
        ),
        DeclareLaunchArgument(
            'bag_storage_id',
            default_value='mcap',
            description='Rosbag storage plugin (e.g. mcap, sqlite3) when enable_logging is true.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
