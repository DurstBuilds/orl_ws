# Full boom stack: single CAN gateway + four translator pipelines + teleop.
# Set use_can_gateway:=false to use legacy per-motor motor_node_continuous nodes.

import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

BOOM_MOTOR_STACKS = (
    {
        'ns': 'knee_motor',
        'gear_ratio': 1.6,
        'motor_model': 'ak80_64',
        'can_id': 4,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'hip_motor',
        'gear_ratio': 33.0,
        'motor_model': 'ak70_10',
        'can_id': 3,
        'joint_angle_limit_deg': 45.0,
    },
    {
        'ns': 'wheel_motor1',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 1,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'wheel_motor2',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 2,
        'joint_angle_limit_deg': 0.0,
    },
)

DEFAULT_NAMESPACES = ','.join(stack['ns'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_MOTOR_MODELS = ','.join(stack['motor_model'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_CAN_IDS = ','.join(str(stack['can_id']) for stack in BOOM_MOTOR_STACKS)
DEFAULT_NAMESPACE_GEAR_RATIOS = ','.join(
    f"{stack['ns']}:{stack['gear_ratio']}" for stack in BOOM_MOTOR_STACKS
)
MOTOR_STATE_TOPICS = [f'/{stack["ns"]}/motor_state' for stack in BOOM_MOTOR_STACKS]


def _logging_enabled(context) -> bool:
    value = LaunchConfiguration('enable_logging').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _use_can_gateway(context) -> bool:
    value = LaunchConfiguration('use_can_gateway').perform(context).strip().lower()
    return value in ('true', '1', 'yes')


def _collect_bag_indices(base: str, bag_dir: Path) -> list[int]:
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
    root = Path(bag_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    indices = _collect_bag_indices(base, root)
    next_index = max(indices) + 1 if indices else 0
    return f'{base}_{next_index}'


def _translator_stack_group(stack: dict) -> GroupAction:
    ns = stack['ns']
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='cm_interface',
            executable='motor_unwrapper_node',
            name='motor_unwrapper_node',
        ),
        Node(
            package='cm_interface',
            executable='joint_translator_node',
            name='joint_translator_node',
            parameters=[{
                'motor_model': stack['motor_model'],
                'gear_ratio': stack['gear_ratio'],
                'omega_max': ParameterValue(LaunchConfiguration('omega_max'), value_type=str),
                'joint_angle_limit_deg': stack['joint_angle_limit_deg'],
                'motor_error_tolerance': ParameterValue(
                    LaunchConfiguration('motor_error_tolerance'), value_type=float
                ),
            }],
        ),
    ])


def _legacy_motor_stack_group(stack: dict, motor_startup_delay_ms: int) -> GroupAction:
    ns = stack['ns']
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='cm_interface',
            executable='motor_node_continuous',
            name='motor_node_continuous',
            parameters=[{
                'motor_model': stack['motor_model'],
                'can_id': stack['can_id'],
                'max_torque': ParameterValue(
                    LaunchConfiguration('max_torque'), value_type=float
                ),
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
        ),
        Node(
            package='cm_interface',
            executable='joint_translator_node',
            name='joint_translator_node',
            parameters=[{
                'motor_model': stack['motor_model'],
                'gear_ratio': stack['gear_ratio'],
                'omega_max': ParameterValue(LaunchConfiguration('omega_max'), value_type=str),
                'joint_angle_limit_deg': stack['joint_angle_limit_deg'],
                'motor_error_tolerance': ParameterValue(
                    LaunchConfiguration('motor_error_tolerance'), value_type=float
                ),
            }],
        ),
    ])


def _launch_setup(context, *args, **kwargs):
    bag_dir = LaunchConfiguration('bag_output_dir').perform(context)

    actions = []

    if _use_can_gateway(context):
        actions.append(
            Node(
                package='cm_interface',
                executable='can_gateway_node',
                name='can_gateway_node',
                output='screen',
                parameters=[{
                    'can_interface': 'can0',
                    'namespaces': DEFAULT_NAMESPACES,
                    'motor_models': DEFAULT_MOTOR_MODELS,
                    'can_ids': DEFAULT_CAN_IDS,
                    'max_torque': ParameterValue(
                        LaunchConfiguration('max_torque'), value_type=float
                    ),
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
                }],
            )
        )
        actions.extend(_translator_stack_group(stack) for stack in BOOM_MOTOR_STACKS)
    else:
        stagger_ms = int(LaunchConfiguration('motor_startup_stagger_ms').perform(context))
        if stagger_ms < 0:
            stagger_ms = 0
        actions.extend(
            _legacy_motor_stack_group(stack, index * stagger_ms)
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
            'hip_angle_limit_deg': ParameterValue(
                LaunchConfiguration('hip_angle_limit_deg'), value_type=float
            ),
        }],
    )

    actions.extend([joy_node, boom_joystick_control_node])

    if _logging_enabled(context):
        base = LaunchConfiguration('bag_output_uri').perform(context)
        storage = LaunchConfiguration('bag_storage_id').perform(context)
        bag_uri = _next_bag_output_uri(base, bag_dir)
        bag_root = str(Path(bag_dir).expanduser().resolve())
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
                    *MOTOR_STATE_TOPICS,
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
            description='If true, one can_gateway_node owns can0; if false, four motor_node_continuous.',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='0',
            description='Joystick device index for joy_node.',
        ),
        DeclareLaunchArgument(
            'max_torque',
            default_value='10.0',
            description='Max modeled torque (Nm) for MIT drives.',
        ),
        DeclareLaunchArgument(
            'omega_max',
            default_value='auto',
            description='joint_translator omega_max (auto uses motor profile).',
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='50.0',
            description='boom_joystick_control publish rate (Hz).',
        ),
        DeclareLaunchArgument(
            'hip_angle_limit_deg',
            default_value='45.0',
            description='Hip joint_despos clamp in boom_joystick_control (deg).',
        ),
        DeclareLaunchArgument(
            'motor_error_tolerance',
            default_value='0.001',
            description='Motor-space goal/hold tolerance (rad) for joint_translator_node.',
        ),
        DeclareLaunchArgument(
            'gateway_loop_rate_hz',
            default_value='100.0',
            description='can_gateway_node service loop rate (Hz).',
        ),
        DeclareLaunchArgument(
            'gateway_startup_stagger_ms',
            default_value='200',
            description='Delay between each drive enable sequence in can_gateway_node (ms).',
        ),
        DeclareLaunchArgument(
            'motor_tx_rate_hz',
            default_value='100.0',
            description='Legacy motor_node_continuous TX rate when use_can_gateway:=false.',
        ),
        DeclareLaunchArgument(
            'motor_feedback_timeout_ms',
            default_value='250',
            description='No fresh feedback for this long (ms) triggers comm fault hold.',
        ),
        DeclareLaunchArgument(
            'motor_feedback_poll_ms',
            default_value='15',
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
            description='Comma-separated namespaces for boom_joystick_control.',
        ),
        DeclareLaunchArgument(
            'namespace_gear_ratios',
            default_value=DEFAULT_NAMESPACE_GEAR_RATIOS,
            description='Per-namespace gear ratios for teleop scaling (ns:ratio,...).',
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='false',
            description='If true, run ros2 bag record on all stack motor_state topics.',
        ),
        DeclareLaunchArgument(
            'bag_output_uri',
            default_value='boom_stack_bag',
            description=(
                'Base name for bags; auto-increments to boom_stack_bag_0, '
                'boom_stack_bag_1, ... when prior bags exist.'
            ),
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
