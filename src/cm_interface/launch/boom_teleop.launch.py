from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

import sys
from pathlib import Path

_launch_dir = Path(__file__).resolve().parent
if str(_launch_dir) not in sys.path:
    sys.path.insert(0, str(_launch_dir))

from boom_motor_config import (  # noqa: E402
    DEFAULT_MOTOR_ERROR_TOLERANCE,
    DEFAULT_MOTOR_FEEDBACK_POLL_MS,
    DEFAULT_MOTOR_FEEDBACK_TIMEOUT_MS,
    origin_jump_threshold_for_motor_model,
)


def _joint_angle_limit_for_ns(context, *, launch_config_name: str) -> float:
    limit_deg = float(LaunchConfiguration(launch_config_name).perform(context))
    ns = LaunchConfiguration('ns').perform(context).lower()
    if 'hip' not in ns:
        return 0.0
    return limit_deg


def _launch_setup(context, *args, **kwargs):
    joint_angle_limit_deg = _joint_angle_limit_for_ns(
        context, launch_config_name='joint_angle_limit_deg'
    )

    motor_node_continuous = Node(
        package='cm_interface',
        executable='motor_node_continuous',
        name='motor_node_continuous',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
            'feedback_timeout_ms': ParameterValue(
                LaunchConfiguration('feedback_timeout_ms'), value_type=int
            ),
            'feedback_poll_ms': ParameterValue(
                LaunchConfiguration('feedback_poll_ms'), value_type=int
            ),
        }],
    )

    motor_model = LaunchConfiguration('motor_model').perform(context).strip()
    origin_jump_threshold = origin_jump_threshold_for_motor_model(motor_model)

    motor_unwrapper_node = Node(
        package='cm_interface',
        executable='motor_unwrapper_node',
        name='motor_unwrapper_node',
        parameters=[{
            'origin_jump_threshold': origin_jump_threshold,
        }],
    )

    joint_translator_node = Node(
        package='cm_interface',
        executable='joint_translator_node',
        name='joint_translator_node',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'omega_max': ParameterValue(LaunchConfiguration('omega_max'), value_type=str),
            'joint_angle_limit_deg': joint_angle_limit_deg,
            'motor_error_tolerance': ParameterValue(
                LaunchConfiguration('motor_error_tolerance'), value_type=float
            ),
        }],
    )

    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        motor_node_continuous,
        motor_unwrapper_node,
        joint_translator_node,
    ])

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
            'namespaces': LaunchConfiguration('ns'),
            'publish_hz': ParameterValue(LaunchConfiguration('publish_hz'), value_type=float),
            'gear_ratio': ParameterValue(LaunchConfiguration('gear_ratio'), value_type=float),
            'hip_angle_limit_deg': ParameterValue(
                LaunchConfiguration('joint_angle_limit_deg'), value_type=float
            ),
        }],
    )

    return [
        namespaced_group,
        joy_node,
        boom_joystick_control_node,
    ]


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='ROS namespace for motor nodes (e.g. hip_motor). Empty = no namespace. Include hip, knee, or wheel',
    )

    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='Motor-to-joint reduction (motor_rad / joint_rad) for joint_translator_node.',
    )

    motor_model_arg = DeclareLaunchArgument(
        'motor_model',
        default_value='ak70_10',
        description='Motor MIT profile: ak70_10 | ak10_9 | ak80_64',
    )

    can_id_arg = DeclareLaunchArgument(
        'can_id',
        default_value='0',
        description='CAN drive ID for motor_node_continuous (MIT command and feedback).',
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device index for joy_node.',
    )
    omega_max_arg = DeclareLaunchArgument(
        'omega_max',
        default_value='auto',
        description='Optional max motor speed (rad/s) for joint_translator_node; auto uses motor profile.',
    )
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='50.0',
        description='boom_joystick_control publish rate (Hz).',
    )
    joint_angle_limit_deg_arg = DeclareLaunchArgument(
        'joint_angle_limit_deg',
        default_value='90.0',
        description='Hip only: clamp joint_despos in joint_translator (deg). Non-hip namespaces use 0.',
    )
    motor_error_tolerance_arg = DeclareLaunchArgument(
        'motor_error_tolerance',
        default_value=DEFAULT_MOTOR_ERROR_TOLERANCE,
        description='Motor-space goal/hold tolerance (rad) for joint_translator_node.',
    )
    feedback_timeout_ms_arg = DeclareLaunchArgument(
        'feedback_timeout_ms',
        default_value=DEFAULT_MOTOR_FEEDBACK_TIMEOUT_MS,
        description='No fresh feedback for this long (ms) triggers comm fault hold.',
    )
    feedback_poll_ms_arg = DeclareLaunchArgument(
        'feedback_poll_ms',
        default_value=DEFAULT_MOTOR_FEEDBACK_POLL_MS,
        description='Blocking RX poll after each MIT TX (ms).',
    )

    return LaunchDescription([
        ns_arg,
        gear_ratio_arg,
        motor_model_arg,
        can_id_arg,
        joy_dev_arg,
        omega_max_arg,
        publish_hz_arg,
        joint_angle_limit_deg_arg,
        motor_error_tolerance_arg,
        feedback_timeout_ms_arg,
        feedback_poll_ms_arg,
        OpaqueFunction(function=_launch_setup),
    ])
