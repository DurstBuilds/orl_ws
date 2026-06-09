# Translator + unwrapper pipeline (pair with can_gateway or motor_node_continuous separately).

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

_launch_dir = Path(__file__).resolve().parent
if str(_launch_dir) not in sys.path:
    sys.path.insert(0, str(_launch_dir))

from boom_motor_config import (  # noqa: E402
    DEFAULT_MOTOR_ERROR_TOLERANCE,
    origin_jump_threshold_for_motor_model,
)


def _launch_setup(context, *args, **kwargs):
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
            'motor_error_tolerance': ParameterValue(
                LaunchConfiguration('motor_error_tolerance'), value_type=float
            ),
        }],
    )

    return [
        GroupAction([
            PushRosNamespace(LaunchConfiguration('ns')),
            motor_unwrapper_node,
            joint_translator_node,
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ns',
            default_value='',
            description='ROS namespace (e.g. motor_a). Empty = no namespace.',
        ),
        DeclareLaunchArgument(
            'motor_model',
            default_value='ak70_10',
            description='Motor profile (pd_kp, pd_kd, mit_kp, mit_kd): ak70_10 | ak10_9 | ak80_64',
        ),
        DeclareLaunchArgument(
            'gear_ratio',
            default_value='10.0',
            description='Motor-to-joint reduction (motor_rad / joint_rad).',
        ),
        DeclareLaunchArgument(
            'omega_max',
            default_value='auto',
            description='Optional max motor speed (rad/s); auto uses motor profile.',
        ),
        DeclareLaunchArgument(
            'motor_error_tolerance',
            default_value=DEFAULT_MOTOR_ERROR_TOLERANCE,
            description='Motor-space goal/hold tolerance (rad) for joint_translator_node.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
