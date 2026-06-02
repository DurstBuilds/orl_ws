# Standalone CAN gateway (boom drive set). Pair with namespaced translator stacks or teleop.

import sys
from pathlib import Path

_launch_dir = Path(__file__).resolve().parent
if str(_launch_dir) not in sys.path:
    sys.path.insert(0, str(_launch_dir))

from boom_motor_config import (  # noqa: E402
    DEFAULT_CAN_IDS,
    DEFAULT_MOTOR_MODELS,
    DEFAULT_NAMESPACES,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('max_torque', default_value='10.0'),
        DeclareLaunchArgument('loop_rate_hz', default_value='200.0'),
        DeclareLaunchArgument('feedback_timeout_ms', default_value='250'),
        DeclareLaunchArgument('feedback_poll_ms', default_value='5'),
        DeclareLaunchArgument('startup_stagger_ms', default_value='200'),
        DeclareLaunchArgument('namespaces', default_value=DEFAULT_NAMESPACES),
        DeclareLaunchArgument('motor_models', default_value=DEFAULT_MOTOR_MODELS),
        DeclareLaunchArgument('can_ids', default_value=DEFAULT_CAN_IDS),
        Node(
            package='cm_interface',
            executable='can_gateway_node',
            name='can_gateway_node',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'namespaces': LaunchConfiguration('namespaces'),
                'motor_models': LaunchConfiguration('motor_models'),
                'can_ids': LaunchConfiguration('can_ids'),
                'max_torque': ParameterValue(LaunchConfiguration('max_torque'), value_type=float),
                'loop_rate_hz': ParameterValue(LaunchConfiguration('loop_rate_hz'), value_type=float),
                'feedback_timeout_ms': ParameterValue(
                    LaunchConfiguration('feedback_timeout_ms'), value_type=int
                ),
                'feedback_poll_ms': ParameterValue(
                    LaunchConfiguration('feedback_poll_ms'), value_type=int
                ),
                'startup_stagger_ms': ParameterValue(
                    LaunchConfiguration('startup_stagger_ms'), value_type=int
                ),
            }],
        ),
    ])
