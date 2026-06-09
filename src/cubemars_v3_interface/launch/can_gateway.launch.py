# V3 firmware CAN gateway for manual MIT control.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('loop_rate_hz', default_value='200.0'),
        DeclareLaunchArgument('feedback_timeout_ms', default_value='250'),
        DeclareLaunchArgument('feedback_poll_ms', default_value='15'),
        DeclareLaunchArgument('bus_warmup_ms', default_value='100'),
        DeclareLaunchArgument('namespaces', default_value='motor'),
        DeclareLaunchArgument('motor_models', default_value='ak60_6'),
        DeclareLaunchArgument('can_ids', default_value='1'),
        Node(
            package='cubemars_v3_interface',
            executable='can_gateway_node',
            name='can_gateway_node',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'namespaces': LaunchConfiguration('namespaces'),
                'motor_models': LaunchConfiguration('motor_models'),
                'can_ids': LaunchConfiguration('can_ids'),
                'loop_rate_hz': ParameterValue(
                    LaunchConfiguration('loop_rate_hz'), value_type=float
                ),
                'feedback_timeout_ms': ParameterValue(
                    LaunchConfiguration('feedback_timeout_ms'), value_type=int
                ),
                'feedback_poll_ms': ParameterValue(
                    LaunchConfiguration('feedback_poll_ms'), value_type=int
                ),
                'bus_warmup_ms': ParameterValue(
                    LaunchConfiguration('bus_warmup_ms'), value_type=int
                ),
            }],
        ),
    ])
