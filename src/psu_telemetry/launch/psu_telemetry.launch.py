# Launch TTI QPX600DP voltage/current/power telemetry over USB serial.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    return [
        Node(
            package='psu_telemetry',
            executable='psu_telemetry_node',
            name='psu_telemetry_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port').perform(context),
                'baud_rate': int(LaunchConfiguration('baud_rate').perform(context)),
                'publish_rate_hz': float(LaunchConfiguration('publish_rate_hz').perform(context)),
                'output_index': int(LaunchConfiguration('output_index').perform(context)),
                'current_topic': LaunchConfiguration('current_topic').perform(context),
                'voltage_topic': LaunchConfiguration('voltage_topic').perform(context),
                'power_topic': LaunchConfiguration('power_topic').perform(context),
                'serial_timeout_s': float(LaunchConfiguration('serial_timeout_s').perform(context)),
                'identify_on_startup': LaunchConfiguration('identify_on_startup').perform(context).strip().lower()
                in ('true', '1', 'yes'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='9600'),
        DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('output_index', default_value='1'),
        DeclareLaunchArgument('current_topic', default_value='power_supply/current'),
        DeclareLaunchArgument('voltage_topic', default_value='power_supply/voltage'),
        DeclareLaunchArgument('power_topic', default_value='power_supply/power'),
        DeclareLaunchArgument('serial_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('identify_on_startup', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
