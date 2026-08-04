from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Joystick topic for sequence start trigger.',
        ),
        DeclareLaunchArgument(
            'sequence_file',
            default_value='',
            description=(
                'Path to waypoint YAML. Empty uses installed '
                'config/joint_sequence_presets.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'joint_sequence',
            default_value='KneeTumble',
            description='Initial preset selection when sequence_file defines presets.',
        ),
        DeclareLaunchArgument(
            'hip_angle_limit_deg',
            default_value='90.0',
            description='Hip joint limit (deg) when clamping sequence targets.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Repeat waypoints after the last step.',
        ),
        Node(
            package='cm_interface',
            executable='joint_position_sequence_node',
            name='joint_position_sequence_node',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'sequence_file': LaunchConfiguration('sequence_file'),
                'joint_sequence': LaunchConfiguration('joint_sequence'),
                'hip_angle_limit_deg': ParameterValue(
                    LaunchConfiguration('hip_angle_limit_deg'), value_type=float
                ),
                'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
            }],
        ),
    ])
