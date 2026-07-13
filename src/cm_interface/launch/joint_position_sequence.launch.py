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
            'start_button_index',
            default_value='7',
            description='Joy button index to start/abort the sequence (default Start).',
        ),
        DeclareLaunchArgument(
            'back_button_index',
            default_value='0',
            description='Joy button index to set origin on all origin_namespaces.',
        ),
        DeclareLaunchArgument(
            'dpad_vertical_axis',
            default_value='0',
            description='Joy axis index for D-pad up/down preset scrolling.',
        ),
        DeclareLaunchArgument(
            'dpad_axis_threshold',
            default_value='0.5',
            description='Axis magnitude threshold for D-pad up/down detection.',
        ),
        DeclareLaunchArgument(
            'origin_namespaces',
            default_value='',
            description=(
                'Comma-separated namespaces for back-button origin reset. '
                'Empty uses union of all preset waypoint namespaces.'
            ),
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
                'start_button_index': ParameterValue(
                    LaunchConfiguration('start_button_index'), value_type=int
                ),
                'back_button_index': ParameterValue(
                    LaunchConfiguration('back_button_index'), value_type=int
                ),
                'dpad_vertical_axis': ParameterValue(
                    LaunchConfiguration('dpad_vertical_axis'), value_type=int
                ),
                'dpad_axis_threshold': ParameterValue(
                    LaunchConfiguration('dpad_axis_threshold'), value_type=float
                ),
                'origin_namespaces': LaunchConfiguration('origin_namespaces'),
                'hip_angle_limit_deg': ParameterValue(
                    LaunchConfiguration('hip_angle_limit_deg'), value_type=float
                ),
                'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
            }],
        ),
    ])
