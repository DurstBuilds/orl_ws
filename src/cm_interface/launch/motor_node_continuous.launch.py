from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motor_model_arg = DeclareLaunchArgument(
        'motor_model',
        default_value='ak70_10',
        description='Motor MIT profile: ak70_10 | ak10_9 | ak80_64',
    )

    can_id_arg = DeclareLaunchArgument(
        'can_id',
        default_value='0',
        description='CAN drive ID for MIT commands and feedback.',
    )

    motor_node_continuous = Node(
        package='cm_interface',
        executable='motor_node_continuous',
        name='motor_node_continuous',
        output='screen',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
        }],
    )

    return LaunchDescription([
        motor_model_arg,
        can_id_arg,
        motor_node_continuous,
    ])
