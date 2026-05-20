from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        description='Motor-to-joint reduction (motor_rad / joint_rad).',
    )

    joint_translator_node = Node(
        package='cm_interface',
        executable='joint_translator_node',
        name='joint_translator_node',
        parameters=[{
            'gear_ratio': LaunchConfiguration('gear_ratio'),
        }],
    )

    return LaunchDescription([
        gear_ratio_arg,
        joint_translator_node,
    ])
