from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='10.0',
        description='Motor-to-joint reduction (motor_rad / joint_rad).',
    )

    mit_kp_arg = DeclareLaunchArgument(
        'mit_kp',
        default_value='4.0',
        description='MIT Kp sent on motor_command (joint_translator_node).',
    )

    mit_kd_arg = DeclareLaunchArgument(
        'mit_kd',
        default_value='0.02',
        description='MIT Kd sent on motor_command (joint_translator_node).',
    )

    joint_translator_node = Node(
        package='cm_interface',
        executable='joint_translator_node',
        name='joint_translator_node',
        parameters=[{
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'mit_kp': LaunchConfiguration('mit_kp'),
            'mit_kd': LaunchConfiguration('mit_kd'),
        }],
    )

    return LaunchDescription([
        gear_ratio_arg,
        mit_kp_arg,
        mit_kd_arg,
        joint_translator_node,
    ])
