from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device index for joy_node.',
    )

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

    joystick_control_node = Node(
        package='cm_interface',
        executable='joystick_control_node',
        name='joystick_control_node',
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        joystick_control_node,
    ])
