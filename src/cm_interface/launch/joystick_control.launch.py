from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': 0,
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
        joy_node,
        joystick_control_node,
    ])
