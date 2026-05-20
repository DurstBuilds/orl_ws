from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        description='Motor-to-joint reduction (motor_rad / joint_rad) for joint_translator_node.',
    )

    motor_node_continuous = Node(
        package='cm_interface',
        executable='motor_node_continuous',
        name='motor_node_continuous',
    )

    motor_unwrapper_node = Node(
        package='cm_interface',
        executable='motor_unwrapper_node',
        name='motor_unwrapper_node',
    )

    joint_translator_node = Node(
        package='cm_interface',
        executable='joint_translator_node',
        name='joint_translator_node',
        parameters=[{
            'gear_ratio': LaunchConfiguration('gear_ratio'),
        }],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    joystick_control_node = Node(
        package='cm_interface',
        executable='joystick_control_node',
        name='joystick_control_node',
    )

    return LaunchDescription([
        gear_ratio_arg,
        motor_node_continuous,
        motor_unwrapper_node,
        joint_translator_node,
    ])
