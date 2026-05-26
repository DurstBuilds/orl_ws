import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('cm_interface')

    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='ROS namespace for motor nodes (e.g. motor_a). Empty = no namespace.',
    )

    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='10.0',
        description='Motor-to-joint reduction (motor_rad / joint_rad) for joint_translator_node.',
    )

    motor_model_arg = DeclareLaunchArgument(
        'motor_model',
        default_value='ak70_10',
        description='Motor MIT profile: ak70_10 | ak10_9 | ak80_64',
    )

    can_id_arg = DeclareLaunchArgument(
        'can_id',
        default_value='0',
        description='CAN drive ID for motor_node_continuous (MIT command and feedback).',
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device index for joy_node.',
    )

    motor_node_continuous = Node(
        package='cm_interface',
        executable='motor_node_continuous',
        name='motor_node_continuous',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
        }],
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
            'motor_model': LaunchConfiguration('motor_model'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
        }],
    )

    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        motor_node_continuous,
        motor_unwrapper_node,
        joint_translator_node,
    ])

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick_teleop.launch.py'),
        ),
        launch_arguments={
            'joy_dev': LaunchConfiguration('joy_dev'),
            'namespaces': LaunchConfiguration('ns'),
        }.items(),
    )

    return LaunchDescription([
        ns_arg,
        gear_ratio_arg,
        motor_model_arg,
        can_id_arg,
        joy_dev_arg,
        namespaced_group,
        joystick_teleop,
    ])
