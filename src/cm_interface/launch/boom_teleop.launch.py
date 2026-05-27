from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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
    omega_max_arg = DeclareLaunchArgument(
        'omega_max',
        default_value='auto',
        description='Optional max motor speed (rad/s) for joint_translator_node; auto uses motor profile.',
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
            'omega_max': ParameterValue(LaunchConfiguration('omega_max'), value_type=str),
        }],
    )

    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        motor_node_continuous,
        motor_unwrapper_node,
        joint_translator_node,
    ])

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

    boom_joystick_control_node = Node(
        package='cm_interface',
        executable='boom_joystick_control_node',
        name='boom_joystick_control_node',
        parameters=[{
            'namespaces': LaunchConfiguration('ns'),
        }],
    )

    return LaunchDescription([
        ns_arg,
        gear_ratio_arg,
        motor_model_arg,
        can_id_arg,
        joy_dev_arg,
        omega_max_arg,
        namespaced_group,
        joy_node,
        boom_joystick_control_node,
    ])
