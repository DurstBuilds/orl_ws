from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='ROS namespace (e.g. motor_a). Empty = no namespace.',
    )

    motor_model_arg = DeclareLaunchArgument(
        'motor_model',
        default_value='ak70_10',
        description='Motor profile (pd_kp, pd_kd, mit_kp, mit_kd): ak70_10 | ak10_9 | ak80_64',
    )

    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='10.0',
        description='Motor-to-joint reduction (motor_rad / joint_rad).',
    )
    omega_max_arg = DeclareLaunchArgument(
        'omega_max',
        default_value='auto',
        description='Optional max motor speed (rad/s); auto uses motor profile.',
    )

    joint_translator_node = Node(
        package='cm_interface',
        executable='joint_translator_node',
        name='joint_translator_node',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'omega_max': LaunchConfiguration('omega_max'),
        }],
    )

    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        joint_translator_node,
    ])

    return LaunchDescription([
        ns_arg,
        motor_model_arg,
        gear_ratio_arg,
        omega_max_arg,
        namespaced_group,
    ])
