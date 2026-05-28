from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='ROS namespace (e.g. motor_a). Empty = no namespace.',
    )

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
    max_torque_arg = DeclareLaunchArgument(
        'max_torque',
        default_value='10.0',
        description='Max modeled torque (Nm) used for deltaP clamp (|kp*deltaP| <= max_torque).',
    )

    motor_node_continuous = Node(
        package='cm_interface',
        executable='motor_node_continuous',
        name='motor_node_continuous',
        output='screen',
        parameters=[{
            'motor_model': LaunchConfiguration('motor_model'),
            'can_id': ParameterValue(LaunchConfiguration('can_id'), value_type=int),
            'max_torque': ParameterValue(LaunchConfiguration('max_torque'), value_type=float),
        }],
    )

    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        motor_node_continuous,
    ])

    return LaunchDescription([
        ns_arg,
        motor_model_arg,
        can_id_arg,
        max_torque_arg,
        namespaced_group,
    ])
