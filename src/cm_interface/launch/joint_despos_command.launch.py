from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='knee_motor',
        description='Motor namespace to command (e.g. knee_motor, hip_motor).',
    )

    joint_despos_command_node = Node(
        package='cm_interface',
        executable='joint_despos_command',
        name='joint_despos_command',
        emulate_tty=True,
        parameters=[{
            'namespace': LaunchConfiguration('ns'),
        }],
    )

    return LaunchDescription([
        ns_arg,
        joint_despos_command_node,
    ])
