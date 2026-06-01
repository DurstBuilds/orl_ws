# Standalone CAN gateway (boom drive set). Pair with namespaced translator stacks or teleop.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration

# Boom stack defaults (must match boom_stack.launch.py BOOM_MOTOR_STACKS).
_NAMESPACES = 'knee_motor,hip_motor,wheel_motor1,wheel_motor2'
_MOTOR_MODELS = 'ak80_64,ak70_10,ak10_9,ak10_9'
_CAN_IDS = '4,3,1,2'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('max_torque', default_value='10.0'),
        DeclareLaunchArgument('loop_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('feedback_timeout_ms', default_value='250'),
        DeclareLaunchArgument('feedback_poll_ms', default_value='15'),
        DeclareLaunchArgument('startup_stagger_ms', default_value='200'),
        DeclareLaunchArgument('namespaces', default_value=_NAMESPACES),
        DeclareLaunchArgument('motor_models', default_value=_MOTOR_MODELS),
        DeclareLaunchArgument('can_ids', default_value=_CAN_IDS),
        Node(
            package='cm_interface',
            executable='can_gateway_node',
            name='can_gateway_node',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'namespaces': LaunchConfiguration('namespaces'),
                'motor_models': LaunchConfiguration('motor_models'),
                'can_ids': LaunchConfiguration('can_ids'),
                'max_torque': ParameterValue(LaunchConfiguration('max_torque'), value_type=float),
                'loop_rate_hz': ParameterValue(LaunchConfiguration('loop_rate_hz'), value_type=float),
                'feedback_timeout_ms': ParameterValue(
                    LaunchConfiguration('feedback_timeout_ms'), value_type=int
                ),
                'feedback_poll_ms': ParameterValue(
                    LaunchConfiguration('feedback_poll_ms'), value_type=int
                ),
                'startup_stagger_ms': ParameterValue(
                    LaunchConfiguration('startup_stagger_ms'), value_type=int
                ),
            }],
        ),
    ])
