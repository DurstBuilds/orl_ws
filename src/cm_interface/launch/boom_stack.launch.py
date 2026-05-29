# Full boom stack: four namespaced motor pipelines + one joy_node + one boom_joystick_control.
# Equivalent to four boom_teleop motor launches with a single shared teleop node.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# Defaults match:
#   ros2 launch cm_interface boom_teleop.launch.py ns:=... (per row)
BOOM_MOTOR_STACKS = (
    {
        'ns': 'knee_motor',
        'gear_ratio': 1.6,
        'motor_model': 'ak80_64',
        'can_id': 4,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'hip_motor',
        'gear_ratio': 30.0,
        'motor_model': 'ak70_10',
        'can_id': 0, # Revert to 3 for testing
        'joint_angle_limit_deg': 45.0,
    },
     {
        'ns': 'wheel_motor1',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 1,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'wheel_motor2',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 2,
        'joint_angle_limit_deg': 0.0,
    },
)

DEFAULT_NAMESPACES = ','.join(stack['ns'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_NAMESPACE_GEAR_RATIOS = ','.join(
    f"{stack['ns']}:{stack['gear_ratio']}" for stack in BOOM_MOTOR_STACKS
)
MOTOR_STATE_TOPICS = [f'/{stack["ns"]}/motor_state' for stack in BOOM_MOTOR_STACKS]


def _motor_stack_group(stack: dict) -> GroupAction:
    ns = stack['ns']
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='cm_interface',
            executable='motor_node_continuous',
            name='motor_node_continuous',
            parameters=[{
                'motor_model': stack['motor_model'],
                'can_id': stack['can_id'],
                'max_torque': ParameterValue(
                    LaunchConfiguration('max_torque'), value_type=float
                ),
            }],
        ),
        Node(
            package='cm_interface',
            executable='motor_unwrapper_node',
            name='motor_unwrapper_node',
        ),
        Node(
            package='cm_interface',
            executable='joint_translator_node',
            name='joint_translator_node',
            parameters=[{
                'motor_model': stack['motor_model'],
                'gear_ratio': stack['gear_ratio'],
                'omega_max': ParameterValue(LaunchConfiguration('omega_max'), value_type=str),
                'joint_angle_limit_deg': stack['joint_angle_limit_deg'],
                'motor_error_tolerance': ParameterValue(
                    LaunchConfiguration('motor_error_tolerance'), value_type=float
                ),
            }],
        ),
    ])


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device index for joy_node.',
    )
    max_torque_arg = DeclareLaunchArgument(
        'max_torque',
        default_value='10.0',
        description='Max modeled torque (Nm) for motor_node_continuous.',
    )
    omega_max_arg = DeclareLaunchArgument(
        'omega_max',
        default_value='auto',
        description='joint_translator omega_max (auto uses motor profile).',
    )
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='50.0',
        description='boom_joystick_control publish rate (Hz).',
    )
    hip_angle_limit_deg_arg = DeclareLaunchArgument(
        'hip_angle_limit_deg',
        default_value='45.0',
        description='Hip joint_despos clamp in boom_joystick_control (deg).',
    )
    motor_error_tolerance_arg = DeclareLaunchArgument(
        'motor_error_tolerance',
        default_value='0.001',
        description='Motor-space goal/hold tolerance (rad) for joint_translator_node.',
    )
    namespaces_arg = DeclareLaunchArgument(
        'namespaces',
        default_value=DEFAULT_NAMESPACES,
        description='Comma-separated namespaces for boom_joystick_control.',
    )
    namespace_gear_ratios_arg = DeclareLaunchArgument(
        'namespace_gear_ratios',
        default_value=DEFAULT_NAMESPACE_GEAR_RATIOS,
        description='Per-namespace gear ratios for teleop scaling (ns:ratio,...).',
    )
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='false',
        description='If true, run ros2 bag record on all stack motor_state topics.',
    )
    bag_output_uri_arg = DeclareLaunchArgument(
        'bag_output_uri',
        default_value='boom_stack_bag',
        description='Output URI for rosbag when enable_logging is true.',
    )
    bag_storage_id_arg = DeclareLaunchArgument(
        'bag_storage_id',
        default_value='mcap',
        description='Rosbag storage plugin (e.g. mcap, sqlite3) when enable_logging is true.',
    )

    bag_record = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_logging'), "'.lower() in ('true', '1', 'yes')",
            ])
        ),
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            LaunchConfiguration('bag_output_uri'),
            '-s',
            LaunchConfiguration('bag_storage_id'),
            *MOTOR_STATE_TOPICS,
        ],
        output='screen',
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

    boom_joystick_control_node = Node(
        package='cm_interface',
        executable='boom_joystick_control_node',
        name='boom_joystick_control_node',
        parameters=[{
            'namespaces': LaunchConfiguration('namespaces'),
            'namespace_gear_ratios': LaunchConfiguration('namespace_gear_ratios'),
            'publish_hz': ParameterValue(LaunchConfiguration('publish_hz'), value_type=float),
            'hip_angle_limit_deg': ParameterValue(
                LaunchConfiguration('hip_angle_limit_deg'), value_type=float
            ),
        }],
    )

    motor_groups = [_motor_stack_group(stack) for stack in BOOM_MOTOR_STACKS]

    return LaunchDescription([
        joy_dev_arg,
        max_torque_arg,
        omega_max_arg,
        publish_hz_arg,
        hip_angle_limit_deg_arg,
        motor_error_tolerance_arg,
        namespaces_arg,
        namespace_gear_ratios_arg,
        enable_logging_arg,
        bag_output_uri_arg,
        bag_storage_id_arg,
        *motor_groups,
        joy_node,
        boom_joystick_control_node,
        bag_record,
    ])
