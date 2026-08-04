"""Launch the V3 MIT CAN gateway with CSV-safe parameter coercion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    # perform() + str() so CLI args like can_ids:=1 are not sent as integer parameters.
    # The node accepts either type, but string keeps multi-drive CSV lists intact.
    namespaces = str(LaunchConfiguration('namespaces').perform(context)).strip()
    motor_models = str(LaunchConfiguration('motor_models').perform(context)).strip()
    can_ids = str(LaunchConfiguration('can_ids').perform(context)).strip()

    return [
        Node(
            package='cubemars_v3_interface',
            executable='can_gateway_node',
            name='can_gateway_node',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface').perform(context),
                'namespaces': namespaces,
                'motor_models': motor_models,
                'can_ids': can_ids,
                'loop_rate_hz': float(LaunchConfiguration('loop_rate_hz').perform(context)),
                'feedback_timeout_ms': int(
                    LaunchConfiguration('feedback_timeout_ms').perform(context)
                ),
                'bus_warmup_ms': int(LaunchConfiguration('bus_warmup_ms').perform(context)),
                'log_unmatched_frames': LaunchConfiguration(
                    'log_unmatched_frames'
                ).perform(context).strip().lower() in ('true', '1', 'yes'),
            }],
        ),
    ]


def generate_launch_description():
    """Declare gateway args and build the node via OpaqueFunction for typed params."""
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'loop_rate_hz',
            default_value='100.0',
            description='MIT hold refresh and comm-fault watchdog rate (Hz).',
        ),
        DeclareLaunchArgument('feedback_timeout_ms', default_value='250'),
        DeclareLaunchArgument('bus_warmup_ms', default_value='100'),
        DeclareLaunchArgument('namespaces', default_value='motor'),
        DeclareLaunchArgument('motor_models', default_value='ak60_6'),
        DeclareLaunchArgument('can_ids', default_value='1'),
        DeclareLaunchArgument(
            'log_unmatched_frames',
            default_value='false',
            description='Throttle-log extended CAN frames that are not periodic feedback.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
