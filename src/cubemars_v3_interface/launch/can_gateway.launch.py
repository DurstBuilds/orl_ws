# V3 firmware CAN gateway for manual MIT control.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    # perform() + str() so CLI args like can_ids:=1 are not sent as integer parameters.
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
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('loop_rate_hz', default_value='200.0'),
        DeclareLaunchArgument('feedback_timeout_ms', default_value='250'),
        DeclareLaunchArgument('bus_warmup_ms', default_value='100'),
        DeclareLaunchArgument('namespaces', default_value='motor'),
        DeclareLaunchArgument('motor_models', default_value='ak60_6'),
        DeclareLaunchArgument('can_ids', default_value='1'),
        OpaqueFunction(function=_launch_setup),
    ])
