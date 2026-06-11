# Standalone CAN gateway (boom drive set). Pair with namespaced translator stacks or teleop.

import sys
from pathlib import Path

_launch_dir = Path(__file__).resolve().parent
if str(_launch_dir) not in sys.path:
    sys.path.insert(0, str(_launch_dir))

from boom_motor_config import (  # noqa: E402
    DEFAULT_CAN_IDS,
    DEFAULT_GATEWAY_AK80_ENABLE_SETTLE_MS,
    DEFAULT_GATEWAY_BUS_WARMUP_MS,
    DEFAULT_GATEWAY_ENABLE_SETTLE_MS,
    DEFAULT_GATEWAY_LOOP_RATE_HZ,
    DEFAULT_GATEWAY_STARTUP_ORIGIN_POLL_MS,
    DEFAULT_GATEWAY_STARTUP_STAGGER_MS,
    DEFAULT_MOTOR_FEEDBACK_POLL_MS,
    DEFAULT_MOTOR_FEEDBACK_TIMEOUT_MS,
    DEFAULT_MOTOR_MODELS,
    DEFAULT_NAMESPACES,
)

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
            package='cm_interface',
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
                'feedback_poll_ms': int(
                    LaunchConfiguration('feedback_poll_ms').perform(context)
                ),
                'startup_stagger_ms': int(
                    LaunchConfiguration('startup_stagger_ms').perform(context)
                ),
                'enable_settle_ms': int(
                    LaunchConfiguration('enable_settle_ms').perform(context)
                ),
                'ak80_enable_settle_ms': int(
                    LaunchConfiguration('ak80_enable_settle_ms').perform(context)
                ),
                'startup_origin_poll_ms': int(
                    LaunchConfiguration('startup_origin_poll_ms').perform(context)
                ),
                'bus_warmup_ms': int(
                    LaunchConfiguration('bus_warmup_ms').perform(context)
                ),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('loop_rate_hz', default_value=DEFAULT_GATEWAY_LOOP_RATE_HZ),
        DeclareLaunchArgument(
            'feedback_timeout_ms', default_value=DEFAULT_MOTOR_FEEDBACK_TIMEOUT_MS
        ),
        DeclareLaunchArgument('feedback_poll_ms', default_value=DEFAULT_MOTOR_FEEDBACK_POLL_MS),
        DeclareLaunchArgument(
            'startup_stagger_ms', default_value=DEFAULT_GATEWAY_STARTUP_STAGGER_MS
        ),
        DeclareLaunchArgument('enable_settle_ms', default_value=DEFAULT_GATEWAY_ENABLE_SETTLE_MS),
        DeclareLaunchArgument(
            'ak80_enable_settle_ms', default_value=DEFAULT_GATEWAY_AK80_ENABLE_SETTLE_MS
        ),
        DeclareLaunchArgument(
            'startup_origin_poll_ms', default_value=DEFAULT_GATEWAY_STARTUP_ORIGIN_POLL_MS
        ),
        DeclareLaunchArgument('bus_warmup_ms', default_value=DEFAULT_GATEWAY_BUS_WARMUP_MS),
        DeclareLaunchArgument('namespaces', default_value=DEFAULT_NAMESPACES),
        DeclareLaunchArgument('motor_models', default_value=DEFAULT_MOTOR_MODELS),
        DeclareLaunchArgument('can_ids', default_value=DEFAULT_CAN_IDS),
        OpaqueFunction(function=_launch_setup),
    ])
