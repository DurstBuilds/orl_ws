# Launch RAIK060 multi-turn encoder SPI reader on Raspberry Pi SPI1 (GPIO16 CE2).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    return [
        Node(
            package='encoder_interface',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
            parameters=[{
                'spi_device': LaunchConfiguration('spi_device').perform(context),
                'spi_speed_hz': int(LaunchConfiguration('spi_speed_hz').perform(context)),
                'poll_rate_hz': float(LaunchConfiguration('poll_rate_hz').perform(context)),
                'frame_id': LaunchConfiguration('frame_id').perform(context),
                'cs_delay_us': int(LaunchConfiguration('cs_delay_us').perform(context)),
                'apply_zero_position_on_startup': LaunchConfiguration(
                    'apply_zero_position_on_startup'
                ).perform(context).strip().lower() in ('true', '1', 'yes'),
                'log_raw_frames': LaunchConfiguration('log_raw_frames').perform(context).strip().lower()
                in ('true', '1', 'yes'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spi_device', default_value='/dev/spidev1.2'),
        DeclareLaunchArgument('spi_speed_hz', default_value='500000'),
        DeclareLaunchArgument('poll_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('frame_id', default_value='encoder_link'),
        DeclareLaunchArgument('cs_delay_us', default_value='20'),
        DeclareLaunchArgument(
            'apply_zero_position_on_startup',
            default_value='true',
            description='Send SPI cmd 0x24 (Apply Zero position offset) once at node start.',
        ),
        DeclareLaunchArgument(
            'log_raw_frames',
            default_value='false',
            description='Log full 6-byte SPI RX when CRC fails (for bring-up).',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
