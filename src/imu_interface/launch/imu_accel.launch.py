# Launch Yost Labs TSS-DL3 corrected acceleration publisher.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    return [
        Node(
            package='imu_interface',
            executable='imu_accel',
            name='IMU_Accel',
            output='screen',
            parameters=[{
                'rate_hz': float(LaunchConfiguration('rate_hz').perform(context)),
                'topic': LaunchConfiguration('topic').perform(context),
                'frame_id': LaunchConfiguration('frame_id').perform(context),
                'serial_port': LaunchConfiguration('serial_port').perform(context),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rate_hz',
            default_value='100.0',
            description='IMU recording / stream frequency in Hz (max 2000).',
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='IMU_Acceleration',
            description='Topic for imu_interface/ImuAcceleration messages.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='TF frame_id stamped on each message.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='USB serial device path. Empty string auto-detects a Yost TSS v3 sensor.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
