from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_imu',
            executable='imu_sensor_node',
            name='imu_sensor_node',
            output='screen',
            parameters=[{
                'spi_bus':    0,
                'spi_device': 0,
                'frame_id':   'imu_link',
                'rate_hz':    100.0,
            }],
        ),
    ])
