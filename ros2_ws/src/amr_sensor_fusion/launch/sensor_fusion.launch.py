from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('amr_sensor_fusion')

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'imu_filter.yaml')],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data',     '/imu/data'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([imu_filter, ekf])
