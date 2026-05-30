"""
Minimal launch for hardware testing (no LiDAR, no foxglove).
Phase 4: includes IMU driver + sensor_fusion (madgwick + EKF → /odom).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'
            ]),
            ' use_sim:=false'
        ])
    }

    ctrl_params = PathJoinSubstitution([
        FindPackageShare('amr_bringup'), 'config', 'controllers.yaml'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ctrl_params],
            output='screen',
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
            ),
        ]),
        TimerAction(period=3.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_drive_controller'],
            ),
        ]),
        TimerAction(period=4.0, actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
            ),
        ]),

        # IMU driver (ISM330DHCX via SPI0 CE0)
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

        # Sensor fusion: madgwick → /imu/data, EKF → /odom (odom→base_link TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_sensor_fusion'),
                '/launch/sensor_fusion.launch.py'
            ])
        ),
    ])
