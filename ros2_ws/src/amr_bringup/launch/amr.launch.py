from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'
            ]),
            ' use_sim:=', use_sim
        ])
    }

    ctrl_params = PathJoinSubstitution([
        FindPackageShare('amr_bringup'), 'config', 'controllers.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),

        # Publishes robot_description and all static TF frames
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),

        # ros2_control node — loads amr_hardware/AMRHardwareInterface
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ctrl_params],
            output='screen',
        ),

        # Allow hardware interface 2s to open serial port before spawning controllers
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

        # Relay mecanum odometry to /odom/wheel for EKF
        TimerAction(period=4.0, actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
            ),
            # Convert /cmd_vel (Twist) → /mecanum_drive_controller/reference (TwistStamped)
            Node(
                package='amr_imu',
                executable='twist_to_reference',
                name='cmd_vel_to_reference',
                output='screen',
            ),
        ]),

        # IMU driver — reads ISM330DHCX via SPI0 CE0, publishes /imu/data_raw
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

        # Sensor fusion — imu_filter_madgwick + robot_localization EKF → /odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_sensor_fusion'),
                '/launch/sensor_fusion.launch.py'
            ])
        ),

        # LiDAR — Slamtec C1M1 R2 uses 460800 baud; scan_mode auto-selected
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/lidar',
                'serial_baudrate': 460800,
                'frame_id': 'base_laser',
                'angle_compensate': True,
            }],
        ),

        # SLAM — slam_toolbox online_async → /map + map→odom TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_slam'),
                '/launch/slam.launch.py'
            ])
        ),

        # Foxglove bridge — connect from Windows via ws://amr.local:8765
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{'port': 8765}],
        ),
    ])
