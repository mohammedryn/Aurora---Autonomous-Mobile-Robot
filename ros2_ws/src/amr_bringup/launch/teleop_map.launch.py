"""
Teleop mapping bringup — drive the robot by keyboard and build a SLAM map.

Runs ONLY: robot_state_publisher, ros2_control (hardware), joint_state_broadcaster,
mecanum_drive_controller, odom relay, cmd_vel_safe_relay, IMU + Madgwick + EKF,
LiDAR, slam_toolbox, foxglove. NO Nav2, NO explore, NO collision_monitor — so the
robot only moves when YOU press keys, giving clean controlled motion for mapping.

Drive it from a separate terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/cmd_vel_safe
(teleop publishes Twist on /cmd_vel_safe → cmd_vel_safe_relay converts to the
mecanum controller's TwistStamped reference.)
"""
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
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster']),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['mecanum_drive_controller']),
        ]),

        TimerAction(period=4.0, actions=[
            # wheel odometry → /odom/wheel for the EKF
            Node(package='topic_tools', executable='relay', name='odom_relay',
                 arguments=['/mecanum_drive_controller/odometry', '/odom/wheel']),
            # /cmd_vel_safe (Twist, from teleop) → reference (TwistStamped)
            Node(package='amr_imu', executable='cmd_vel_safe_relay',
                 name='cmd_vel_safe_relay', output='screen'),
        ]),

        # IMU → /imu/data_raw
        Node(
            package='amr_imu', executable='imu_sensor_node', name='imu_sensor_node',
            output='screen',
            parameters=[{'spi_bus': 0, 'spi_device': 0,
                         'frame_id': 'imu_link', 'rate_hz': 100.0}],
        ),

        # Madgwick + EKF → /odom and odom→base_link TF (needed by SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_sensor_fusion'),
                '/launch/sensor_fusion.launch.py'
            ])
        ),

        # LiDAR
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            parameters=[{'serial_port': '/dev/lidar', 'serial_baudrate': 460800,
                         'frame_id': 'base_laser', 'angle_compensate': True}],
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_slam'),
                '/launch/slam.launch.py'
            ])
        ),

        # Foxglove (optional remote view)
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765}]),
    ])
