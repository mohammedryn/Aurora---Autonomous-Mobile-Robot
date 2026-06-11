from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')
    foxglove = LaunchConfiguration('foxglove', default='false')
    map_save_path = LaunchConfiguration(
        'map_save_path', default='~/AMR/maps/explore_map'
    )

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
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='Enable Foxglove bridge on port 8765 (costs ~15% CPU on Pi 5).'
        ),
        DeclareLaunchArgument(
            'map_save_path', default_value='~/AMR/maps/explore_map',
            description='Path prefix for saved map (.pgm/.yaml). Expanded by home_manager.'
        ),

        # ── Hardware ──────────────────────────────────────────────────────────
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
            # Relay wheel odom → /odom/wheel for EKF
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
            ),
            # Twist→TwistStamped relay: /cmd_vel_safe → /mecanum_drive_controller/reference
            # (topic_tools relay fails due to QoS mismatch with Jazzy chainable controller)
            Node(
                package='amr_imu',
                executable='cmd_vel_safe_relay',
                name='cmd_vel_safe_relay',
                output='screen',
            ),
        ]),

        # ── IMU ───────────────────────────────────────────────────────────────
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

        # ── Sensor Fusion (imu_filter_madgwick + robot_localization EKF → /odom) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_sensor_fusion'),
                '/launch/sensor_fusion.launch.py'
            ])
        ),

        # ── LiDAR ─────────────────────────────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':      '/dev/lidar',
                'serial_baudrate':  460800,
                'frame_id':         'base_laser',
                'angle_compensate': True,
            }],
        ),

        # ── SLAM (slam_toolbox online_async → /map + map→odom TF) ─────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr_slam'),
                '/launch/slam.launch.py'
            ])
        ),

        # ── Nav2 (SmacPlanner2D + MPPI DiffDrive + collision_monitor) ─────────
        # Delayed: SLAM + hardware + sensor fusion are still finishing their own
        # startup CPU spike around T+10s on the Pi 5. Bringing up the entire Nav2
        # stack (controller_server, both costmaps, planner, bt_navigator,
        # behavior_server, collision_monitor) at the same moment stacks two CPU/
        # current spikes on top of each other -- this is the moment the Pi has
        # browned out and rebooted (Bug B5 recurrence under heavier load).
        TimerAction(period=15.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('amr_nav'),
                    '/launch/nav2.launch.py'
                ])
            ),
            # ── Home Manager (records start pose, saves map on 'stop' command) ──
            Node(
                package='amr_home_manager',
                executable='home_manager',
                name='amr_home_manager',
                output='screen',
                parameters=[{'map_save_path': map_save_path}],
            ),
        ]),

        # ── Frontier Explorer (explore_lite — auto-starts on Nav2 ready) ──────
        # Further delayed past Nav2's own activation spike (T+15s + ~5-7s for
        # all lifecycle nodes to configure/activate).
        TimerAction(period=23.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('amr_explore'),
                    '/launch/explore.launch.py'
                ])
            ),
        ]),

        # ── Foxglove bridge (optional — disable during long unmanned runs) ─────
        TimerAction(period=23.0, actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                parameters=[{'port': 8765}],
                condition=IfCondition(foxglove),
            ),
        ]),

        # ── Operator instructions after stack is up ────────────────────────────
        TimerAction(period=28.0, actions=[
            LogInfo(msg=(
                '\n\n'
                '══════════════════════════════════════════════════════\n'
                '  explore_map READY — robot is exploring autonomously\n'
                '══════════════════════════════════════════════════════\n'
                '  Monitor:  ros2 topic hz /cmd_vel\n'
                '            ros2 topic echo /map_metadata --once\n'
                '  Stop & save map:\n'
                '    ros2 topic pub /amr/command std_msgs/msg/String \\\n'
                '      "data: \'stop\'" --once\n'
                '  Map will be saved to: ~/AMR/maps/explore_map.pgm/.yaml\n'
                '  Return home after stopping:\n'
                '    ros2 topic pub /amr/command std_msgs/msg/String \\\n'
                '      "data: \'go_home\'" --once\n'
                '══════════════════════════════════════════════════════\n'
            )),
        ]),
    ])
