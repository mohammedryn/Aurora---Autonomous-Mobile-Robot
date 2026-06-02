"""
Minimal hardware-only bringup for odometry calibration / hand-push testing.

Brings up ONLY: robot_state_publisher, ros2_control (hardware interface),
joint_state_broadcaster, mecanum_drive_controller, and the /odom/wheel relay.
NO nav2, NO explore, NO SLAM — the robot cannot drive itself, so it's safe
to push by hand and read /odom/wheel.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

        # Relay mecanum odometry to /odom/wheel
        TimerAction(period=4.0, actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='odom_relay',
                arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
            ),
        ]),
    ])
