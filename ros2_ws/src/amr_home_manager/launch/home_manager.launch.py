from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_home_manager',
            executable='home_manager',
            name='amr_home_manager',
            output='screen',
        )
    ])
