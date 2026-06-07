import os
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('amr_nav')
    bt_pkg = get_package_share_directory('nav2_bt_navigator')
    nav2_params_path = os.path.join(pkg, 'config', 'nav2_params.yaml')
    collision_params_path = os.path.join(pkg, 'config', 'collision_monitor.yaml')
    bt_xml_path = os.path.join(
        bt_pkg, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')

    # Inject BT XML path, then write to temp file so ROS2's ParameterFile
    # mechanism handles node-name scoping correctly.
    with open(nav2_params_path) as f:
        params = yaml.safe_load(f)
    # "" doesn't resolve to the built-in default in Jazzy — must be explicit
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml_path

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(params, tmp, default_flow_style=False)
    tmp.close()
    patched_params = tmp.name

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[patched_params],
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[patched_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[patched_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[patched_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 0.0,  # disable bond — Pi startup load causes false failures
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'collision_monitor',
            ],
        }],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_params_path],
    )

    return [
        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
        collision_monitor,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
