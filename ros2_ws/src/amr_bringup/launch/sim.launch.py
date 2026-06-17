import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def _patch_yaml(path: str, extra: dict) -> str:
    """Read YAML, inject extra into every node's ros__parameters, return temp path."""
    with open(path) as f:
        data = yaml.safe_load(f)
    for _node, cfg in (data or {}).items():
        if isinstance(cfg, dict) and 'ros__parameters' in cfg:
            cfg['ros__parameters'].update(extra)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(data, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


SIM = {'use_sim_time': True}


def launch_setup(context, *args, **kwargs):
    bringup  = get_package_share_directory('amr_bringup')
    desc     = get_package_share_directory('amr_description')
    nav_pkg  = get_package_share_directory('amr_nav')
    slam_pkg = get_package_share_directory('amr_slam')
    fusion   = get_package_share_directory('amr_sensor_fusion')
    explore  = get_package_share_directory('amr_explore')
    bt_pkg   = get_package_share_directory('nav2_bt_navigator')

    world_file = os.path.join(desc, 'worlds', 'warehouse.sdf')

    import xacro
    robot_description_content = xacro.process_file(
        os.path.join(desc, 'urdf', 'amr.urdf.xacro'),
        mappings={'use_sim': 'true'}
    ).toxml()

    nav2_raw = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    with open(nav2_raw) as f:
        nav2_params = yaml.safe_load(f)
    bt_xml = os.path.join(bt_pkg, 'behavior_trees',
                          'navigate_to_pose_w_replanning_and_recovery.xml')
    nav2_params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    for cfg in nav2_params.values():
        if isinstance(cfg, dict) and 'ros__parameters' in cfg:
            cfg['ros__parameters']['use_sim_time'] = True
    # local_costmap and global_costmap are doubly-nested — patch use_sim_time separately
    for key in ('local_costmap', 'global_costmap'):
        nav2_params[key][key]['ros__parameters']['use_sim_time'] = True

    # Sim-specific nav2 overrides (not applied to real hardware):
    # • Progress checker: DiffDrive model rotates before translating; 0.5m/10s fires
    #   before the robot has even finished turning — loosen to 0.2m/30s.
    # • MPPI speeds: sim has perfect odometry/physics so no need for the real-hardware
    #   0.10 m/s conservatism; 0.35 m/s lets exploration finish in reasonable time.
    # • Inflation: real-hw 0.45m is tight for 1.5m warehouse aisles (0 margin);
    #   0.30m gives ~0.15m clearance per side through corridors.
    cs = nav2_params['controller_server']['ros__parameters']
    cs['progress_checker']['required_movement_radius'] = 0.20
    cs['progress_checker']['movement_time_allowance'] = 30.0
    cs['FollowPath']['vx_max'] = 0.35
    cs['FollowPath']['vx_min'] = -0.35
    cs['FollowPath']['vx_std'] = 0.15
    cs['FollowPath']['wz_max'] = 1.0
    cs['FollowPath']['wz_std'] = 0.30
    nav2_params['local_costmap']['local_costmap']['ros__parameters'] \
        ['inflation_layer']['inflation_radius'] = 0.30
    nav2_params['global_costmap']['global_costmap']['ros__parameters'] \
        ['inflation_layer']['inflation_radius'] = 0.30

    nav2_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(nav2_params, nav2_tmp, default_flow_style=False)
    nav2_tmp.close()
    nav2_patched = nav2_tmp.name

    collision_patched = _patch_yaml(
        os.path.join(nav_pkg, 'config', 'collision_monitor.yaml'), SIM)
    slam_patched = _patch_yaml(os.path.join(slam_pkg, 'config', 'slam_toolbox.yaml'), SIM)
    imu_patched  = _patch_yaml(os.path.join(fusion, 'config', 'imu_filter.yaml'), SIM)
    expl_patched = _patch_yaml(os.path.join(explore, 'config', 'explore.yaml'), SIM)

    # In sim, Madgwick 6-DOF cannot correct yaw drift (no magnetometer).
    # Switch EKF to use wheel odom for yaw — sim odometry is perfect, no slip.
    # odom0_config / imu0_config are flat 15-element lists:
    # [x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]  (indices 0-14)
    with open(os.path.join(fusion, 'config', 'ekf.yaml')) as f:
        ekf_data = yaml.safe_load(f)
    params = ekf_data['ekf_filter_node']['ros__parameters']
    params['use_sim_time'] = True
    params['odom0_config'][5]  = True   # yaw ON from wheel odom
    params['odom0_config'][11] = True   # vyaw ON from wheel odom
    params['imu0_config'][5]   = False  # yaw OFF from IMU (Madgwick can't fix yaw drift)
    params['imu0_config'][11]  = False  # vyaw OFF from IMU
    ekf_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(ekf_data, ekf_tmp, default_flow_style=False)
    ekf_tmp.close()
    ekf_patched = ekf_tmp.name

    os.makedirs('/amr_ws/bags', exist_ok=True)
    os.makedirs('/amr_ws/maps', exist_ok=True)

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'amr', '-topic', '/robot_description',
                   '-x', '0.0', '-y', '-7.0', '-z', '0.1'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
    )

    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/mecanum_drive_controller/odometry', '/odom/wheel'],
    )
    cmd_vel_relay = Node(
        package='amr_imu',
        executable='cmd_vel_safe_relay',
        name='cmd_vel_safe_relay',
        output='screen',
        parameters=[SIM],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_patched],
        remappings=[('imu/data_raw', '/imu/data_raw'),
                    ('imu/data',     '/imu/data')],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_patched],
        remappings=[('odometry/filtered', '/odom')],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_patched],
    )
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['slam_toolbox'], **SIM}],
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_patched], remappings=[('cmd_vel', '/cmd_vel')],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_patched],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_patched],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_patched],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[collision_patched],
    )
    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'bt_navigator', 'behavior_server',
                                    'collision_monitor'], **SIM}],
    )

    explore_node = Node(
        package='explore_lite', executable='explore',
        name='explore', output='screen',
        parameters=[expl_patched],
    )
    home_manager = Node(
        package='amr_home_manager', executable='home_manager',
        name='amr_home_manager', output='screen',
        parameters=[{'map_save_path': '/amr_ws/maps/sim_explore_map', **SIM}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'sim.rviz')],
        parameters=[SIM],
        output='screen',
    )

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', '/amr_ws/bags/sim_demo'],
        output='screen',
    )

    return [
        gz_sim,
        robot_state_publisher,
        rviz,
        bag_record,
        TimerAction(period=2.0,  actions=[spawn]),
        TimerAction(period=3.0,  actions=[bridge]),
        TimerAction(period=4.0,  actions=[joint_state_broadcaster,
                                          mecanum_controller]),
        TimerAction(period=5.0,  actions=[odom_relay, cmd_vel_relay,
                                          imu_filter, ekf]),
        TimerAction(period=7.0,  actions=[slam, slam_lifecycle]),
        TimerAction(period=10.0, actions=[controller_server, planner_server,
                                          bt_navigator, behavior_server,
                                          collision_monitor, nav2_lifecycle]),
        TimerAction(period=20.0, actions=[explore_node, home_manager]),
        TimerAction(period=25.0, actions=[
            LogInfo(msg=(
                '\n'
                '══════════════════════════════════════════\n'
                '  AURORA SIM READY — exploring autonomously\n'
                '  Use RViz2 "2D Nav Goal" after exploration\n'
                '══════════════════════════════════════════\n'
            )),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
