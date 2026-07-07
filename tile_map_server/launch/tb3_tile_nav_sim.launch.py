# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""Launch file for TurtleBot3 Gazebo + Nav2 + tile_map_server integration drive test.

A configuration that replaces the map serving of the standard Nav2 tb3_simulation
with tile_map_server.
  Gazebo (gz sim, headless) + TB3 spawn + robot_state_publisher
  tile_map_server + amcl  (managed by lifecycle_manager_localization)
  navigation_launch.py    (controller / planner / costmaps / bt / nav lifecycle)

Usage:
  ros2 launch tile_map_server tb3_tile_nav_sim.launch.py
  # Boundary-crossing navigation in a separate terminal:
  ros2 run tile_map_server nav_across_boundary_test.py    (bundled under test/)
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    pkg_dir = get_package_share_directory('tile_map_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    params_file = LaunchConfiguration('params_file')
    tile_params_file = LaunchConfiguration('tile_params_file')
    tileset_path = LaunchConfiguration('tileset_path')
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    use_rviz = LaunchConfiguration('use_rviz')

    # Spawn position (start point for boundary-crossing navigation)
    pose = {'x': '-2.00', 'y': '-0.50', 'z': '0.01', 'R': '0.00', 'P': '0.00', 'Y': '0.00'}

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare_cmds = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Parameters for the Nav2 stack'),
        DeclareLaunchArgument(
            'tile_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'tb3_sim_tile_params.yaml'),
            description='Parameters for tile_map_server'),
        DeclareLaunchArgument(
            'tileset_path',
            default_value=os.path.join(
                pkg_dir, 'maps', 'tb3_sandbox_tiles', 'tileset.yaml'),
            description='Pre-split tileset'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')),
    ]

    # Add the model path so gz sim can resolve model://turtlebot3_world
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))

    # --- Gazebo world (xacro -> temp sdf -> gz sim -s -r) ---
    world_sdf = tempfile.mktemp(prefix='tile_nav_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen')
    remove_temp_sdf = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]))

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': 'turtlebot3_waffle',
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'], 'y_pose': pose['y'], 'z_pose': pose['z'],
            'roll': pose['R'], 'pitch': pose['P'], 'yaw': pose['Y']}.items())

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        remappings=remappings)

    # --- Localization: tile_map_server + amcl ---
    tile_map_server = Node(
        package='tile_map_server',
        executable='tile_map_server_node',
        name='tile_map_server',
        output='screen',
        parameters=[tile_params_file,
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        # The nav2_amcl pf_kdtree assertion is a known upstream bug that occurs
        # intermittently under the high load of starting all nodes at once.
        # Use respawn to recover automatically.
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file,
                    {'use_sim_time': use_sim_time,
                     # Accept the window updates from tile_map_server (key to the integration)
                     'first_map_only': False}],
        # set_initial_pose is not used because it triggers the nav2_amcl kdtree
        # assertion; instead publish /initialpose from the integration test script to converge
        remappings=remappings)

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['tile_map_server', 'amcl']}])

    # --- Navigation stack (does not include map_server) ---
    # Since the nav2_amcl pf_kdtree assertion occurs intermittently under the high
    # load of simultaneous startup, start the nav stack after localization (amcl)
    # is stable to spread out the load.
    navigation = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': 'False',
                'namespace': ''}.items())])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    ld = LaunchDescription()
    for c in declare_cmds:
        ld.add_action(c)
    ld.add_action(set_gz_resource_path)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf)
    ld.add_action(gazebo_server)
    ld.add_action(gz_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(tile_map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_localization)
    ld.add_action(navigation)
    ld.add_action(rviz)
    return ld
