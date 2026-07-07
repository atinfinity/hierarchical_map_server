# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""Integrated driving test of TurtleBot3 Gazebo + dual-costmap Nav2 + hierarchical map (full localization version).

Verifies the main goal of design 3: "whole-area path planning to goals outside the
high-res window". Self-localization uses a static map->odom from Gazebo ground truth
(same reason as the groundtruth version of design 1: to avoid environment-dependent
nav2_amcl crashes). For the amcl configuration, see hierarchical_localization.launch.py.

  tile_map_server          -> /map (high-res sliding window, 6m)
  global_lowres_map_server -> /map_global_lowres (low-res whole area)
  global_costmap.static_layer <- /map_global_lowres  (whole area is plannable)
  local_costmap            <- /scan (real time)
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
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    pkg = get_package_share_directory('hierarchical_map_server')
    tms_dir = get_package_share_directory('tile_map_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    params_file = LaunchConfiguration('params_file')
    tileset_path = LaunchConfiguration('tileset_path')
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')

    spawn_x, spawn_y = -2.0, -0.5
    pose = {'x': str(spawn_x), 'y': str(spawn_y), 'z': '0.01',
            'R': '0.00', 'P': '0.00', 'Y': '0.00'}
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_hierarchical_params.yaml')),
        DeclareLaunchArgument(
            'tileset_path',
            default_value=os.path.join(
                tms_dir, 'maps', 'tb3_sandbox_tiles', 'tileset.yaml'),
            description='Pre-split tb3_sandbox tiles bundled with design 1'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')),
    ]

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))

    world_sdf = tempfile.mktemp(prefix='tile_hier_', suffix='.sdf')
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
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    # Full localization: publish map->odom statically (odom origin = spawn position)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_truth_map_to_odom',
        output='screen',
        arguments=['--x', str(spawn_x), '--y', str(spawn_y), '--z', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings)

    # High-res sliding window (design 1)
    tile_map_server = Node(
        package='tile_map_server',
        executable='tile_map_server_node',
        name='tile_map_server',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'tb3_sim_tile_params.yaml'),
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    # Low-res whole-area map (this package)
    global_lowres = Node(
        package='hierarchical_map_server',
        executable='global_lowres_map_server',
        name='global_lowres_map_server',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'global_lowres_params.yaml'),
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['tile_map_server', 'global_lowres_map_server']}])

    navigation = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': 'False',
                'namespace': ''}.items())])

    ld = LaunchDescription()
    for d in declare:
        ld.add_action(d)
    ld.add_action(set_gz_resource_path)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf)
    ld.add_action(gazebo_server)
    ld.add_action(gz_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(map_to_odom)
    ld.add_action(tile_map_server)
    ld.add_action(global_lowres)
    ld.add_action(lifecycle_localization)
    ld.add_action(navigation)
    return ld
