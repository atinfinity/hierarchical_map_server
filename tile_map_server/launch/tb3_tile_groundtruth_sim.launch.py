# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""TurtleBot3 Gazebo + Nav2 + tile_map_server 結合走行テスト(完全自己位置版)。

tb3_tile_nav_sim.launch.py のうち amcl を、Gazeboのグラウンドトゥルースに基づく
静的な map->odom TF(=完全自己位置)に置き換えた構成。

意図: nav2_amcl は環境によって起動時に pf_kdtree アサーションでクラッシュする
既知の上流バグがある。このlaunchはそれを避け、tile_map_server の結合面
  - ロボット走行に伴う map->base_footprint TF 監視 → タイル窓の再センタリング
  - global costmap(static layer)が窓更新を消費して経路計画
  - タイル境界を跨ぐゴールへのナビゲーション成功
を Gazebo 実機シミュレーション上で検証する。自己位置推定そのものを AMCL で
評価したい場合は tb3_tile_nav_sim.launch.py を使う(要: 安定した nav2_amcl)。

robot は (-2, -0.5) にスポーンし、gz の odom フレームはスポーン位置を原点とする。
したがって map->odom は静的に (-2, -0.5) 平行移動となる。
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
    pkg_dir = get_package_share_directory('tile_map_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    params_file = LaunchConfiguration('params_file')
    tile_params_file = LaunchConfiguration('tile_params_file')
    tileset_path = LaunchConfiguration('tileset_path')
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')

    spawn_x, spawn_y = -2.0, -0.5
    pose = {'x': str(spawn_x), 'y': str(spawn_y), 'z': '0.01',
            'R': '0.00', 'P': '0.00', 'Y': '0.00'}
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare_cmds = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'tile_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'tb3_sim_tile_params.yaml')),
        DeclareLaunchArgument(
            'tileset_path',
            default_value=os.path.join(
                pkg_dir, 'maps', 'tb3_sandbox_tiles', 'tileset.yaml')),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')),
    ]

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))

    world_sdf = tempfile.mktemp(prefix='tile_gt_', suffix='.sdf')
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

    # 完全自己位置: map->odom を静的に発行(odom原点=スポーン位置なので spawn 分の平行移動)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_truth_map_to_odom',
        output='screen',
        arguments=['--x', str(spawn_x), '--y', str(spawn_y), '--z', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings)

    tile_map_server = Node(
        package='tile_map_server',
        executable='tile_map_server_node',
        name='tile_map_server',
        output='screen',
        parameters=[tile_params_file,
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['tile_map_server']}])

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
    for c in declare_cmds:
        ld.add_action(c)
    ld.add_action(set_gz_resource_path)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf)
    ld.add_action(gazebo_server)
    ld.add_action(gz_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(map_to_odom)
    ld.add_action(tile_map_server)
    ld.add_action(lifecycle_localization)
    ld.add_action(navigation)
    return ld
