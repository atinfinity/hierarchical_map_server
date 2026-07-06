# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""階層localizationモジュール(再利用可能)。

  tile_map_server        -> /map                (高解像度スライディング窓, amcl用)
  global_lowres_map_server -> /map_global_lowres (低解像度全域, global costmap用)
  amcl                   -> /map を購読して自己位置推定

3ノードを lifecycle_manager_localization が一括管理する。Nav2 bringup に組み込む
場合は localization_launch.py の代わりにこれを使い、navigation_launch.py を
nav2_hierarchical_params.yaml で起動する。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hierarchical_map_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    tileset_path = LaunchConfiguration('tileset_path')
    nav2_params = LaunchConfiguration('params_file')
    tile_params = LaunchConfiguration('tile_params_file')
    lowres_params = LaunchConfiguration('lowres_params_file')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('tileset_path', description='案1のtileset.yamlへの絶対パス'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_hierarchical_params.yaml'),
            description='amclを含むNav2パラメータ'),
        DeclareLaunchArgument(
            'tile_params_file',
            default_value=os.path.join(pkg, 'config', 'tb3_sim_tile_params.yaml')),
        DeclareLaunchArgument(
            'lowres_params_file',
            default_value=os.path.join(pkg, 'config', 'global_lowres_params.yaml')),
    ]

    tile_map_server = Node(
        package='tile_map_server',
        executable='tile_map_server_node',
        name='tile_map_server',
        output='screen',
        parameters=[tile_params,
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    global_lowres = Node(
        package='hierarchical_map_server',
        executable='global_lowres_map_server',
        name='global_lowres_map_server',
        output='screen',
        parameters=[lowres_params,
                    {'use_sim_time': use_sim_time, 'tileset_path': tileset_path}],
        remappings=remappings)

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=remappings)

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['tile_map_server',
                                    'global_lowres_map_server',
                                    'amcl']}])

    ld = LaunchDescription()
    for d in declare:
        ld.add_action(d)
    ld.add_action(tile_map_server)
    ld.add_action(global_lowres)
    ld.add_action(amcl)
    ld.add_action(lifecycle)
    return ld
