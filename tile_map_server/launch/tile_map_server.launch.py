"""tile_map_server 単体起動用launch。

Nav2のbringupに組み込む場合はmap_serverをtile_map_server_nodeに置き換え、
既存のlifecycle_manager_localizationのnode_namesに'tile_map_server'を追加する。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='tile_map_serverのパラメータYAML(tileset_pathを含めること)'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='tile_map_server',
            executable='tile_map_server_node',
            name='tile_map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_tile_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['tile_map_server'],
            }],
        ),
    ])
