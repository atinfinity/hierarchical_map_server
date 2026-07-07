"""Launch file for running tile_map_server standalone.

To integrate into a Nav2 bringup, replace map_server with tile_map_server_node
and add 'tile_map_server' to the node_names of the existing
lifecycle_manager_localization.
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
            description='Parameter YAML for tile_map_server (must include tileset_path)'),
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
