#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_file = os.path.join(
        os.getenv('HOME'),
        'bcr_bot', 'src',
        'config', 'bcr_map.yaml'
    )

    return LaunchDescription([
        # 1) The map_server itself
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_file}],
            output='screen'
        ),

        # 2) Lifecycle manager to auto‐configure & activate map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'use_sim_time': False,
                'node_names': ['map_server']
            }]
        ),
    ])
