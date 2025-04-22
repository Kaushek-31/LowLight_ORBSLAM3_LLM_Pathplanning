#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Launch arguments
camera_enabled = LaunchConfiguration('camera_enabled', default='True')
two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled', default='True')
stereo_camera_enabled = LaunchConfiguration('stereo_camera_enabled', default='False')
position_x = LaunchConfiguration('position_x', default='-5.0')
position_y = LaunchConfiguration('position_y', default='7.0')
orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')
odometry_source = LaunchConfiguration('odometry_source', default='world')
robot_namespace = LaunchConfiguration('robot_namespace', default='bcr_bot')
use_sim_time = LaunchConfiguration('use_sim_time', default='True')

# Locate package share
pkg_share = get_package_share_directory('bcr_bot')

# File paths
urdf_xacro = os.path.join(pkg_share, 'urdf', 'bcr_bot.xacro')
map_yaml = os.path.join(pkg_share, 'maps', 'bcr_map.yaml')
rviz_config = os.path.join(pkg_share, 'rviz', 'entire_setup.rviz')
ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
world_file = os.path.join(pkg_share, 'worlds', 'world_rob530.sdf')

# Process XACRO to XML
def process_xacro(xacro_file, mappings=None):
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings=mappings or {})
    return doc.toxml()

robot_desc_xml = process_xacro(urdf_xacro, {'wheel_odom_topic': 'odom'})

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch args
    ld.add_action(DeclareLaunchArgument('camera_enabled', default_value='True'))
    ld.add_action(DeclareLaunchArgument('two_d_lidar_enabled', default_value='True'))
    ld.add_action(DeclareLaunchArgument('stereo_camera_enabled', default_value='False'))
    ld.add_action(DeclareLaunchArgument('position_x', default_value='-5.0'))
    ld.add_action(DeclareLaunchArgument('position_y', default_value='7.0'))
    ld.add_action(DeclareLaunchArgument('orientation_yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('odometry_source', default_value='world'))
    ld.add_action(DeclareLaunchArgument('robot_namespace', default_value='bcr_bot'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))

    # Static transforms
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map']
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_tf',
        arguments=['0', '0.06', '0.02', '0', '0', '0', 'base_link', 'imu_frame']
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_laser_tf',
        arguments=['0.06', '0', '0.08', '0', '0', '0', 'base_link', 'laser']
    ))

    # Gazebo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'camera_enabled': camera_enabled,
            'two_d_lidar_enabled': two_d_lidar_enabled,
            'stereo_camera_enabled': stereo_camera_enabled,
            'position_x': position_x,
            'position_y': position_y,
            'orientation_yaw': orientation_yaw,
            'odometry_source': odometry_source,
            'world_file': world_file,
            'robot_namespace': robot_namespace,
            'use_sim_time': use_sim_time,
        }.items()
    ))

    # Spawn robot after Gazebo
    ld.add_action(TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'bcr_bot',
                '-file', urdf_xacro,
                '-x', position_x, '-y', position_y, '-z', '0.01'
            ],
            output='screen'
        )]
    ))

    # Map server
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'static_map.launch.py'))
    ))

    # EKF
    ld.add_action(Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    ))

    # Semantic obstacle extractor
    ld.add_action(Node(
        package='bcr_bot', executable='semantic_obstacle_extractor', name='semantic_obstacle_extractor',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # D* Lite planner
    ld.add_action(Node(
        package='bcr_bot', executable='dstar_lite_planner', name='dstar_lite_planner',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # MPC controller
    ld.add_action(Node(
        package='bcr_bot', executable='mpc_controller', name='mpc_controller',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # LLM Path Planner
    ld.add_action(Node(
        package='bcr_bot', executable='llm_path_planner', name='llm_path_planner',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # Object Marker Publisher
    ld.add_action(Node(
        package='bcr_bot', executable='object_marker_publisher', name='object_marker_publisher',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # Waypoint Executor
    ld.add_action(Node(
        package='bcr_bot', executable='waypoint_executor', name='waypoint_executor',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    ))

    # Robot State Publisher & RViz
    ld.add_action(GroupAction([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_desc_xml}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            remappings=[
                ('/map', '/map'),
                ('/dstar_path', '/dstar_waypoints'),
                ('/object_markers', '/obstacles'),
                ('/odom', '/odom'),
                ('/tf', '/tf'),
                ('/llm_waypoints_pose_array', '/llm_waypoints_pose_array'),
                ('/llm_waypoints_path', '/llm_waypoints_path')
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]))

    return ld
