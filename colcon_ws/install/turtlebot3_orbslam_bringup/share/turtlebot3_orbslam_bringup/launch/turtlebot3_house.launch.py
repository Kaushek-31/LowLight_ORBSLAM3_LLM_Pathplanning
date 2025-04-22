from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to the turtlebot3_gazebo package
    turtlebot3_gazebo_dir = '/opt/ros/humble/share/turtlebot3_gazebo'

    # Set the path to the world file
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_house.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
    ])

