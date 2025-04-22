from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Set the TURTLEBOT3_MODEL environment variable
    set_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # Get the launch directory of the turtlebot3_orbslam_bringup package
    bringup_dir = get_package_share_directory('turtlebot3_orbslam_bringup')
    
    # Include the turtlebot3_house launch file
    turtlebot3_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'turtlebot3_house.launch.py')
        )
    )

    # Launch the image enhancer Python script
    image_enhancer = Node(
            package='orbslam3',
            executable=LaunchConfiguration('python_executable'),
            name='image_enhancer',
            arguments=[os.path.join(
                get_package_share_directory('orbslam3'),
                'lib',
                'orbslam3',
                'image_enhancer.py'
            )],
            output='screen'
    )

    # Launch the ORB-SLAM3 node with specified arguments
    orbslam3_node = Node(
        package='orbslam3',
        executable='mono',
        name='orbslam3_mono',
        output='screen',
        arguments=[
            '/home/kaushek/MRob/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            './src/orbslam3_ros2/config/monocular-inertial/RealSense_T265.yaml',
            'true'
        ]
    )
    
    return LaunchDescription([
        set_model_env,
        turtlebot3_house,
        image_enhancer,
        orbslam3_node
    ])
