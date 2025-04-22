from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set the environment variable
        # SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        # Launch TurtleBot3 simulation
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'bcr_bot', 'gazebo.launch.py',
                'camera_enabled:=True',
                'two_d_lidar_enabled:=True',
                'stereo_camera_enabled:=False',
                'position_x:=-5.0',
                'position_y:=6.0',
                'orientation_yaw:=0.0',
                'odometry_source:=world',
                'world_file:=world_rob530.sdf',
                'robot_namespace:=bcr_bot'
            ],
            output='screen'
        ),

        # Run the Python image enhancer script
        ExecuteProcess(
            cmd=['python3', 'install/orbslam3/lib/orbslam3/image_enhancer_rgbd.py'],
            cwd='.',  # set current working directory
            output='screen'
        ),
        
        # Run the Python image enhancer script
        # ExecuteProcess(
        #     cmd=['python3', 'install/orbslam3/lib/orbslam3/image_enhancer.py'],
        #     cwd='.',  # set current working directory
        #     output='screen'
        # ),

        # Launch ORB-SLAM3 node
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'orbslam3', 'rgbd',
                '/home/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                './src/orbslam3_ros2/config/rgb-d/bcr_bot.yaml',
                'true'
            ],
            
            # cmd=[
            #     'ros2', 'run', 'orbslam3', 'mono',
            #     '/home/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            #     './src/orbslam3_ros2/config/monocular/bcr_bot.yaml',
            #     'true'
            # ],
            
            # cmd=[
            #     'ros2', 'run', 'orbslam3', 'mono',
            #     '/home/kaushek/MRob/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            #     './src/orbslam3_ros2/config/monocular/RealSense_R200.yaml',
            #     'true'
            # ],
            
            # cmd=[
            #     'ros2', 'run', 'orbslam3', 'mono-inertial',
            #     '/home/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            #     '/home/colcon_ws/src/orbslam3_ros2/config/monocular-inertial/bcr_bot.yaml',
            #     'true'
            # ],
            output='screen'
        )
    ])
