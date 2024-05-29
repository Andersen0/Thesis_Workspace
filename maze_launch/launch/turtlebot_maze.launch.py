
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument for the world file
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(get_package_share_directory("maze_launch"), 'worlds', 'big_hello.world'),

            description='Specify the Gazebo world file to load',
        ),

        # Include launch file from gazebo_ros
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('maze_launch'), 'launch', 'maze.launch.py')
            ),
            # Pass the specified world file as an argument
            launch_arguments=[('world', LaunchConfiguration('world'))],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')) 
        ),

    ])

