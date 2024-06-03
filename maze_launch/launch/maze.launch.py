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
            default_value=os.path.join(get_package_share_directory('maze_launch'), 'worlds', 'generated.world'),
            description='Full path to world model file to load'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            # Pass the specified world file as an argument
            launch_arguments=[('world', LaunchConfiguration('world'))],
        ),
    ])

