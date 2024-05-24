#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_included_launch', default_value='true',
                              description='Whether to include the included_launch.py'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("turtlebot3_gazebo"),
                "launch/empty_world.launch.py")
                )
         ),
       Node(
            package='imrt_virtual_joy',
            executable='virtual_gamepad',
            name='virtual_gamepad'
        ),
       Node(
           package='imrt_teleop',
           executable='teleop_triggers',
           name='teleop',
           remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/teleop/button1_trigger', 'text_to_speech/say_random'),
              ],
       )
    ])

