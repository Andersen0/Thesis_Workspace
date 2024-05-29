#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    log_level = LaunchConfiguration('log_level')

    # Declare the log level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Set the log level for all nodes.')

    # Set environment variable for all nodes in this launch file
    set_log_level = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY', log_level
    )

    # Include another launch description from the turtlebot3_gazebo package
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch/empty_world.launch.py")
        )
    )

    # Define the virtual gamepad node
    virtual_gamepad_node = Node(
        package='imrt_virtual_joy',
        executable='virtual_gamepad',
        name='virtual_gamepad',
        output='screen'
    )

    # Define the teleop triggers node
    teleop_triggers_node = Node(
        package='imrt_teleop',
        executable='teleop_triggers',
        name='teleop',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/teleop/button1_trigger', 'text_to_speech/say_random'),
        ],
        output='screen'
    )

    return LaunchDescription([
        log_level_arg,
        set_log_level,
        included_launch,
        virtual_gamepad_node,
        teleop_triggers_node
    ])

