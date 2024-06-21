from __future__ import annotations

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    robot_name = LaunchConfiguration("robot_name")

    # Create launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name="robot_name", default_value="robot_model"
    )

    start_spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name,
            "-x",
            "-1.1830045919634307",
            "-y",
            "-3.467631140636199",
            "-z",
            "0.35",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "1.5499498283934454",
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(start_spawn_entity_cmd)

    return ld