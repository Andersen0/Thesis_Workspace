from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='copilot',
            executable='copilot',
            name='copilot_monitor'
        ),
        Node(
            package='copilot',
            executable='copilot_logger',
            name='copilot_logger'
        ),
    ])
