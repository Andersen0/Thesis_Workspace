from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='srobot_realistic_output',
            executable='publisher',
            name='publisher_node'
        ),
        Node(
            package='srobot_realistic_output',
            executable='subscriber',
            name='subscriber_node'
        ),
    ])
