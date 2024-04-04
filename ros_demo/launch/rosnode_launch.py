from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for copilot
        Node(
            package='copilot',
            executable='copilot',
            name='copilotrv'
        ),
        # Node for copilot_logger
        Node(
            package='copilot',
            executable='copilot_logger',
            name='copilot_logger'

        ),
        # Node for publisher
#        Node(
#            package='srobot_realistic_output',
#            executable='publisher',
#            name='publisher'
#        ),
        # Node for subscriber
#        Node(
#            package='srobot_realistic_output',
#            executable='subscriber',
#            name='subscriber'
#        ),
    ])
