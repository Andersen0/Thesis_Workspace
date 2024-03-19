# my_launch_file.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pyflask',
            executable='server',
            name='my_server'
        ),
#        Node(
#            package='pyflask',
#           executable='talker',
#            name='my_talker'
#    ),
#        Node(
#         package='pyflask',
#          executable='robot',
#           name='my_robot'
#        ),
       Node(
           package='yolov6',
           executable='inferer',
           name='inferer'
       )
#        Node(
#            package='pyflask',
#            executable='cycle_dist',
#            name='cycle_dist'
#        )
    ])
