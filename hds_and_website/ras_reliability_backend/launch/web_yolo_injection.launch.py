from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='copilot',
            executable='copilot',
            name='copilotrv'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            launch_arguments={
                'enable_rgbd': 'true', 
                'enable_sync': 'true', 
                'align_depth.enable': 'true', 
                'enable_color': 'true', 
                'enable_depth': 'true'
            }.items()
        ),
        Node(
            package='yolov6',
            executable='inferer',
            name='inferer'
        ),
        Node(
            package='srobot_realistic_output',
            executable='subscriber',
            name='subscriber'
        )
    ])
