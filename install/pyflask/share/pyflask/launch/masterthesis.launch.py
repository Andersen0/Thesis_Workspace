import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['./launch.sh'],  # Relative path to launch.sh from pyflask directory
            cwd=['/home/eliash/colcon_ws/src/Thesis_Workspace/hds_and_website/ras_reliability_backend/pyflask'],
            output='screen',
        ),
        Node(
            package='srobot_publisher',
            executable='publisher',
            name='my_publisher'
        )
    ])
