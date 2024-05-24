import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'empty_world.world'
    world_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', world_file_name)
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_launch_dir, 'gazebo.launch.py')]),
            launch_arguments={'world': world_path}.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf')],
            output='screen'),

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
        ),
        # Add the toggle_light_plugin node here if it has a separate ROS node
    ])

if __name__ == '__main__':
    generate_launch_description()
