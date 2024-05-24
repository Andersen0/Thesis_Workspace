from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    virtual_gamepad_node = launch_ros.actions.Node(
        namespace="imrt_virtual_joy",
        package="imrt_virtual_joy",
        executable="virtual_gamepad",
        output="screen"
    )

    return LaunchDescription([
        virtual_gamepad_node
    ])

