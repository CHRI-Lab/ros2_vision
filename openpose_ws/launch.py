from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openpose_controller',
            executable='openpose_node',
            name='openpose_node'
        )

    ])