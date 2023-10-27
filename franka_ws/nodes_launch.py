from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detector',
            executable='yolo_detector',
            name='yolo_detector'
        ),
        Node(
            package='depth_estimator',
            executable='realsense_depth_estimator',
            name='realsense_depth_estimator'
        )
    ])
