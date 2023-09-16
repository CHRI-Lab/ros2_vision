#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Define the YOLO detector node
class YoloDetectorNode(Node):

    def __init__(self):
        # Give a node name
        super().__init__("yolo_detector")

    def publish_objects(self):
        pass

def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the YOLO detector node
    yolo_node = YoloDetectorNode()

    rclpy.spin(node=yolo_node)

    # Stop ROS2 communication
    rclpy.shutdown()