#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import subprocess


class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        #self.publisher = self.create_publisher()

        #self.get_logger().info("Hello from ROS23")


    def run_api(self):
        try:
            # Replace '/path/to/external_script.py' with the actual path to your external Python script
            external_script_path = '../openpose/build/examples/tutorial_api_python/01_body_from_image.py'
            subprocess.run(['python3', external_script_path])
        except Exception as e:
            self.get_logger().error(f'Error executing external script: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    keypoint_node = KeypointNode()

    keypoint_node.run_api()

    rclpy.spin(keypoint_node) # This node is going to be kept alive, until you actively kill it
    keypoint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()