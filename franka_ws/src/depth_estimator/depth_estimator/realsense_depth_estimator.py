#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from realsense_depth import *

# Define the depth estimator node
class RealsenseDepthEstimatorNode(Node):

    def __init__(self):
        # Give a node name
        super().__init__("realsense_depth_estimator")

        self.colour_frame = None
        self.depth_frame = None

    def estimate_distance(self):
        pass

    def subscribe_object_info(self):
        # Create a message subscriber
        self.object_subscriber_ = self.create_subscription(String, "/vision/yolo_object", self.extract_object_info, 10)

    def extract_object_info(self, msg: String):
        msg_content = msg.data
        self.get_logger().info("Object info received: " + msg_content)

    def init_publisher(self):
        # Create a publisher
        self.depth_distance_pub_ = self.create_publisher(String, "/vision/realsense_depth_distance", qos_profile=10)

        # Create a timer, every 1 sec
        self.timer_ = self.create_timer(1, self.publish_object_distance_message)

    def publish_object_distance_message(self):
        pass

def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the depth estimator node
    depth_estimator_node = RealsenseDepthEstimatorNode()



    rclpy.spin(node=depth_estimator_node)

    # Stop ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()