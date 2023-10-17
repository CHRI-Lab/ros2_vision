#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import re
import cv2
from realsense_depth import *

# Define the depth estimator node
class RealsenseDepthEstimatorNode(Node):

    def __init__(self):
        # Give a node name
        super().__init__("realsense_depth_estimator")

        # Connect RealSense depth camera
        self.depth_camera = DepthCamera()

        self.colour_frame = None
        self.depth_frame = None

    def subscribe_object_info(self):
        # Create a message subscriber
        self.object_subscriber_ = self.create_subscription(String, "/vision/yolo_object", self.extract_object_info, 10)

    def extract_object_info(self, msg: String):
        # Read original message content received
        msg_content = msg.data
        # self.get_logger().info("Object info received: " + msg_content)

        # Extract coordinates floats
        content_lst = msg_content.strip().split("\n")
        float_rx = re.compile(r'\d+\.\d+')

        top_left_coordinates = [float(i) for i in float_rx.findall(content_lst[2])]
        bot_right_coordinates = [float(i) for i in float_rx.findall(content_lst[3])]
        top_left_coordinates = (top_left_coordinates[0], top_left_coordinates[1])
        bot_right_coordinates = (bot_right_coordinates[0], bot_right_coordinates[1])

        self.get_logger().info("Object info received: {}, coordinates: [{}, {}]".format(content_lst[0], top_left_coordinates, bot_right_coordinates))

    def init_publisher(self):
        # Create a publisher
        self.depth_distance_pub_ = self.create_publisher(String, "/vision/realsense_depth_distance", qos_profile=10)

        # Create a timer, every 1 sec
        self.timer_ = self.create_timer(1, self.publish_object_distance_message)

    def publish_object_distance_message(self):
        pass

    def estimate_distance(self):
        pass

def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the depth estimator node
    # Establish connection to depth camera
    depth_estimator_node = RealsenseDepthEstimatorNode()

    depth_estimator_node.subscribe_object_info()




    rclpy.spin(node=depth_estimator_node)

    # Stop ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()