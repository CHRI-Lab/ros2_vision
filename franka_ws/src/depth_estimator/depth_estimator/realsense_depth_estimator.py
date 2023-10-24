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

        self.object_subscriber_ = None
        self.depth_distance_pub_ = None

        # Connect RealSense depth camera
        self.depth_camera = DepthCamera()

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

        object_class = content_lst[0].split()[1]

        # self.get_logger().info("Object info received: {}, coordinates: [{}, {}]".format(content_lst[0], top_left_coordinates, bot_right_coordinates))

        # Pass received coordinates to extract corresponding distance
        self.estimate_distance(object_class, top_left_coordinates, bot_right_coordinates)

    def init_publisher(self):
        # Create a publisher
        # Specify topic to publish
        self.depth_distance_pub_ = self.create_publisher(String, "/vision/realsense_depth_distance", qos_profile=10)

    def publish_object_distance_message(self, object_class, centre_pos, distance):
        msg_content = ""
        msg_content += "Object: {}\n".format(object_class)
        msg_content += "Centre: ({:.2f}, {:.2f})\n".format(centre_pos[0], centre_pos[1])
        msg_content += "Distance: {:.2f}mm".format(distance)

        msg_str = String()
        msg_str.data = msg_content
        self.depth_distance_pub_.publish(msg=msg_str)

    def estimate_distance(self, object_class, top_left_coordinates, bot_right_coordinates):
        # Calculate the centre coordinates of objects
        x = int((top_left_coordinates[0] + bot_right_coordinates[0]) // 2)
        y = int((top_left_coordinates[1] + bot_right_coordinates[1]) // 2)

        # self.get_logger().info("Original: {}, Centre: {}".format([top_left_coordinates, bot_right_coordinates], (x, y)))

        res, depth_frame = self.depth_camera.get_frame()
        distance = depth_frame[y, x]  # [Y_pos, X_pos]

        # Ignore invalid estimates
        if distance != 0:
            # self.get_logger().info("\nObject: {}\nCentre: {}\nDistance: {}mm\n".format(object_class, (x,y), distance))
            self.publish_object_distance_message(object_class, (x,y), distance)

            # Create a timer, every 1 sec
            # self.timer_ = self.create_timer(1, self.publish_object_distance_message(object_class, (x,y), distance))

def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the depth estimator node
    # Establish connection to depth camera
    depth_estimator_node = RealsenseDepthEstimatorNode()

    depth_estimator_node.init_publisher()

    # Start receive info about detected objects
    depth_estimator_node.subscribe_object_info()




    rclpy.spin(node=depth_estimator_node)

    # Stop ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()