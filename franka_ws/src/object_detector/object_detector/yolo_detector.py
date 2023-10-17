#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

import cv2

# Define the YOLO detector node
class YoloDetectorNode(Node):

    def __init__(self):
        # Give a node name
        super().__init__("yolo_detector")

        # Load pretrained YOLOv8 model
        # The yolov8m model may be too computational intensive
        self.yolo_model_ = YOLO('yolov8n.pt')
        self.class_names_ = self.yolo_model_.names

        self.results_ = None

    def detect_objects(self):
        # Define streaming source (e.g. camera dev number)
        # May need to try out for different ports
        camera_dev = '7'

        # Return a list of Results objects
        self.results_ = self.yolo_model_(source=camera_dev, 
                        show=True, 
                        # save=True, 
                        boxes=True,
                        stream=True
                    )

    def init_publisher(self):
        # Create a publisher
        self.yolo_object_pub_ = self.create_publisher(String, "/vision/yolo_object", qos_profile=10)

        # Create a timer, every 1 sec
        self.timer_ = self.create_timer(1, self.publish_objects_message)

    def publish_objects_message(self):
        # Process results list
        for result in self.results_:
            confidences = result.boxes.conf
            class_ids = result.boxes.cls
            box_coordinates = result.boxes.xyxy


            # print("=============\n")
            for i in range(len(class_ids)):
                class_id = class_ids[i]
                coordinates = box_coordinates[i]

                x1, y1 = float(coordinates[0]), float(coordinates[1])
                x2, y2 = float(coordinates[2]), float(coordinates[3])


                msg_content = ""
                msg_content += "Class: {}\n".format(self.class_names_[int(class_id)])
                msg_content += "Confidence: {:.2f}\n".format(float(confidences[i]))
                msg_content += "Top left coordinates: ({:.2f}, {:.2f})\n".format(x1, y1)
                msg_content += "Bot right coordinates: ({:.2f}, {:.2f})".format(x2, y2)

                msg_str = String()
                msg_str.data = msg_content
                self.yolo_object_pub_.publish(msg=msg_str)

def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the YOLO detector node
    yolo_node = YoloDetectorNode()

    # Start detecting objects
    yolo_node.detect_objects()
    # Publish messages of detected objects
    yolo_node.init_publisher()
    yolo_node.publish_objects_message()

    rclpy.spin(node=yolo_node)

    # Stop ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()