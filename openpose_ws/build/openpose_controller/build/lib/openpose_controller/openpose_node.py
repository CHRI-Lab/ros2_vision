#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import subprocess
import cv2
import os
from sys import platform
import argparse
import time
from std_msgs.msg import String

#from your_message_package.msg import FaceKeypoints, BodyKeypoints, HandKeypoints  # Import your specific message types



sys.path.append('../openpose/build/python')
from openpose import pyopenpose as op


class OpenWrapper(Node):

    def __init__(self, args):
        

        self.params = dict()
        self.params["model_folder"] = "../openpose/models/"
        self.params["net_resolution"] = "-1x128"

        if "--face" in args[1]:

            self.params["face"] = True
            self.params["face_detector"] = 1
            self.params["body"] = 0
            
        # maybe shift

        if "--hand" in args[1]:
            self.params["hand"] = True
            self.params["hand_detector"] = 0

    
      
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()


        self.face_rectangles = [
        op.Rectangle(330.119385, 277.532715, 48.717274, 48.717274),
        op.Rectangle(24.036991, 267.918793, 65.175171, 65.175171),
        op.Rectangle(151.803436, 32.477852, 108.295761, 108.295761),
        ]
        
        self.hand_rectangles = [
        # Left/Right hands person 0
        [
        op.Rectangle(320.035889, 377.675049, 69.300949, 69.300949),
        op.Rectangle(0., 0., 0., 0.),
        ],
        # Left/Right hands person 1
        [
        op.Rectangle(80.155792, 407.673492, 80.812706, 80.812706),
        op.Rectangle(46.449715, 404.559753, 98.898178, 98.898178),
        ],
        # Left/Right hands person 2
        [
        op.Rectangle(185.692673, 303.112244, 157.587555, 157.587555),
        op.Rectangle(88.984360, 268.866547, 117.818230, 117.818230),
        ]
        ]

    def body_to_image(self, image):
        datum = op.Datum()
        datum.cvInputData = image

        datum.handRectangles = self.hand_rectangles

        datum.faceRectangles = self.face_rectangles

        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        
        return datum
    

class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        

        # Create publishers for face, body, and hand keypoints
        self.face_keypoints_publisher = self.create_publisher(String, '/vision/face_keypoints', 10)
        self.body_keypoints_publisher = self.create_publisher(String, '/vision/body_keypoints', 10)
        self.hand_keypoints_publisher = self.create_publisher(String, '/vision/hand_keypoints', 10)


        parser = argparse.ArgumentParser()
        self.args = parser.parse_known_args()

        self.openpose_wrapper = OpenWrapper(self.args)

        # To do: Publish messages to a topic: face for --face, body and hand for --hand 

   

    def run_api(self):
        
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


        while cap.isOpened():
            
            ret, frame = cap.read()
            if not ret:
                break

            datum = self.openpose_wrapper.body_to_image(frame)




            if "--face" in self.args[1]:
                face_msg = String()
                face_msg.data = str(datum.faceKeypoints)
                self.face_keypoints_publisher.publish(face_msg)

                """
                print("Face keypoints: \n" + str(datum.faceKeypoints))
                """

            else:
                if "--hand" in self.args[1]:

                    left_hand_msg = String()
                    left_hand_msg.data = str(datum.handKeypoints[0])

                    right_hand_msg = String()
                    right_hand_msg.data = str(datum.handKeypoints[1])

                    self.hand_keypoints_publisher.publish(left_hand_msg)
                    self.hand_keypoints_publisher.publish(right_hand_msg)


                    """
                    print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
                    print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
                    """

                body_msg = String()
                body_msg.data = str(datum.poseKeypoints)
                self.body_keypoints_publisher.publish(body_msg)

                """
                print("Body keypoints: \n" + str(datum.poseKeypoints))
                """


            # Perform any processing on the frame here if needed
            cv2.imshow("OpenPose", datum.cvOutputData)

            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                break

        cap.release()
        cv2.destroyAllWindows()
    

        
    
def main(args=None):
    rclpy.init(args=args)
    keypoint_node = KeypointNode()
        
    keypoint_node.run_api()
    #rclpy.spin(keypoint_node) # This node is going to be kept alive, until you actively kill it
    keypoint_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()