#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import subprocess
import cv2
import os
from sys import platform
import argparse

sys.path.append('../openpose/build/python')
from openpose import pyopenpose as op

class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        #self.get_logger().info("Hello from ROS2")

    def display(datums):
        datum = datums[0]
        cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
        key = cv2.waitKey(1)
        return (key == 27)


    def printKeypoints(datums):
        datum = datums[0]
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        print("Face keypoints: \n" + str(datum.faceKeypoints))
        print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))

    def run_api(self):
        try:
            
            # Flags
            parser = argparse.ArgumentParser()
            parser.add_argument("--video", default="../openpose/examples/media/video_1.mp4", help="Process a video example")
            parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
            args = parser.parse_known_args()

            # Custom Params (refer to include/openpose/flags.hpp for more parameters)
            params = dict()
            params["model_folder"] = "../openpose/models/"

            # Add others in path?
            for i in range(0, len(args[1])):
                curr_item = args[1][i]
                if i != len(args[1])-1: next_item = args[1][i+1]
                else: next_item = "1"
                if "--" in curr_item and "--" in next_item:
                    key = curr_item.replace('-','')
                    if key not in params:  params[key] = "1"
                elif "--" in curr_item and "--" not in next_item:
                    key = curr_item.replace('-','')
                    if key not in params: params[key] = next_item

            # Construct it from system arguments
            # op.init_argv(args[1])
            # oppython = op.OpenposePython()

            # Starting OpenPose
            opWrapper = op.WrapperPython()
            opWrapper.configure(params)
            opWrapper.start()

            # Process Image
            datum = op.Datum()
            imageToProcess = cv2.imread(args[0].image_path)
            datum.cvInputData = imageToProcess
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))

            # Display Image
            print("Body keypoints: \n" + str(datum.poseKeypoints))
            cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
            cv2.waitKey(0)
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