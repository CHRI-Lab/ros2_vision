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

sys.path.append('../openpose/build/python')
from openpose import pyopenpose as op



class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
   
        

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
            
            parser = argparse.ArgumentParser()
            parser.add_argument("--no-display", action="store_true", help="Disable display.")
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
            #opWrapper = op.WrapperPython(op.ThreadManagerMode.AsynchronousOut)

            opWrapper = op.WrapperPython()
            opWrapper.configure(params)
            opWrapper.start()

                        
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


            if not cap.isOpened():
                print("Error: Could not open the camera or video file.")
            else:
                print("GStreamer is working with OpenCV!")

            print("CAMERA NOT OPENED YET")

            userWantsToExit = False
            while not userWantsToExit:
                ret, frame = cap.read()


                if not ret:
                    self.get_logger().error("Error reading frame from the camera.")
                    break

                datumProcessed = op.VectorDatum()
                opWrapper.waitAndEmplace(datumProcessed, frame)
                
                if not args[0].no_display:
                    userWantsToExit = self.display(datumProcessed)
                self.printKeypoints(datumProcessed)
           
            """
# Look at testing_3.py to fix it
            # Main loop
            userWantsToExit = False
            while not userWantsToExit:


                datumProcessed = op.VectorDatum()
                # Pop frame
                datumProcessed = op.VectorDatum()
                if opWrapper.waitAndPop(datumProcessed):
                    if not args[0].no_display:
                        # Display image
                        userWantsToExit = self.display(datumProcessed)
                    self.printKeypoints(datumProcessed)
                else:
                    break
            """
        except Exception as e:
            self.get_logger().error(f'Error executing external script: {str(e)}')
        
        
    
def main(args=None):
    rclpy.init(args=args)
    keypoint_node = KeypointNode()
        
    keypoint_node.run_api()
    #rclpy.spin(keypoint_node) # This node is going to be kept alive, until you actively kill it
    keypoint_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()