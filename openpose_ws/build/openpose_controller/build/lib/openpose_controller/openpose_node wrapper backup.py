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


class OpenWrapper(Node):

    def __init__(self):
        self.params = dict()
        self.params["model_folder"] = "../openpose/models/"

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()

    def body_to_image(self, image):
        datum = op.Datum()
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        
        return datum


class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        self.openpose_wrapper = OpenWrapper()
    
    
        
        
    
def main(args=None):
    rclpy.init(args=args)
    keypoint_node = KeypointNode()

    try:
        rclpy.spin(keypoint_node)

    except KeyboardInterrupt:
        pass

    finally:
        keypoint_node.destroy_node()
        rclpy.shutdown
        

    #keypoint_node.run_api()
    # # This node is going to be kept alive, until you actively kill it
    
    

if __name__ == '__main__':
    main()