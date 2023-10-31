#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import cv2
from sys import platform
import argparse
from std_msgs.msg import String


sys.path.append('../openpose/build/python')
from openpose import pyopenpose as op


class OpenWrapper(Node):

    def __init__(self, args):
        
        self.params = dict()
        self.params["model_folder"] = "../openpose/models/"

        #Reduces resolution to increase speed
        self.params["net_resolution"] = "-1x128" 

        if "--face" in args[1]:

            self.params["face"] = True
            self.params["face_detector"] = 1

            # Turns off the body keypoints 
            self.params["body"] = 0
            

        if "--hand" in args[1]:
            self.params["hand"] = True
            self.params["hand_detector"] = 0

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()



    def body_to_image(self, image):
        datum = op.Datum()
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        
        return datum
    

class KeypointNode(Node):
    def __init__(self):
        super().__init__("keypoint_node")
        

        # Create publishers for face, body, and hand keypoints
        self.face_keypoints_publisher = self.create_publisher(String, '/vision/face_keypoints', 10)
        self.hand_keypoints_publisher = self.create_publisher(String, '/vision/hand_keypoints', 10)
        self.body_keypoints_publisher = self.create_publisher(String, '/vision/body_keypoints', 10)

        parser = argparse.ArgumentParser()
        self.args = parser.parse_known_args()

        self.openpose_wrapper = OpenWrapper(self.args)

   

    def run_api(self):
        

        # Sets camera to use V4L2 and MJPG 
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

                
                # Command below if want to print to terminal
                #print("Face keypoints: \n" + str(datum.faceKeypoints))
                

            else:
                if "--hand" in self.args[1]:

                    left_hand_msg = String()
                    left_hand_msg.data = "Left: " + str(datum.handKeypoints[0])

                    right_hand_msg = String()
                    right_hand_msg.data = "Right: " + str(datum.handKeypoints[1])

                    self.hand_keypoints_publisher.publish(left_hand_msg)
                    self.hand_keypoints_publisher.publish(right_hand_msg)


                    # Commands below if want to print to terminal
                    #print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
                    #print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
                
                    
                body_msg = String()
                body_msg.data = str(datum.poseKeypoints)
                self.body_keypoints_publisher.publish(body_msg)

                # Command below if want to print to terminal
                #print("Body keypoints: \n" + str(datum.poseKeypoints))
                

            # Perform any processing on the frame here if needed
            cv2.imshow("OpenPose", datum.cvOutputData)

            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                break

        cap.release()
        cv2.destroyAllWindows()
    

def main(args=None):
    rclpy.init(args=args)

    try:
        keypoint_node = KeypointNode()
        keypoint_node.run_api() 
    except Exception as e:
        # Handle the exception (print an error message, log the exception, etc.)
        print(f"An exception occurred: {e}")
    finally:
        keypoint_node.destroy_node()
        rclpy.shutdown()


    
    

if __name__ == '__main__':
    main()