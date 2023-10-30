# Posture Detection


## Overview
The openpose_node node under the openpose_controller package will be able to detect the posture of the human body. It is also able to detect the keypoints of a person's hands with the --hand flag, and facial keypoints with the --face flag. 

## Prerequisites
The installation of CUDA (11.7) and cuDNN (8.5.0) are required for gpu acceleration of openpose. All the openpose prerequisites can be found here: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/1_prerequisites.md. 

GStreamer and opencv also needs to be installed. GStreamer instructions are found here: https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c.
For opencv, run the commands found in the prerequisite page of openpose above, except for the command: 
```
$ sudo pip install numpy opencv-python
```
This will build a version of opencv that is nto compatible with gstreamer. Instead run:
```
$ sudo apt-get install python3-opencv
```
## Usage
1. Setting up the Workspace  
    - Navigate to the ROS2 workspace
      ```shell
      cd openpose_ws
      ``` 
    - Build packages
      ```shell
      colcon build --symlink-install 
      ```
    - Source the setup file
      ```shell 
      source install/setup.bash
      ```
2. Launch the `openpose_node` node 
    - After successfully built, `openpose_node` ROS2 node can be launched via the following command: 
      ```shell
      ros2 run openpose_controller openpose_node 
      ```
3. Check detected objects 
    - A window will show the camera with keypoints attached 
    - The keypoint values can also be inspected through the topics:
      - /vision/face_keypoints 
      - /vision/hand_keypoints 
      - /vision/body_keypoints
    
    - The topics can be run with the commands: 
      ```shell
      ros2 topic echo /vision/<face/hand/body>_keypoints
      ```

## Published Topic Information
- Individual keypoints within the message will include:
  - x coordinate
  - y coordinate
  - confidence score
- Format for face and hand keypoints can be found here: https://github.com/ArtificialShane/OpenPose/blob/master/doc/output.md
- Format for body keypoints can be found underneath "Keypoint Ordering in C++/Python": https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_02_output.html

## Release Notes

### Features
- Integrate Openpose functionality within a ROS2 node
- Display keypoints visually on top of camera feed
- Publish keypoints towards ROS2 topics

### Known Issues
- cudnn version currently has an issue, and is not used:
  ```shell
  status == CUDNN_STATUS_SUCCESS (3 vs. 0) CUDNN_STATUS_BAD_PARAM
  ```
### Notes
- Additional instructions if using WSL2:
    -  Need to install usbipd-win and rebuild the linux kernel in order to connect to the camera
        - Link: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
    -  Find camera id and attach with commands in powershell
       ```shell
       usbipd wsl list
       usbipd wsl attach --busid <camera id>
       ```
    - Confirm camera attachment and give permissions to camera with:
      ```shell
      lsusb 
      ls -al /dev/video*
      sudo chmod 777 /dev/video0
      ```
    - Change the number 0 to the appropriate index
  
  
## Future Improvements
1. Attempt to separate the openpose wrapper and the camera into two separate nodes, and have communication using publishers and subscribers
2. Attempt to publish the messages in an ndarray format, rather than the current string format





## Examples:

<p align="center">
  <img src="./images/openpose_1.png" alt="Hand and face keypoint detection" width="600" />
  <figcaption>Openpose hand and face keypoint detection</figcaption>
</p>

<p align="center">
  <img src="./images/openpose_2.png" alt="Finger keypoint estimation" width="600" />
  <figcaption>Openpose estimation of finger keypoints</figcaption>
</p>

<p align="center">
  <img src="./images/openpose_3.png" alt="Sideways face keypoint detection" width="600" />
  <figcaption>Openpose face keypoint detection (sideview)</figcaption>
</p>
