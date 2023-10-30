# AI-RedBack-Vision
[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-2-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->
## Overview
The aim of this repo is to develop the visual components of the AI-RedBack project. The code should allow the robot to perceive and interpret its surroundings through cameras, enabling it to make informed decisions, through the identification of objects and human gestures.

## ROS2 Nodes
There are three ROS nodes in this repository:
|                   **Feature**                  	|        **ROS2 Node**        	|
|:----------------------------------------------:	|:---------------------------:	|
| 2D Object Detection<br>(YOLO Object Detection) 	| _yolo_detector_             	|
| 3D Object Detection<br>(Distance Estimation)   	| _realsense_depth_estimator_ 	|
| Human Posture Recognition<br>Face Tracking     	| _openpose_node_             	|

## Installation
<details>
  <summary>
    Step 1: Install Ubuntu 22.04 LTS (Jammy Jellyfish)
  </summary>
  
- Head to [Ubuntu release page](https://releases.ubuntu.com/jammy/) and download the “Desktop Image”
- Can either install via
  - Bootable USB flash drive
  - Virtual Machine
  - WSL
    
</details>

<details>
  <summary>
    Step 2: Configure ROS2 environment 
  </summary>
  
- Open the [official documentation](https://docs.ros.org/en/humble/Installation.html) of ROS Humble for installation instructions
    
</details>

<details>
  <summary>
    Step 3: Install required packages for object detection
  </summary>
  
- Please use `pip` to install the following packages
```
ultralytics >= 8.0
```
    
</details>

<details>
  <summary>
    Step 4: Install required packages for distance estimation
  </summary>
  
- Please use `pip` to install the following packages
```
pyrealsense >= 2.54
```
- Please note that `pyrealsense` **doesn't** support Linux with ARM architecture
    
</details>

<details>
  <summary>
    Step 5: Install required packages for human posture detection
  </summary>

- Please install the following packages
```
CUDA: https://developer.nvidia.com/cuda-11-7-1-download-archive
cuDNN: https://developer.nvidia.com/cudnn
Caffe prerequisites: Openpose repo -> ./scripts/ubuntu/install_deps.sh
```

- Please use `apt-get` to install the following packages
```
cmake-qt-gui
libopencv-dev
python3-dev
```

- Please use `apt` to install the following packages
```
protobuf-compiler
libgoogle-glog-dev
libboost-all-dev
libhdf5-dev
libatlas-base-dev
```

- Please use `pip` to install the following packages
```
numpy
opencv-python
```
</details>

## Usage
### Build
```
$ cd franka_ws
$ colcon build --symlink-install
$ source install/setup.sh
```

```
$ git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
$ cd openpose
$ git submodule update --init --recursive --remote
$ mkdir build
$ cd build
$ cmake-gui ..
# Click configure, tick BUILD_PYTHON flag, and click configure again. Click generate.
$ make -j`nproc`
```


### Run yolo_detector node
After successfully built, `yolo_detector` ROS2 node can be launched via the following command:
```
ros2 run object_detector yolo_detector
```
Details about `yolo_detector` can be found in [docs](./docs/yolo_object_detector.md)

<p align="center">
  <img src="./docs/images/obj_detec/yolo_detector_running.png" alt="Screenshot of yolo_detector running" width="600" />
</p>


### Run realsense_depth_estimator node
Similarly, `realsense_depth_estimator` ROS2 node can be launched via the following command:
```
ros2 run depth_estimator realsense_depth_estimator
```
Details about `realsense_depth_estimator` can be found in [docs](./docs/realsense_depth_estimator.md)

<p align="center">
  <img src="./docs/images/distance_est/depth_estimator_running.png" alt="Screenshot of realsense_depth_estimator running" width="600" />
</p>


### Run openpose
After being built, openpose can be run with the following command:  
```
./build/examples/openpose/openpose.bin 
```
Details about `openpose` can be found in [docs](./docs/openpose.md)


## Camera Mounting
As mentioned on the page [Depth Camera Mounting Analysis](./docs/camera_mounting/camera_mounting_analysis.md), a custom 3D-printed model needs to be designed and built to mount the depth camera on the end-effector of the robotic arm. We found there was [another team](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-franka-pbvs.html) working with computer vision solutions using both Franka Emika robotic arm and Intel Realsense depth camera released the STL file of the CAD model of a 3D printed part "that allows to attach an Intel RealSense D435 camera to the Panda flange".

<p align="center">
  <img src="./docs/images/model_blender.png" alt="Model file in Blender" width="600" />
</p>

The original model file was downloaded and rescaled to match the actual dimensions of the depth camera. The modified model file has been uploaded [here](./docs/camera_mounting/franka-rs-D435-camera-holder(modified).stl). The process of analysing and building the 3D printed mounting component has been documented in detail in this [PDF](./docs/camera_mounting/3D%20Printed%20Camera%20Mounting%20Part.pdf). This camera mounting model can be built using a 3D printer in the lab with the configurations specified in the [documentation](./docs/camera_mounting/3D%20Printed%20Camera%20Mounting%20Part.pdf).

<p align="center">
  <img src="./docs/images/camera_mounted.jpg" alt="Camera mounted preview" width="500" />
</p>

## Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/jackson-hu1279"><img src="https://avatars.githubusercontent.com/u/68998854?v=4?s=100" width="100px;" alt="jackson-hu1279"/><br /><sub><b>jackson-hu1279</b></sub></a><br /><a href="https://github.com/COMP90082-2023-SM2/AI-RedBack-Vision/commits?author=jackson-hu1279" title="Code">💻</a> <a href="https://github.com/COMP90082-2023-SM2/AI-RedBack-Vision/commits?author=jackson-hu1279" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Tempest371"><img src="https://avatars.githubusercontent.com/u/102500895?v=4?s=100" width="100px;" alt="Tempest371"/><br /><sub><b>Tempest371</b></sub></a><br /><a href="https://github.com/COMP90082-2023-SM2/AI-RedBack-Vision/commits?author=Tempest371" title="Code">💻</a> <a href="https://github.com/COMP90082-2023-SM2/AI-RedBack-Vision/commits?author=Tempest371" title="Documentation">📖</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!


[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
