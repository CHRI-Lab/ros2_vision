# AI-RedBack-Vision
[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-2-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->
## Overview
The aim of this repo is to develop the visual components of the AI-RedBack project. The code should allow the robot to perceive and interpret its surroundings through cameras, enabling it to make informed decisions, through the identification of objects and human gestures.

## ROS2 Nodes
There are two ROS nodes in this repository:
- 3D object detection
- Human posture detection (hand gesture, gaze)

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
    Step 4: Install required packages for human posture detection
  </summary>
  
- Please use `pip` to install the following packages
```
ultralytics >= 8.0
```
    
</details>



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