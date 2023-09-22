# Openpose

## Openpose link: https://github.com/CMU-Perceptual-Computing-Lab/openpose

## Running Openpose
./build/examples/openpose/openpose.bin

## Openpose Flags
### Additional keypoints:
--face: Adds facial keypoints  
--hand: Adds hand keypoints  


### Input:
--video <path_to_video>: Runs openpose on a video  
--image_dir <path_to_folder>: Runs openpose on all images within a folder  
--camera <camera_number>: Selects which camera to run, if --video and --image_dir flags are not used  

### Output:
--write_video <path_to_video>: Records video to the path specified  
--write_images <path_to_folder>: Records images in the folder path specified  


### Examples:

<p align="center">
  <img src="./images/openpose_1.png" alt="Hand and face keypoint detection" width="600" />
  <figcaption>Openpose hand and face keypoint detection</figcaption>
</p>



<p align="center">
  <img src="./images/openpose_1.png" alt="Finger keypoint estimation" width="600" />
  <figcaption>Openpose estimation of finger keypoints</figcaption>
</p>

<p align="center">
  <img src="./images/openpose_1.png" alt="Sideways face keypoint detection" width="600" />
  <figcaption>Openpose face keypoint detection (sideview)</figcaption>
</p>
