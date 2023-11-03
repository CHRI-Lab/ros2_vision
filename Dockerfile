FROM python:3.11.6
# Set a working directory
WORKDIR /workspace

# Install Python packages
RUN pip3 install --no-cache-dir \
    ultralytics>=8.0 \
    pyrealsense2>=2.54 \
    numpy \
    opencv-python


# Update and init submodules
RUN git submodule update --init --recursive --remote

# Create a build directory
RUN mkdir build && cd build


# Clone OpenPose and build it
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose /openpose
WORKDIR /openpose
RUN git submodule update --init --recursive --remote


# Set the default command to run when starting the container
RUN echo "source /franka_ws/install/setup.bash" >> ~/.bashrc

# Expose any ports the application uses
EXPOSE 8000

# Set environment variables
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics


# Set up the environment
RUN echo "source /franka_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint commands for the object_detector and depth_estimator nodes
# These commands won't run until a container based on this image is started

ENTRYPOINT ["/bin/bash", "-c", "source install/setup.bash && \
            ros2 run object_detector yolo_detector & \
            ros2 run depth_estimator realsense_depth_estimator & \
            /openpose/build/examples/openpose/openpose.bin"]
