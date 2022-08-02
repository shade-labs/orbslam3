# ORBSLAM3-ROS2 Wrapper

This is a ROS2 wrapper for the Orbslam3 algorithm. Currently we only have distributions for ROS2 Foxy.

# Installation Guide

## Using Docker Pull

1. `docker pull shaderobotics/orbslam2-ros2:latest`
2. Follow the run commands in the usage section below

# Build Natively

1. Directly follow the steps in the `Dockerfile`

# Usage

1. Use an Ubuntu host or ensure that an X11 server is installed (for the UI)
2. Run `xhost + local:docker`
3. Run on the D435i `docker run -it -v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw"  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e MALLOC_CHECK_=2 shaderobotics/orbslam3:foxy rgbd /usr/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /usr/src/ORB_SLAM3/Examples/RGB-D/RealSense_D435i.yaml true`