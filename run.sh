#!/bin/bash

source /opt/shade/setup.sh
source /opt/ros/$ROS_DISTRO/setup.bash
source /usr/src/ros_ws/install/setup.bash
export LD_LIBRARY_PATH="$PANGOLIN_DIR/build:$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM3_DIR/Thirdparty/g2o/lib:$ORB_SLAM3_DIR/Thirdparty/Sophus/lib:$ORB_SLAM3_DIR/lib:$LD_LIBRARY_PATH"

ros2 run ros2_orbslam3 rgbd $ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt $ORB_SLAM3_DIR/Examples/RGB-D/RealSense_D435i.yaml --ros-args -r /camera/rgb:=/camera/color/image_raw -r /camera/depth:=/camera/depth/image_rect_raw

