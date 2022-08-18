#!/bin/bash -e

: "${ROS_WS:=$(pwd)/ros_ws}"
: "${ORB_SLAM3_DIR:=$(pwd)/ORB_SLAM3}"

cd $ORB_SLAM3_DIR
./build.sh

cd $ROS_WS
colcon build
