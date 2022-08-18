#!/bin/bash -e

usage() { echo "Usage: $0 SENSOR PATH_TO_VOCABULARY PATH_TO_SETTINGS GUI [--ros-args ...]" 1>&2; exit 1; }

[ $# -lt 4 ] && usage

OS3_DIR="$( dirname -- "$0"; )"
echo $OS3_DIR
: "${ROS_DISTRO:=foxy}"
: "${ROS_WS:=$OS3_DIR/ros_ws}"
: "${ORB_SLAM3_DIR:=$(pwd)/ORB_SLAM3}"

[[ -f /opt/shade/setup.sh ]] && source /opt/shade/setup.sh
source /opt/ros/$ROS_DISTRO/setup.bash

source $ROS_WS/install/setup.bash

export LD_LIBRARY_PATH="$PANGOLIN_DIR/build:$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM3_DIR/Thirdparty/g2o/lib:$ORB_SLAM3_DIR/Thirdparty/Sophus/lib:$ORB_SLAM3_DIR/lib:$LD_LIBRARY_PATH"

ros2 run ros2_orbslam3 $@
