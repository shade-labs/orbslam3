#!/bin/bash


: "${ROS_DISTRO:=foxy}"
: "${ROS_WS:=$(pwd)/ros_ws}"
: "${ORB_SLAM3_DIR:=$(pwd)/ORB_SLAM3}"

[[ -f /opt/shade/setup.sh ]] && source /opt/shade/setup.sh
source /opt/ros/$ROS_DISTRO/setup.bash


if [[ "$1" == "-b" ]]; then
    shift

    DIR="$(pwd)"
    echo Building...

    cd $ORB_SLAM3_DIR
    ./build.sh

    cd $ROS_WS
    colcon build

    cd "$DIR"
fi

source $ROS_WS/install/setup.bash

export LD_LIBRARY_PATH="$PANGOLIN_DIR/build:$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM3_DIR/Thirdparty/g2o/lib:$ORB_SLAM3_DIR/Thirdparty/Sophus/lib:$ORB_SLAM3_DIR/lib:$LD_LIBRARY_PATH"

ros2 run ros2_orbslam3 slam $@

