#!/bin/bash -e

docker build . -t shaderobotics/orbslam3-ros2
docker run -it --rm \
    -v /dev/shm:/dev/shm \
    --env="DISPLAY" \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    --net=host \
    shaderobotics/orbslam3-ros2 $@
