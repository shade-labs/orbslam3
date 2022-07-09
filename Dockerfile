ARG ROS_DISTRO=foxy

FROM shaderobotics/ros:$ROS_DISTRO

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install deps
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-vision-opencv \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libglew-dev \
    libeigen3-dev \
    libboost-serialization-dev \
    libboost-python-dev \
    cmake \
    g++ \
    ninja-build \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libdc1394-22-dev \
    libraw1394-dev \
    libopenni-dev \
    python3.9-dev \
    python3-distutils && \
    rm -rf /var/lib/apt/lists/

# pangolin
WORKDIR /usr/src/pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build && \
    cmake --build build && \
    cd build && \
    make install
ENV PANGOLIN_DIR=/usr/src/pangolin/Pangolin

# build orbslam3
WORKDIR /usr/src/ORB_SLAM3
COPY ./ORB_SLAM3 .
RUN ./build.sh
ENV ORB_SLAM3_DIR=/usr/src/ORB_SLAM3

# build ros nodes
WORKDIR /usr/src/ros_ws
COPY ./ros2-ORB_SLAM3 ./src/ros2-ORB_SLAM3/
RUN bash -c 'source /opt/ros/$ROS_DISTRO/setup.sh && colcon build'

WORKDIR /root

COPY ./run.sh ./run.sh

RUN chmod +x ./run.sh

ENV ROS_DISTRO=$ROS_DISTRO

ENTRYPOINT ["/root/run.sh"]
