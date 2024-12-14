# Base Docker image with ROS Noetic
FROM osrf/ros:noetic-desktop-full

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT=2022-03-22

# NVIDIA runtime (optional for GPU support)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics

# Set timezone
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install essential dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    git \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    python3-catkin-tools \
    python3-matplotlib \
    gfortran \
    autoconf \
    coinor-libipopt-dev \
    libgfortran5 \
    curl \
    libopenmpi-dev \
    apt-utils \
    software-properties-common \
    build-essential \
    libssl-dev \
    wget \
    libtool \
    cmake \
    python3-pip \
    tzdata \
    x11-apps \
    gazebo11

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Setup ROS workspace
ENV SUPPORT_WS=/root/support_files
ENV UNITREE_WS=/root/unitree_ws
RUN mkdir -p $SUPPORT_WS $UNITREE_WS/src
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init;"

# Clone and build Unitree SDKs
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd lcm && git checkout tags/v1.4.0 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    cd .. && rm -rf build # clean build directory

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd unitree_legged_sdk && git checkout v3.2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && cd .. && rm -rf build # clean build directory

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/aliengo_sdk.git && \
    cd aliengo_sdk && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && cd .. && rm -rf build # clean build directory

# Install ROS Noetic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-control \
    ros-noetic-gazebo-ros \
    ros-noetic-joy \
    ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher \
    ros-noetic-plotjuggler-ros

# Clone and build Unitree ROS packages
WORKDIR $UNITREE_WS/src
RUN git clone --branch master https://github.com/ShuoYangRobotics/unitree_ros.git

# Remove problematic packages (if necessary)
RUN rm -rf $UNITREE_WS/src/unitree_ros/unitree_legged_real

# Fix Gazebo compatibility in draw_force_plugin.cc
RUN sed -i 's|#include <gazebo/common/Color.hh>|#include <ignition/math/Color.hh>|' \
    $UNITREE_WS/src/unitree_ros/unitree_gazebo/plugin/draw_force_plugin.cc && \
    sed -i 's|common::Color|ignition::math::Color|g' \
    $UNITREE_WS/src/unitree_ros/unitree_gazebo/plugin/draw_force_plugin.cc

# Clone the Go1-QP-MPC-Controller from GitHub
WORKDIR /root/Go1_ctrl_ws/src
RUN git clone https://github.com/boihhs/Go1-QP-MPC-Controller-Real.git Go1_Ctrl

# Build workspace packages step-by-step
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin build unitree_legged_msgs && \
    source ${UNITREE_WS}/devel/setup.bash && \
    catkin build"

# Update CMake to the latest version
RUN apt-get update && apt-get install -y wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.22.3/cmake-3.22.3-linux-x86_64.sh && \
    chmod +x cmake-3.22.3-linux-x86_64.sh && \
    ./cmake-3.22.3-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.22.3-linux-x86_64.sh

# Install OSQP and osqp-eigen
WORKDIR $SUPPORT_WS
RUN git clone -b release-0.6.3 --recursive https://github.com/oxfordcontrol/osqp && \
    cd osqp && mkdir build && cd build && \
    cmake .. -DBUILD_SHARED_LIBS=ON && \
    make -j$(nproc) && \
    make install && \
    cp /usr/local/include/osqp/* /usr/local/include

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Install and configure SSH server
 RUN apt-get update && apt-get install -y openssh-server && \
    echo 'root:password' | chpasswd && \
     sed -i 's/#Port 22/Port 2233/' /etc/ssh/sshd_config && \
      sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    service ssh restart

# Update environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ${UNITREE_WS}/devel/setup.bash" >> ~/.bashrc

# To use rosparam load yaml files
RUN pip3 install pyyaml

# Default environment settings
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc && \
    echo "export ROS_IP=localhost" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
