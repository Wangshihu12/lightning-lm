#!/usr/bin/env bash

set -e

sudo apt update
sudo apt install -y \
    libopencv-dev \
    libpcl-dev \
    pcl-tools \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-ros \
    ros-noetic-rosbag
