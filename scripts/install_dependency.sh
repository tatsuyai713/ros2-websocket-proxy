#!/bin/bash

set -e

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 is not sourced. Please source your ROS 2 installation before running this script."
    exit 1
fi

sudo apt update
sudo apt install -y \
    libasio-dev \
    libboost-dev \
    libwebsocketpp-dev \
    libyaml-cpp-dev

sudo apt install -y \
    ros-"$ROS_DISTRO"-rclcpp

echo "All dependencies installed successfully."