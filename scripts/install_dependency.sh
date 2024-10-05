#!/bin/bash

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 is not sourced. Please source your ROS 2 installation before running this script."
    exit 1
fi

sudo apt update
sudo apt install -y libasio-dev libwebsocketpp-dev
sudo apt install -y ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclcpp-components
sudo apt install -y libwebsocketpp-dev nlohmann-json3-dev
sudo apt install -y ros-$ROS_DISTRO-rosbridge-server
if [ $? -eq 0 ]; then
    echo "All dependencies installed successfully."
else
    echo "Error occurred during the installation of dependencies."
    exit 1
fi