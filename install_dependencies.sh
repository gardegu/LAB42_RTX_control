#!/bin/bash

sudo apt update

# Pinocchio for the inverse kinematics
sudo apt install ros-foxy-pinocchio -y
sudo apt install ros-foxy-xacro -y

# Verification that an important file is present, because it wasn't in my case
cd /opt/ros/foxy/include/rviz_common
if ! [ -e ./tool_manager.hpp ]
then 
    sudo wget "https://raw.githubusercontent.com/ros2/rviz/foxy/rviz_common/src/rviz_common/tool_manager.hpp"
fi

