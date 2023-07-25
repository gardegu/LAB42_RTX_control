#!/bin/bash


# Pinocchio for the inverse kinematics
sudo apt install ros-foxy-pinocchio
sudo apt install ros-foxy-xacro

echo export PATH=/opt/openrobots/bin:$PATH >> ~/.bashrc
echo export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH >> ~/.bashrc
echo export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH >> ~/.bashrc
echo export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH >> ~/.bashrc
echo export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH >> ~/.bashrc

cd /opt/ros/foxy/include/rviz_common
if [-f tool_manager.hpp ]
then 
    rm tool_manager.hpp
fi
wget "https://raw.githubusercontent.com/ros2/rviz/foxy/rviz_common/src/rviz_common/tool_manager.hpp"

source ~/.bashrc

