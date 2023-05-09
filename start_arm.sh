#!/bin/bash

if [ "$SHELL" = "usr/bin/zsh" ]; then
    source ROS_ws/install/local_setup.zsh
else
    source ROS_ws/install/local_setup.bash
fi

cd umi-rtx/bin; sudo ./rtxd /dev/ttyUSB2

cd ../../ROS_ws/
# ros2 run ros_interface_umi_rtx nodeArm

ros2 launch ros_interface_umi_rtx arm.launch.py