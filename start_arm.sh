#!/bin/bash


### sourcing ROS files
if [ "$SHELL" = "usr/bin/zsh" ]; then
    source ROS_ws/install/local_setup.zsh
else
    source ROS_ws/install/local_setup.bash
fi

### Launch the daemon
cd umi-rtx/bin
sudo ./rtxd /dev/ttyUSB0


### Launch ros_interface
cd ../..
if [ -e ./umi-rtx/ports/rtx-socket ]; then
# ros2 run ros_interface_umi_rtx nodeArm
    echo "Start ROS2 interface ..."
    ros2 launch ros_interface_umi_rtx arm.launch.py
else 
    echo "rtx-socket file not found"
fi