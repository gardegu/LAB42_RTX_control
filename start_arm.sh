#!/bin/bash

cd umi-rtx/bin 
sudo ./rtxd /dev/ttyUSB0

ros2 run ros_interface_umi_rtx nodeArm

# ros2 launch ros_interface_umi_rtx arm.launch.py