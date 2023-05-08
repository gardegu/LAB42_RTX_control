#!/bin/bash

cd umi-rtx/bin 
sudo ./rtxd /dev/ttyUSB0

ros2 launch ros_interface_umi_rtx arm.launch.py