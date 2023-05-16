#!/bin/bash


### sourcing ROS files
if [ "$SHELL" = "usr/bin/zsh" ]; then
    source ROS_ws/install/local_setup.zsh
else
    source ROS_ws/install/local_setup.bash
fi


### Launch the daemon

# Finds the USB port used for the arm, assumed that only one port is used
port_usb=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -z "$port_usb" ]
then
  echo "Arm not connected"
else
  cd umi-rtx/bin
  sudo ./rtxd $port_usb

  
  ### Launch ros_interface
  cd ../..
  if [ -e ./umi-rtx/ports/rtx-socket ]; then
  # ros2 run ros_interface_umi_rtx nodeArm
      echo "Start ROS2 interface ..."
      ros2 launch ros_interface_umi_rtx arm.launch.py
  else 
      echo "rtx-socket file not found"
  fi
fi



