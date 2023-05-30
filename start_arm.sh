#!/bin/bash

current_dir=$(pwd)

sudo -i << EOF

cd "$current_dir"

### sourcing ROS files
source /opt/ros/foxy/setup.bash
source ROS_ws/install/setup.bash

### Launch the daemon
# Finds the USB port used for the arm, assumed that only one port is used
if [ -z "$(ls /dev/ttyUSB* 2>/dev/null)" ]
then
  echo "Arm not connected"
else
  cd umi-rtx/bin
  sudo ./rtxd $(ls /dev/ttyUSB* 2>/dev/null)

  echo "------------------"
  ### Launch ros_interface
  cd ../..
  if [ -e ./umi-rtx/ports/rtx-socket ]; then
  # ros2 run ros_interface_umi_rtx nodeArm
      echo "Start ROS2 interface ..."

      ros2 launch ros_interface_umi_rtx arm.launch.py

      ##### REMOVE NEXT LINE IF YOU WANT LOGS #####
      echo "Delete logs...."
      rm -rf logs/*
      cd
      rm -rf .ros/log/*
      #####

      echo "------------------"
      echo "Reboot ROS2 daemon"
      ros2 daemon stop
      ros2 daemon start
  else 
      echo "umi-rtx/ports/rtx-socket file not found"
  fi
fi

exit
EOF
