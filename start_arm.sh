#!/bin/bash

current_dir=$(pwd)

sudo -i << EOF

source ~/.bashrc
cd "$current_dir"

### sourcing ROS files
if [ "$SHELL" = "usr/bin/zsh" ]; then
    source /opt/ros/foxy/setup.zsh
    source ROS_ws/install/local_setup.zsh
else
    source /opt/ros/foxy/setup.bash
    source ROS_ws/install/local_setup.bash
fi


### Launch the daemon
# Finds the USB port used for the arm, assumed that only one port is used
port_usb=$(ls /dev/ttyUSB* 2>/dev/null)
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

      # sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" \"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\" bash -c "source /opt/ros/foxy/setup.bash && ros2 launch ros_interface_umi_rtx arm.launch.py"



