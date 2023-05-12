#!/bin/bash
# Example script for stereo capture

J7ROS_DIR=${HOME}/j7ros_home
PYTHON=python3

# stereo capture
${PYTHON} stereo_calibration.py \
    -p ${J7ROS_DIR}/ros_ws/src/robotics_sdk/tools/stereo_camera/calibration \
    -o ${J7ROS_DIR}/ros_ws/src/robotics_sdk/ros1/drivers/zed_capture/config \
    -m HD \
    -c 9 -r 7 -s 25
