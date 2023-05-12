#!/bin/bash
# Example script for stereo capture

J7ROS_DIR=${HOME}/j7ros_home
PYTHON=python3

# stereo capture
${PYTHON} stereo_test.py \
    -p ${J7ROS_DIR}/ros_ws/src/robotics_sdk/tools/stereo_camera/calibration \
    -m HD \
    -d 2