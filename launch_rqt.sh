#!/bin/bash

current_dir=$(pwd)

sudo -i << EOF

cd "$current_dir"

### sourcing ROS files
source /opt/ros/foxy/setup.bash

rqt