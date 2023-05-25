#!/bin/bash


# Pinocchio for the inverse kinematics
sudo apt install -qqy lsb-release gnupg2 curl
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt update -y
sudo apt install -qqy robotpkg-py38-pinocchio

sudo apt install ros-foxy-xacro

echo export PATH=/opt/openrobots/bin:$PATH >> ~/.bashrc
echo export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH >> ~/.bashrc
echo export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH >> ~/.bashrc
echo export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH >> ~/.bashrc
echo export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH >> ~/.bashrc

source ~/.bashrc

