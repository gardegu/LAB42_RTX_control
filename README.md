# ROS2 Interface for the UMI-RTX Arm

### Autors
* GARDE Guillaume guillaume.garde@ensta-bretagne.org (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)
* MASSA Théo theo.massa@ensta-bretagne.org (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)

### Description
This repository has for purpose to set up a ROS2 interface in order to use the UMI-RTX Arm.

### Configuration
This project is built and tested with Ubuntu 20.04 and ROS2 Foxy.

### Usage
To use the interface, all you have to do is :

    git clone https://github.com/gardegu/LAB42_RTX_control
    cd LAB42_RTX_control
    ./start_arm

If you want to monitor the nodes and topics or interact with them, all you have to do is logging as root:

    sudo -i

And then sourcing ROS:

    source /opt/ros/foxy/setup.bash

