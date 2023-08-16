# ROS2 Interface for the UMI-RTX Arm

### Autors
* GARDE Guillaume guillaume.garde@ensta-bretagne.org (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)
* MASSA Théo theo.massa@ensta-bretagne.org (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)

### Description
This repository has for purpose to set up a ROS2 interface in order to use the UMI-RTX Arm.

### Configuration
This project is built and tested with **Ubuntu 20.04** and **ROS2 Foxy**.

### Usage
#### Real arm
To use the interface, all you have to do is :

    git clone https://github.com/gardegu/LAB42_RTX_control
    cd LAB42_RTX_control
    ## install the required dependencies
    ./install_dependencies.sh
    cd ROS_ws
    ## Build the package
    colcon build
    cd..
    ## Launch the arm
    ./start_arm

The custom GUI will launch in the same time as the arm

If you want to monitor the nodes and topics or interact with them, all you have to do is logging as root:

    sudo -i

And then sourcing ROS:

    source /opt/ros/foxy/setup.bash

#### Simulation
In order to start only the simulation, just place yourself in LAB42_RTX_control and do:

If you are on bash:

    source ROS_ws/install/setup.bash
If you are on zsh:

    source ROS_ws/install/setup.zsh

and then :

    ros2 launch ros_interface_umi_rtx simu.launch.py

### Docker
As told earlier, this project works under Ubuntu 20.04 and ROS2 Foxy. However, the usage of the interface is not limited only to this configuration thanks to a custom Docker image, that allow to use our interface with a different configuration.

The only requirement is to have Docker installed on your computer, of course, to have a NVIDIA GPU with the necessary drivers for which the installation process is described earlier, and having downloaded a Stereolab custom image.


