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

#### Installation
The only requirement is to have Docker installed on your computer, of course, to have a NVIDIA GPU with the necessary drivers for which the installation process is described [earlier](### TODO), and having installed the [nvidia-docker-toolkit](https://github.com/NVIDIA/nvidia-docker). 

    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

    sudo apt-get update && sudo apt-get install -y nvidia-docker2 nvidia-container-toolkit
    sudo systemctl daemon-reload
    sudo systemctl restart docker

Once this is done, you can build the docker image with the command:

    # Place yourself in LAB42_RTX_CONTROL
    docker build -t "name" .

Be careful to replace "name" with the name you want, and everything is ready !
#### Usage
To run our image into a container, run :

    # Give the permission to use the screen
    xhost +

    # Launch the container (replace "name" with the name you chose)
    docker run --gpus all -it --privileged -e DISPLAY=$DISPLAY -v \\
                /tmp/.X11-unix:/tmp/.X11-unix --rm "name":latest

Once the container is running, the process is similar as the one descibed [here](#usage)

    cd ROS_ws/
    colcon build
    source install/setup.bash
    cd ..

    # If you want to use the real arm
    ./start_arm.sh

    # If you want to launch only the simulation
    ros2 launch ros_interace_umi_rtx simu.launch.py

