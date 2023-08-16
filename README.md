# ROS2 Interface for the UMI-RTX Arm

### Autors
* GARDE Guillaume - <guillaume.garde@ensta-bretagne.org> (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome)
* MASSA Théo - <theo.massa@ensta-bretagne.org> (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome)

This project was made during an internship at the [_University of Amsterdam_](https://www.uva.nl/en) under the supervision of Arnoud Visser PhD - <A.Visser@uva.nl>

### Description
![The UMI-RTX robotic arm](Media/UMI-RTX-photo.png)

This repository provides tools to set up a ROS 2 interface for controlling the UMI-RTX robotic arm.
The mission of this project is to make the arm detect a target (we chose a banana plush) via computer vision and then move to grab it and lift it.
The different nodes of the ROS architecture correspond to:
* inverse kinematics
* arm control 
* computer vision
* simulation
* custom Graphic User Interface (GUI)

The computer vision part is split into two versions. One using only _OpenCV_, which is still in progress and therefore not used; the other using _OpenCV_ and [Stereolabs](https://www.stereolabs.com/)' Software Development Kit (SDK).

A vast majority of this project's code is in C++.

### Documentation
A documentation of the code can be found in [/ROS_ws/doc/](/ROS_ws/doc/). To open the doc in html, just launch *index.html*.

### Configuration
This project is built and tested with **Ubuntu 20.04** and **ROS2 Foxy**. In case these settings are not supported, see [this part](#docker).


Used material in this project:
* A UMI-RTX robotic arm

  ![UMI-RTX robotic arm](Media/UMI-RTX-Arm.png)

* A Stereolabs [_ZED Mini_](https://www.stereolabs.com/zed-mini/) stereo camera and its cable
    
  ![The ZED Mini camera](Media/ZEDM.png)

* A standard banana plush

  ![The banana plush](Media/Banana.jpg)

* An Intel Core i9 processor and a Nvidia GPU (Geforce **RTX**)

**Note**: The ZED Mini cable needs to be plugged into the device with its incurved arrows on the same side as the lenses.

### Requirements
To use the SDK, a powerful [_Nvidia_](https://www.nvidia.com/fr-fr/) Graphics Processing Unit (GPU) is required. See [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#system-requirements) for a compatibility check and installation.

If you plan not to use [Docker](#docker), you need to download and install the SDK as well as Nvidia drivers.
See [here](https://www.stereolabs.com/docs/installation/linux/) to install the SDK on linux. If you followed all the steps, the Nvidia drivers should be installed.

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

#### Computer vision
They are two computer vision nodes

#### Simulation
In order to start only the simulation, just place yourself in LAB42_RTX_control and do:

If you are on bash:

    source ROS_ws/install/setup.bash
If you are on zsh:

    source ROS_ws/install/setup.zsh

and then :

    ros2 launch ros_interface_umi_rtx simu.launch.py

### Docker
As told earlier, this project works under Ubuntu 20.04 and ROS2 Foxy. However, the usage of the interface is not limited to this configuration thanks to a custom Docker image that allows to use our interface with a different configuration.

#### Installation
The only requirements are to have Docker installed on your computer (see [here](https://docs.docker.com/get-docker/) to install Docker), to have a NVIDIA GPU with the necessary drivers for which the installation process is described [earlier](#requirements), and having installed the [nvidia-docker-toolkit](https://github.com/NVIDIA/nvidia-docker). 

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

    # If you want to use the arm
    ./start_arm.sh

    # If you want to launch only the simulation
    ros2 launch ros_interace_umi_rtx simu.launch.py

