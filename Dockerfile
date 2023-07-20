# Downloading the Ubuntu 20.04 Docker image
FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get -y install sudo


ENV SIMPLE_USER=newubu
ENV UID=1000
ENV GID=1000
RUN groupadd -g $GID $SIMPLE_USER
RUN useradd -rm -d /home/$SIMPLE_USER -s /bin/bash -g $SIMPLE_USER -G sudo -u $UID $SIMPLE_USER
RUN echo $SIMPLE_USER':pswd!!' | chpasswd
RUN echo "newubu ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
#RUN useradd -ms /bin/bash ${SIMPLE_USER}
USER ${SIMPLE_USER}



# Setlocale
RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8


RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe


RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN sudo apt-get update
RUN sudo apt upgrade


RUN sudo DEBIAN_FRONTEND=noninteractive apt install ros-foxy-desktop python3-argcomplete -y --fix-missing


RUN echo \"source /opt/ros/foxy/setup.bash\" >> ~/.bashrc
RUN source /opt/ros/foxy/setup.bash


RUN sudo apt-get install git -y
RUN sudo apt-get install wget -y


WORKDIR /home/Stage
RUN git clone https://github.com/gardegu/LAB42_RTX_control.git
WORKDIR /home/Stage/LAB42_RTX_control
RUN sudo ./install_dependencies.sh
WORKDIR ROS_ws


WORKDIR /home/Stage
RUN wget "https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu20"
RUN sudo apt install zstd -y
#RUN chmod +x ZED_SDK_Ubuntu20_cuda12.1_v4.0.5.zstd.run
RUN sudo chmod +x ubuntu20
RUN DEBIAN_FRONTEND=noninteractive ./ubuntu20


RUN wget "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin"
RUN sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget "https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda-repo-ubuntu2004-12-2-local_12.2.0-535.54.03-1_amd64.deb"
RUN sudo dpkg -i cuda-repo-ubuntu2004-12-2-local_12.2.0-535.54.03-1_amd64.deb
RUN cp /var/cuda-repo-ubuntu2004-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN sudo apt-get update
RUN sudo apt-get -y install cuda


RUN source /etc/lsb-release
RUN UBUNTU_VERSION=ubuntu${DISTRIB_RELEASE/./}
RUN wget "https://developer.download.nvidia.com/compute/cuda/repos/${UBUNTU_VERSION}/x86_64/cuda-keyring_1.0-1_all.deb"
RUN sudo dpkg -i cuda-keyring_1.0-1_all.deb


RUN sudo apt-get update
RUN sudo apt-get -y install cuda-11-8


RUN sudo snap install --classic code


WORKDIR /home/Stage/LAB42_RTX_control/ROS_ws
RUN colcon build





