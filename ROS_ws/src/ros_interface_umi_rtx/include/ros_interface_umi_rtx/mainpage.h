/**
 * @mainpage ROS2 Interface for the UMI-RTX Arm
 *
 * @section authors_sec Authors
 * - GARDE Guillaume \<guillaume.garde@ensta-bretagne.org\> (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)
 * - MASSA Théo \<theo.massa@ensta-bretagne.org\> (Promotion ENSTA Bretagne 2024 - Spécialité Robotique Autonome)
 *
 * @section description_sec Description
 * This repository has for purpose to set up a ROS2 interface in order to use the UMI-RTX Arm.
 *
 * @section config_sec Configuration
 * This project is built and tested with **Ubuntu 20.04** and **ROS2 Foxy**.
 *
 * @section usage_sec Usage
 * @subsection real_arm_sec Real Arm
 * To use the interface, all you have to do is:
 * \code{.sh}
 * git clone https://github.com/gardegu/LAB42_RTX_control
 * cd LAB42_RTX_control
 * ## install the required dependencies
 * ./install_dependencies.sh
 * cd ROS_ws
 * ## Build the package
 * colcon build
 * cd..
 * ## Launch the arm
 * ./start_arm
 * \endcode
 *
 * The custom GUI will launch at the same time as the arm.
 *
 * If you want to monitor the nodes and topics or interact with them, all you have to do is logging as root:
 * \code{.sh}
 * sudo -i
 * \endcode
 *
 * And then sourcing ROS:
 * \code{.sh}
 * source /opt/ros/foxy/setup.bash
 * \endcode
 *
 * @subsection simulation_sec Simulation
 * In order to start only the simulation, just place yourself in LAB42_RTX_control and do:
 * If you are on bash:
 * \code{.sh}
 * source ROS_ws/install/setup.bash
 * \endcode
 * If you are on zsh:
 * \code{.sh}
 * source ROS_ws/install/setup.zsh
 * \endcode
 * and then:
 * \code{.sh}
 * ros2 launch ros_interface_umi_rtx simu.launch.py
 * \endcode
 */
