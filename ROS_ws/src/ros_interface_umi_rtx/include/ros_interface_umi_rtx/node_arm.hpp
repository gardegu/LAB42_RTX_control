/**
 * @file node_arm.hpp
 * @author Théo MASSA (theo.massa@ensta-bretagne.org)
 * @brief ROS2 node that is connected to the arm and allows to send received commands to it.
 * @version 0.1
 * @date 2023-07-19
 * 
 */

#ifndef __NODE_ARM_H__
#define __NODE_ARM_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "ros_interface_umi_rtx/umi-drivers/armlib.h"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"
#include "ros_interface_umi_rtx/umi-drivers/armraw.h"
#include "ros_interface_umi_rtx/umi-drivers/comm.h"
#include "ros_interface_umi_rtx/umi-drivers/rtxd.h"
#include "ros_interface_umi_rtx/arm_parts/arm.h"
#include "ros_interface_umi_rtx/robotics/umi.h"

#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <fcntl.h>
#include <signal.h>
#include <chrono>
#include <map>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;


/**
 * @brief Node that communicates with the arm, intermediary between ROS architecture and arm's drivers 
 */
class Arm_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Arm_node object
     */
    Arm_node() : Node("arm_node") {
        init_interfaces();
        
        // Initialize the arm
        umi_init();

    };

private:
    /**
     * @brief Timer callback, actions that will be done at every iterations
     */
    void timer_callback();
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();
    /**
     * @brief Get the commands that will be sent to the arm
     * 
     * @param msg States of the joints required to reach the desired position
     */
    void get_commands(const sensor_msgs::msg::JointState::SharedPtr msg);
    /**
     * @brief Get the targeted position
     *
     * @param msg 
     */
    void get_pose(const geometry_msgs::msg::Pose::SharedPtr msg);
    /**
     * @brief Get the targeted grip
     * 
     * @param msg 
     */
    void get_grip(const std_msgs::msg::Float32::SharedPtr msg);
    /**
     * @brief Set the motors/joints to their required state to reach the desired position
     * 
     */
    void set_motors();
    /**
     * @brief Get the joints' parameters
     * 
     */
    void get_params();

    /**
     * @brief Converts motors_params into a string to publish more easily
     * 
     * @return string 
     */
    string params2msg();
    
    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,double> commands_motor; // Map that stores the commands for each motor
    map<int,map<int,int>> motors_params; // Keeps in memory the parameters of the motors

    Arm full_arm;
    double targ_x,targ_y,targ_z, x,y,z;
    double target_yaw, target_pitch, target_roll, yaw, pitch, roll;
    double target_grip, grip;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_commands;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr grip_subscription;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_params;
};

#endif