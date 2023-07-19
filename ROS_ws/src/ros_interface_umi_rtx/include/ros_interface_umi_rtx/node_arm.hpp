#ifndef __NODE_ARM_H__
#define __NODE_ARM_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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

class Arm_node : public rclcpp::Node{
public:
    Arm_node() : Node("arm_node") {
        init_interfaces();
        
        // Initialize the arm
        umi_init();

    };

private:
    void timer_callback();
    void init_interfaces();
    void get_commands(const sensor_msgs::msg::JointState::SharedPtr msg);
    void get_position(const geometry_msgs::msg::Point::SharedPtr msg);
    void get_angles(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void get_grip(const std_msgs::msg::Float32::SharedPtr msg);
    void set_motors();
    void get_params();

    string params2msg(); //Converts motors_params into a string to publish more easily
    
    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,double> commands_motor; // Map that stores the commands for each motor
    map<int,map<int,int>> motors_params; // Keeps in memory the parameters of the motors

    Arm full_arm;
    double targ_x,targ_y,targ_z, x,y,z;
    double target_yaw, target_pitch, target_roll, yaw, pitch, roll;
    double target_grip, grip;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_commands;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angles_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr grip_subscription;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_params;
};

#endif