#ifndef __NODE_ARM_H__
#define __NODE_ARM_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros_interface_umi_rtx/umi-drivers/armlib.h"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"
#include "ros_interface_umi_rtx/umi-drivers/armraw.h"
#include "ros_interface_umi_rtx/umi-drivers/comm.h"
#include <ros_interface_umi_rtx/umi-drivers/rtxd.h>

#include <ros_interface_umi_rtx/arm_parts/arm.h>

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
    };

private:
    void timer_callback();
    void init_interfaces();
    void get_commands(const std_msgs::msg::String::SharedPtr msg);
    void set_motors();
    void get_params();

    string params2msg(); //Converts motors_params into a string to publish more easily
    
    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,int> commands_motor; // Map which keeps the commands for each motor
    map<int,map<int,int>> motors_params; // Keeps in memory the parameters of the motors

    Arm full_arm;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_commands;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_params;
};

#endif