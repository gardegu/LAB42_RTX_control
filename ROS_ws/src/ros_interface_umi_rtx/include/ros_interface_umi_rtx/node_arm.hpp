#ifndef __ARM__
#define __ARM__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "umi-rtx/include/umi-drivers/armlib.h"
#include "umi-rtx/include/umi-drivers/rtx.h"

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


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_commands;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_params;
};

#endif