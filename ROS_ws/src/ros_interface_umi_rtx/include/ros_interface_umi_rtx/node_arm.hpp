#ifndef __ARM__
#define __ARM__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.h"

#include <chrono>
#include <map>

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
    void get_params();
    
    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,int> commands_motor; // Map which keeps the commands for each motor
};

#endif