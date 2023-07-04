#ifndef __SIMU_H__
#define __SIMU_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include "rapidxml.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;
using namespace rapidxml;

class Simu_node : public rclcpp::Node{
public:
    Simu_node() : Node("simulation"){
        init_interfaces();
        init_urdf();
    };

private :
    void init_interfaces();
    void timer_callback();
    void init_urdf();
    void get_commands(const sensor_msgs::msg::JointState::SharedPtr msg);
    void get_mission(const std_msgs::msg::String::SharedPtr msg);

    std::chrono::milliseconds loop_dt_ = 40ms;

    map<string,map<string, double>> dependent_joints;
    map<string,map<string, double>> free_joints;
    map<string,double> zeros;

    vector<string> joint_list;
    vector<string> names;

    string urdf_file = ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx")+"/urdf/umi_rtx.urdf";
    string mission;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr invkin_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr simu_publisher;
    
};

#endif