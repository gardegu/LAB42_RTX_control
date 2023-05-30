#ifndef __INVKIN_H__
#define __INVKIN_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ros_interface_umi_rtx/umi-drivers/rtx.h"

#include <map>
#include <math.h>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class InvKin_node : public rclcpp::Node{
public:
    InvKin_node() : Node("inverse_kinematics") {
        init_interfaces();
    };

private:
    void init_interfaces();
    void timer_callback();
    void get_pose(const geometry_msgs::msg::Point::SharedPtr msg);
    void get_state(float x, float y, float z);

    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,float> state;
    float targeted_z=0.;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher;

};



#endif