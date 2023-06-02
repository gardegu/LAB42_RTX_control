#ifndef __INVKIN_H__
#define __INVKIN_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "ros_interface_umi_rtx/umi-drivers/rtx.h"

#include <map>
#include <math.h>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class InvKin_node : public rclcpp::Node{
public:
    InvKin_node() : Node("inverse_kinematics") {
        init_interfaces();
        pinocchio::urdf::buildModel("./ROS_ws/src/ros_interface_umi_rtx/urdf/umi_rtx.urdf",model); //TODO replace "." by ".." when real arm
        data = pinocchio::Data(model);
        J = pinocchio::Data::Matrix6x(6,model.nv);
    };

private:
    void init_interfaces();
    void timer_callback();
    void get_pose(const geometry_msgs::msg::Point::SharedPtr msg);
    void get_state(double x, double y, double z);

    void correct_angle(Eigen::VectorXd &q); // Put angles in [-pi,pi]

    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,float> state;
    float targeted_z;


    // pinocchio variables and constants for inverse kinematics
    pinocchio::Model model;
    pinocchio::Data data;
    const int JOINT_ID = 5;
    const double eps  = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 1e-1;
    const double damp = 1e-12;
    pinocchio::Data::Matrix6x J;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher;

};



#endif