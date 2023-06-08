#ifndef __INVKIN_H__
#define __INVKIN_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

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
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class InvKin_node : public rclcpp::Node{
public:
    InvKin_node() : Node("inverse_kinematics") {
        init_interfaces();
        pinocchio::urdf::buildModel(urdf_file,model);
        data = pinocchio::Data(model);
        J = pinocchio::Data::Matrix6x(6,model.nv);
    };

private:
    void init_interfaces();
    void timer_callback();
    void get_pose(const geometry_msgs::msg::Point::SharedPtr msg);
    void get_pitch(const std_msgs::msg::Float32::SharedPtr msg);
    void get_state(double x, double y, double z);

    void correct_angle(Eigen::VectorXd &q); // Put angles in [-pi,pi]

    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,double> state;

    double last_x,last_y,last_z;
    int ROLL=6, PITCH=7;

    double target_pitch, last_pitch;

    string urdf_file = ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx")+"/urdf/umi_rtx.urdf";

    // pinocchio variables and constants for inverse kinematics
    pinocchio::Model model;
    pinocchio::Data data;
    const int JOINT_ID = 6;
    const double eps  = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 1e-1;
    const double damp = 1e-12;
    pinocchio::Data::Matrix6x J;
    Eigen::VectorXd q;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher;

};



#endif