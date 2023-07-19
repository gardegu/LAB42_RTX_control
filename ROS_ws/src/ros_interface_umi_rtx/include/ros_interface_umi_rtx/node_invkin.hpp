/**
 * @file node_invkin.hpp
 * @author Théo MASSA (theo.massa@ensta-bretagne.org)
 * @brief Node dedicated in the inverse kinematics processing
 * @version 0.1
 * @date 2023-07-19
 * 
 */

#ifndef __INVKIN_H__
#define __INVKIN_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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
    /**
     * @brief Construct a new InvKin_node object
     * 
     */
    InvKin_node() : Node("inverse_kinematics") {
        init_interfaces();
        pinocchio::urdf::buildModel(urdf_file,model);
        data = pinocchio::Data(model);
        J = pinocchio::Data::Matrix6x(6,model.nv);
    };

private:
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();
    /**
     * @brief Timer callback, actions that will be done at every iterations
     */
    void timer_callback();
    /**
     * @brief Get the targeted position
     *
     * @param msg 
     */
    void get_position(const geometry_msgs::msg::Point::SharedPtr msg);
    /**
     * @brief Get the targeted orientation
     * 
     * @param msg 
     */
    void get_angles(const geometry_msgs::msg::Vector3::SharedPtr msg);
    /**
     * @brief Get the targeted grip
     * 
     * @param msg 
     */
    void get_grip(const std_msgs::msg::Float32::SharedPtr msg);
    
    /**
     * @brief Processed the joints' states required to reach the desired pose, using an inverse kinematics algorithm
     * 
     * @param x 
     * @param y 
     * @param z 
     */
    void get_state(double x, double y, double z);

    /**
     * @brief Put joints' angles in [-pi,pi]
     * 
     * @param q 
     */
    void correct_angle(Eigen::VectorXd &q);

    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,double> state;

    float L = 0.15; // Length of the hand

    double last_x,last_y,last_z;
    int ROLL=6, PITCH=7;

    double target_yaw, last_yaw;
    double target_pitch, last_pitch;
    double target_roll, last_roll;
    double target_grip, lats_grip;

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
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angles_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr grip_subscription;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher;

};



#endif