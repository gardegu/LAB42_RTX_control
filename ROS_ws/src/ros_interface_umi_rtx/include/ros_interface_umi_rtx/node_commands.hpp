/**
 * @file node_commands.hpp
 * @author Théo MASSA (theo.massa@ensta-bretagne.org)
 * @brief Node associated with the GUI
 * @version 0.1
 * @date 2023-07-19
 * 
 */

#ifndef __OBJ_H__
#define __OBJ_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include <QApplication>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QObject>

#include <iostream>
#include <math.h>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

/**
 * @brief Node that works in pair with the GUI, manage the targeted pose and the command we send to the arm. 
 */
class Objective_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Objective_node object
     */
    Objective_node() : Node("objective"){
        init_interfaces();
    };

    /**
     * @brief Function that will be used at each iteration in the GUI to update the target that will be communicated.
     * 
     * @param new_x 
     * @param new_y 
     * @param new_z 
     * @param new_yaw 
     * @param new_pitch 
     * @param new_roll 
     * @param new_grip 
     */
    void update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip);

    string mode="manual";
    cv::Mat processed_frame, depth_frame;
    double x=0., y=0.6, z=0.6, yaw=0.,pitch=0.,roll=0., grip=0.2;

private :
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();
    /**
     * @brief Timer callback, actions that will be done at every iterations
     */
    void timer_callback();
    /**
     * @brief Lissajou's trajectory for the target, function to delete
     */
    void Lissajou(); //TODO delete

    /**
     * @brief Get the targeted position processed by the camera.
     * 
     * @param msg 
     */
    void get_processed_position(const geometry_msgs::msg::Point::SharedPtr msg);
    /**
     * @brief Get the targeted angles processed by the camera.
     * 
     * @param msg 
     */
    void get_processed_angles(const geometry_msgs::msg::Vector3::SharedPtr msg);
    /**
     * @brief Get the processed images published
     * 
     * @param msg 
     */
    void get_processed_image(const sensor_msgs::msg::Image::SharedPtr msg);
    /**
     * @brief Get the depth images published
     * 
     * @param msg 
     */
    void get_depth_image(const sensor_msgs::msg::Image::SharedPtr msg);
    
    std::chrono::milliseconds loop_dt_ = 40ms;

    double x0,y0,z0,yaw0,pitch0,roll0,t0;
    float t=0,dt=0.04;

    double processed_x,processed_y,processed_z,processed_yaw,processed_pitch,processed_roll;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr objective_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angles_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angles_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber;
    
};

#endif