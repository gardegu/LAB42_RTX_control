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

class Objective_node : public rclcpp::Node{
public:
    Objective_node() : Node("objective"){
        init_interfaces();
    };

    void update_state(double new_x, double new_y, double new_z, double new_roll, double new_pitch);

    string mode="manual";
    cv::Mat frame;

private :
    void init_interfaces();
    void timer_callback();
    void Lissajou();

    void get_processed_position(const geometry_msgs::msg::Point::SharedPtr msg);
    void get_processed_angles(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void get_image(const sensor_msgs::msg::Image::SharedPtr msg);
    
    std::chrono::milliseconds loop_dt_ = 40ms;

    double x=0., y=0.5, z=0.5, yaw,pitch,roll;
    double x0,y0,z0,yaw0,pitch0,roll0,t0;
    double grip;
    float t=0,dt=0.04;

    double processed_x,processed_y,processed_z,processed_yaw,processed_pitch,processed_roll;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr objective_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angles_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angles_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    
};

#endif