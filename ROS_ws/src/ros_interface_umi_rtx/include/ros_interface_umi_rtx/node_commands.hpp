#ifndef __OBJ_H__
#define __OBJ_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
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

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class Objective_node : public rclcpp::Node{
public:
    Objective_node() : Node("objective"){
        init_interfaces();
    };

    void update_state(double new_x, double new_y, double new_z, double new_roll, double new_pitch);

    bool manual_control=true;
    cv::Mat frame;

private :
    void init_interfaces();
    void timer_callback();
    void Lissajou();

    void get_image(const sensor_msgs::msg::Image::SharedPtr msg);
    
    std::chrono::milliseconds loop_dt_ = 40ms;

    double x,y,z,pitch,roll;
    float t,dt=0.04;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr objective_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    
};

#endif