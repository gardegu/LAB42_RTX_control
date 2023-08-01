#ifndef CAMERA_API_H
#define CAMERA_API_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sl/Camera.hpp> // stereolabs API


using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;
using namespace sl;

class Camera_API : public rclcpp::Node{
public:
    Camera_API() : Node("API_node") {
        init_interfaces();
        init_camera();
    };

private:
    void timer_callback();
    void init_interfaces();
    void init_camera();
    void get_banana_and_angles(geometry_msgs::msg::Point coord_msg, geometry_msgs::msg::Vector3 angles_msg);
    void get_angles(vector<vector<cv::Point>> &contours);

    std::chrono::milliseconds loop_dt_ = 40ms;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angles_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr double_publisher;

    double m_cx, m_cy, m_cz, yaw, pitch, roll;

    Camera zed;
    InitParameters init_parameters;
    sl::Mat zed_image, zed_image_left, zed_image_right;
    sl::Mat zed_depth, zed_point_cloud;

    int zed_image_left_width, zed_image_left_height;

};


#endif //CAMERA_API_H