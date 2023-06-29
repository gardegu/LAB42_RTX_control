#ifndef __CAMERA__
#define __CAMERA__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

class Camera : public rclcpp::Node{
public:
    Camera() : Node("control") {
        init_interfaces();
        init_camera();
    };

private:
    void timer_callback();
    void init_interfaces();
    void init_camera();
    void get_angles(vector<vector<cv::Point>> &contours);
    void stereo_calibration();
    void stereo_rectification();

    std::chrono::milliseconds loop_dt_ = 40ms;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angles_publisher;

    cv::VideoCapture cap;
    cv::Mat frame;

    cv::Size m_patternSize(7,5);
    float m_squareSize(3.1);
    double m_baseline = 63;
    double m_focalLength = 2.8;

    double m_rms_error;

    cv::Mat m_cameraMatrixLeft, m_distCoeffsLeft, m_cameraMatrixRight, m_distCoeffsRight;
    cv::Mat m_R, m_T, m_E, m_F;
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;
    cv::Mat m_map1Left, m_map2Left, m_map1Right,

    double m_cx, m_cy, m_cz, yaw, pitch, roll;

    int m_frame_width, m_frame_height;

};


#endif