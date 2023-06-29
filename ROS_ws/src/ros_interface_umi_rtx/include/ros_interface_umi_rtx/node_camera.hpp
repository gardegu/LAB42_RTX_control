#ifndef __CAMERA__
#define __CAMERA__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
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
    void get_angles(vector<vector<cv::Point>> &contours;);
    void stereo_calibration();

    std::chrono::milliseconds loop_dt_ = 40ms;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_publisher;

    cv::VideoCapture cap;
    cv::Mat frame;

    cv::Size m_patternSize(7,5);
    float m_squareSize(3.1);
    double m_baseline = 63;
    double m_focalLength = 2.8;

    double m_rms_error;
    cv::Mat m_cameraMatrixLeft;
    cv::Mat m_distCoeffsLeft;
    cv::Mat m_cameraMatrixRight;
    cv::Mat m_distCoeffsRight;
    cv::Mat m_R;
    cv::Mat m_T;
    cv::Mat m_E;
    cv::Mat m_F;

    cv::Mat m_R1;
    cv::Mat m_R2;
    cv::Mat m_P1;
    cv::Mat m_P2;
    cv::Mat m_Q;

    double m_cx;
    double m_cy;

    int m_frame_width;
    int m_frame_height;

};


#endif