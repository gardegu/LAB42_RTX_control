/**
 * @file node_camera.hpp
 * @author Guillaume GARDE (guillaume.garde@ensta-bretagne.org)
 * @brief This node is dedicated to the computer vision part of this project.
 * It allows to detect the targeted banana, gets its centroid's 2D coordinates 
 * and tries to compute the depth maps of the scene.
 * @version 0.1
 * @date 2023-08-15
 * 
 * 
 */
#ifndef __CAMERA__
#define __CAMERA__

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

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

/**
 * @brief This class uses OpenCV
 * 
 */
class Camera : public rclcpp::Node{
public:
    /**
     * @brief Sets up the block-matching algorithm and creates useful publishers.
     * Tries to open the stereo camera, calibrates it and rectifies the images.
     * 
     */
    Camera() : Node("control") {
        init_interfaces();
        init_camera();
    };

private:
    /**
     * @brief Work loop. Splits the views, computes the banana's pose(position and orientation),
     * computes the disparity map of the scene, computes the depth map of the scene and publishes this data.
     * 
     */
    void timer_callback();

    /**
     * @brief Creates an object to use the block-matching algorithm and publishers to display
     * useful images and coordinates.
     * 
     */
    void init_interfaces();

    /**
     * @brief Tries to open the stereo camera, then calibrates it and rectifies the views.
     * 
     */
    void init_camera();

    /**
     * @brief Binarizes the image to find the banana, finds it, computes its area,
     * computes its centroid's coordinates, gets its orientation and publish this data.
     * 
     * @param coord_msg Message to publish the position of the banana.
     * @param angles_msg Message to publish the orientation of the banana.
     */
    void get_banana_and_angles(geometry_msgs::msg::Point coord_msg, geometry_msgs::msg::Vector3 angles_msg);

    /**
     * @brief Finds the fittest line with respect to the contour of the target and
     * computes its orientation.
     * 
     * @param contours The set of detected contours in the binarized image.
     */
    void get_angles(vector<vector<cv::Point>> &contours);

    /**
     * @brief Uses stereo images with easy-to-detect points on a chessboard and applies correspondence
    algorithms to compute the intrinsic and extrinsic parameters of the stereo camera. It will look for 
    an inner pattern. 
     * 
     */
    void stereo_calibration();

    /**
     * @brief 
     * 
     */
    void stereo_rectification();

    /**
     * @brief 
     * 
     */
    void stereo_split_views();

    /**
     * @brief 
     * 
     */
    void stereo_get_disparity();

    /**
     * @brief 
     * 
     */
    void stereo_get_depth();

    std::chrono::milliseconds loop_dt_ = 40ms;//!

    rclcpp::TimerBase::SharedPtr timer_;//!

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;//!
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_publisher;//!
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angles_publisher;//!
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_publisher;//!
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;//!
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr double_publisher;//!

    cv::VideoCapture cap;//!

    cv::Mat frame, frameLeft, frameRight;//!
    cv::Mat m_cameraMatrixLeft, m_distCoeffsLeft, m_cameraMatrixRight, m_distCoeffsRight;//!
    cv::Mat m_R, m_T, m_E, m_F;//!
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;//!
    cv::Mat m_map1Left, m_map2Left, m_map1Right, m_map2Right;//!
    cv::Mat disparityMap;//!
    cv::Mat depthMap;//!

    cv::Ptr<cv::StereoSGBM> stereo;//!

    int m_frame_width, m_frame_height;//!
    int m_frame_width_left, m_frame_height_left;//!
    int blockSize = 7;//!
    int min_disp = 0;//!
    int max_disp = 80;//!
    int num_disp = max_disp - min_disp;//!
    int uniquenessRatio = 10;//!
    int speckleWindowSize = 200;//!
    int speckleRange = 2;//!
    int disp12MaxDiff = 0;//!
    int iP1 = 8 * 1 * blockSize * blockSize;//!
    int iP2 = 16 * 1 * blockSize * blockSize;//!

    //cv::Size m_patternSize(7,5);
    float m_squareSize = 3.1;//!

    double m_baseline = 6.3; // centimeters //!
    double m_focalLength = 2.8;// millimeters //!
    double m_rms_error;//!
    double m_cx, m_cy, m_cz, yaw, pitch, roll;//!
    double depth_factor = 1.5;//!


};


#endif