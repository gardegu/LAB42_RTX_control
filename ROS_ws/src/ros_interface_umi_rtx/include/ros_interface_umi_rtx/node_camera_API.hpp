/**
 * @file node_camera_API.hpp
 * @author Guillaume Garde (guillaume.garde@ensta-bretagne.org)
 * @brief This node is dedicated to the computer vision part of this project. 
 * It allows to detect the targeted banana, get its 3D position in the work frame of the UMI-RTX
 * and publish its coordinates for the inverse kinematics node.
 * @version 0.1
 * @date 2023-08-15
 * 
 * 
 */
#ifndef CAMERA_API_H
#define CAMERA_API_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
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

/**
 * @brief This class uses Stereolabs' SDK and OpenCV.
 * 
 */
class Camera_API : public rclcpp::Node{
public:
    Camera_API() : Node("API_node") { /**
     * @brief Creates the publishers, sets up the camera.
     * 
     */
        init_interfaces();
        init_camera(); 
    };

private:
    /**
     * @brief Work loop. This method tries to read the scene, computes depth,
     * converts images from SL format to OpenCV format, get the target's position
     * and orientation, computes the 3D coordinates of the target and publish them.
     * 
     */
    void timer_callback();

    /**
     * @brief Sets up the work loop and instantiates the publishers.
     * 
     */
    void init_interfaces();

    /**
     * @brief Sets up the chosen parameters of the stereo camera and tries to open it.
     * 
     */
    void init_camera();

    /**
     * @brief Binarizes the image to find the banana, finds it, computes its area,
     * computes its centroid's coordinates, gets its orientation and publish this data.
     * 
     * @param msg Message to publish the pose of the banana.
     */
    void get_banana_and_angles(geometry_msgs::msg::Pose msg);

    /**
     * @brief Finds the fittest line with respect to the contour of the target and
     * computes its orientation.
     * 
     * @param contours The set of detected contours in the binarized image.
     */
    void get_angles(vector<vector<cv::Point>> &contours);

    /**
     * @brief Associates the corresponding OpenCV format for a SL image.
     * 
     * @param type SL format of the image to convert.
     * @return int OpenCV format of the image to convert.
     */
    int getOCVtype(sl::MAT_TYPE type);

    /**
     * @brief Converts a SL image to OpenCV format.
     * 
     * @param input Image to convert.
     * @return cv::Mat Converted image.
     */
    cv::Mat slMat2cvMat(sl::Mat& input);
    
    std::chrono::milliseconds loop_dt_ = 40ms; //! Loop timer of the node.

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher; //! Image publisher of the left view with the contour of the target.
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr processed_pose_publisher; //! Pose publisher of the position and orientation of the target.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher; //! Image publisher of the depth map of the scene.

    double m_cx, m_cy, m_cz, yaw, pitch, roll; //! Coordinates of the target's centroid and associated Euler angles.
    double h=1.19; //! Height of the stereo camera in meters with respect to the arm's support.

    float fx = 1543.25, fy = 1543.29; //! Horizontal and vertical focal lengthes.

    Camera zed; //! Instance of SDK's Camera class.
    InitParameters init_parameters; //! Instance of the SDK's InitParameters class used to set the device up.
    sl::Mat zed_image_left; //! Image captured by the left lens.
    sl::Mat zed_depth, zed_point_cloud; //! Depth map of the scene and associated 3D point cloud.
    sl::float4 point_cloud_value; //! Variable to get the values of one point of the point cloud.

    cv::Mat cv_image_left, cv_depth; //! Image captured by the left lens and associated depth map with OpenCV format.

    int zed_image_left_width, zed_image_left_height; //! Size of the captured image.

};


#endif //CAMERA_API_H
