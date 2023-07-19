#include "ros_interface_umi_rtx/node_camera_API.hpp"

void Camera_API::init_interfaces(){
    m_cx = 0;
    m_cy = 0;

    stereo = cv::StereoSGBM::create(min_disp,num_disp,blockSize,iP1,iP2,disp12MaxDiff,0,uniquenessRatio,speckleWindowSize,speckleRange);

    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera_API::timer_callback,this));

    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
    coord_publisher = this->create_publisher<geometry_msgs::msg::Point>("processed_position",10);
    angles_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("processed_angles",10);
    depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image",10);
    double_publisher = this->create_publisher<std_msgs::msg::Float64>("target_depth",10);
}

void Camera_API::init_camera(){
    init_parameters.camera_resolution = RESOLUTION::HD720;
    init_parameters.camera_fps_ = 15;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinates_units = UNIT::MILLIMETER;
    init_parameters.depth_minimum_distance = 0; // set it to the minimum authorized value (should be 100 mm)


    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS){
        std::cout << "Error " << returned_state << ", exit program." << std::endl;
        //return EXIT_FAILURE;
    }

    //std::cout << "init done" << std::endl;
}

void Camera_API::timer_callback(){
    geometry_msgs::msg::Point coord_msg;
    geometry_msgs::msg::Vector3 angles_msg;

    if (zed.grab() == ERROR_CODE::SUCCESS){
        zed.retrieveImage(zed_image_left,VIEW::LEFT);
        zed.retrieveImage(zed_image_right,VIEW::RIGHT);
        zed.retrieveMeasure(zed_depth,MEASURE::DEPTH);
        zed.retrieveMeasure(zed_point_cloud,MEASURE::XYZRGBA);

        zed_image_left_width = zed_image_left.getWidth();
        zed_image_left_height = zed_image_left.getHeight();

    }

    get_banana_and_angles(coord_msg,angles_msg);

    sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",zed_depth).toImageMsg();
    depth_publisher->publish(*depth_msg);

    sl::float4 point_cloud_value;
    zed_point_cloud.getValue(m_cx,m_cy,&point_cloud_value);

    std_msgs::msg::Float64 target_depth_msg;
    target_depth_msg.data = point_cloud_value;
    double_publisher->publish(target_depth_msg);

}

void Camera_API::get_banana_and_angles(geometry_msgs::msg::Point coord_msg, geometry_msgs::msg::Vector3 angles_msg){
    cv::Mat hsv_img;
    cv::cvtColor(zed_image_left,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if(contours.empty()){
        //std::cout << "Cannot detect the target" << std::endl;
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",zed_image_left).toImageMsg();
        image_publisher->publish(*img_msg);
    }

    else{

        double maxArea = 0;
        int maxAreaIdx = -1;

        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);

            if (area > maxArea)
            {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        //std::cout << "indice : " << maxAreaIdx << std::endl;

        if(maxAreaIdx > -1) {
            get_angles(contours);
            cv::drawContours(zed_image_left, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);

            cv::Moments moments = cv::moments(contours[maxAreaIdx]);

            if (moments.m00 != 0) {
                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;
                //std::cout << "Centroid : (" << cx << ", " << cy << ")" << std::endl;

                coord_msg.x = cx;
                coord_msg.y = cy;
                m_cx = cx;
                m_cy = cy;
            }

            else {
                std::cout << "Impossible centroid calculation" << std::endl;

                coord_msg.x = m_cx;
                coord_msg.y = m_cy;
            }
        }

        else{
            coord_msg.x = m_cx;
            coord_msg.y = m_cy;
        }

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", zed_image_left).toImageMsg();
        image_publisher->publish(*img_msg);
    }

    coord_publisher->publish(coord_msg);

    angles_msg.x = yaw;
    angles_msg.y = pitch;
    angles_msg.z = roll;
    angles_publisher->publish(angles_msg);
}

void Camera_API::get_angles(vector<vector<cv::Point>> &contours){
    vector<cv::Point> longest_contour;
    double max_area = 0.0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            longest_contour = contour;
        }
    }

    cv::Vec4f line_params;
    cv::fitLine(longest_contour, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line_params[0];
    float vy = line_params[1];
    float theta = atan2(vy, vx) + M_PI/2;

    yaw = 90.;
    pitch = 90.;
    roll = theta*180/M_PI;

}


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera_API>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}