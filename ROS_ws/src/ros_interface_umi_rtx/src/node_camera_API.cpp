#include "ros_interface_umi_rtx/node_camera_API.hpp"

void Camera_API::init_interfaces(){
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
    init_parameters.camera_fps = 15;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_units = UNIT::MILLIMETER;
    init_parameters.depth_minimum_distance = 100; // set it to the minimum authorized value (should be 100 mm)

    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS){
        std::cout << "Error " << returned_state << ", exit program." << std::endl;
        //return EXIT_FAILURE;
    }

    //std::cout << "init done" << std::endl;
}

void Camera_API::timer_callback(){
    if (zed.grab() == ERROR_CODE::SUCCESS){
        zed.retrieveImage(zed_image_left,VIEW::LEFT);
        // zed.retrieveMeasure(zed_depth,MEASURE::DEPTH);
        zed.retrieveImage(zed_depth,VIEW::DEPTH);
        zed.retrieveMeasure(zed_point_cloud,MEASURE::XYZRGBA);

        zed_image_left_width = zed_image_left.getWidth();
        zed_image_left_height = zed_image_left.getHeight();

        cv_image_left = slMat2cvMat(zed_image_left);
        cv_depth = slMat2cvMat(zed_depth);

        cv::cvtColor(cv_image_left,cv_image_left,cv::COLOR_BGRA2BGR);
        cv::cvtColor(cv_depth,cv_depth,cv::COLOR_BGRA2BGR);

        cv::resize(cv_depth,cv_depth,cv::Size(1280,720));
    }
    else{
        std::cout << "Could read the scene" << std::endl;
    }

    geometry_msgs::msg::Point coord_msg;
    geometry_msgs::msg::Vector3 angles_msg;

    get_banana_and_angles(angles_msg);

    zed_point_cloud.getValue(m_cx,m_cy,&point_cloud_value);

    coord_msg.x = m_cx;
    coord_msg.y = m_cy;

    if(std::isfinite(point_cloud_value.z)){
        std_msgs::msg::Float64 target_depth_msg;
        target_depth_msg.data = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
        double_publisher->publish(target_depth_msg);

        m_cz = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
        coord_msg.z = m_cz/1000;
//        std::cout << "Distance at {"<<m_cx<<";"<<m_cy<<"}: " << target_depth_msg.data << std::endl;
    }
    else{
//        std::cout << "The distance could not be computed at {"<<m_cx<<";"<<m_cy<<"}" << std::endl;
    }

    coord_publisher->publish(coord_msg);

    sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",cv_depth).toImageMsg();
    depth_publisher->publish(*depth_msg);

}

void Camera_API::get_banana_and_angles(geometry_msgs::msg::Vector3 angles_msg){
    cv::Mat hsv_img;
    cv::cvtColor(cv_image_left,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if(contours.empty()){
        //std::cout << "Cannot detect the target" << std::endl;
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",cv_image_left).toImageMsg();
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
            cv::drawContours(cv_image_left, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);

            cv::Moments moments = cv::moments(contours[maxAreaIdx]);

            if (moments.m00 != 0) {
                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;
                //std::cout << "Centroid : (" << cx << ", " << cy << ")" << std::endl;

                m_cx = cx;
                m_cy = cy;
            }

            else {
                std::cout << "Impossible centroid calculation" << std::endl;
            }
        }

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image_left).toImageMsg();
        image_publisher->publish(*img_msg);
    }

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

int Camera_API::getOCVtype(sl::MAT_TYPE type){
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

cv::Mat Camera_API::slMat2cvMat(sl::Mat& input){
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera_API>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}