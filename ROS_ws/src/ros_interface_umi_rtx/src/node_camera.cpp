#include "ros_interface_umi_rtx/node_camera.hpp"

void Camera::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
}

void Camera::init_camera(){
    cap.open(0);
    if (!cap.isOpened()) {
        std::cout << "ERROR! Unable to open camera" << std::endl;
    }
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "init done" << std::endl;
}

void Camera::timer_callback(){
    cap.read(frame);

    cv::Mat hsv_img;
    cv::cvtColor(frame,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    /*sensor_msgs::msg::Image::SharedPtr message = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",bin_hsv_img).toImageMsg();

    image_publisher->publish(*message);*/

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

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
    cv::drawContours(frame, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);

    sensor_msgs::msg::Image::SharedPtr message = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg();
    image_publisher->publish(*message);

}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}