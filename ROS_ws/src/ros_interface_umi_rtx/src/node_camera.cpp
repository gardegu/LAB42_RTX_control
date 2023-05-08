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

void Camera::test(){
    //The following code is just a test and won't be used in the project
    cv::Mat img_test = cv::imread("testLab42.png");

    if(img_test.empty()){
        std::cout << "Could not read the image" << std::endl;
        std::cout << "ici" << std::endl;
    }

    cv::Size displaySize(600,800);
    cv::Mat resized_img;
    cv::resize(img_test,resized_img,displaySize);

    /*
    cv::Mat gray_img;
    cv::cvtColor(resized_img,gray_img,cv::COLOR_BGR2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::imshow("Binary image", binary_img);

    cv::Mat struc = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::Mat opened_img;
    cv::morphologyEx(binary_img, opened_img, cv::MORPH_OPEN, struc,cv::Point(-1,-1), 2);
    cv::imshow("Opened binary image",opened_img);

    int k = cv::waitKey(0);
    if(k == 'q'){
        cv::destroyAllWindows();
    }
    */

    /*Warning : Hue (H) values are divided by 2 in OpenCv (from 0 to 179 rather than
     * from 0 to 360 in theory)
     */

    cv::Mat hsv_img;
    cv::cvtColor(resized_img,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    cv::imshow("Binary HSV image", bin_hsv_img);

    /*
    std::vector<cv::Mat> hsv_planes;
    cv::split(hsv_img, hsv_planes);
    cv::Mat hue_img = hsv_planes[0];
    cv::Mat saturation_img = hsv_planes[1];
    cv::Mat value_img = hsv_planes[2];
    cv::imshow("Hue img",hue_img);
    cv::imshow("Saturation img",saturation_img);
    cv::imshow("Value img",value_img);

    cv::Mat saturation_thresh_img;
    cv::threshold(saturation_img, saturation_thresh_img, 0.9 * 255, 255, cv::THRESH_BINARY);
    cv::Mat mask;
    cv::bitwise_not(saturation_thresh_img, mask);

    cv::Mat img_j;
    cv::bitwise_and(hue_img, mask, img_j);

    cv::Mat img_binary;
    cv::threshold(img_j, img_binary, 0.1 * 60, 255, cv::THRESH_BINARY);
    cv::imshow("test",img_binary);
    */



    int k = cv::waitKey(0);
    if(k == 'q'){
        cv::destroyAllWindows();
    }
}

void Camera::binFeed(){
    cv::VideoCapture cap(-1);
    if (!cap.isOpened()) {
        std::cout << "Unable to connect to camera." << std::endl;
    }
    while(true){
        cv::Mat frame;
        cap.read(frame);
        //cv::imshow("Camera Feed", frame);

        cv::Mat hsv_img;
        cv::cvtColor(frame,hsv_img,cv::COLOR_BGR2HSV);

        cv::Scalar lower_bound = cv::Scalar(20,100,100);
        cv::Scalar upper_bound = cv::Scalar(60,255,255);

        cv::Mat bin_hsv_img;
        cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

        cv::imshow("Binary HSV camera feed", bin_hsv_img);
        if(cv::waitKey(30)=='q'){
            break;
        }
    }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}