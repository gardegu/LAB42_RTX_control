#include "ros_interface_umi_rtx/node_camera.hpp"

void Camera::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));
}

void Camera::init_camera(){
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
    double Hmin = 0, Hmax = 179;
    double Smin = 0, Smax = 255;
    double Vmin = 0, Vmax = 255;

    cv::Mat hsv_img;
    cv::cvtColor(resized_img,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    cv::imshow("Binary HSV image", bin_hsv_img);
    int k = cv::waitKey(0);
    if(k == 'q'){
        cv::destroyAllWindows();
    }



}

void Camera::timer_callback(){

}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}