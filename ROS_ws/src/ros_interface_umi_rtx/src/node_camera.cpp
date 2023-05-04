#include "ros_interface_umi_rtx/node_camera.hpp"

void Camera::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));
}

void Camera::init_camera(){
    //The following code is just a test and won't be used in the project
    cv::Mat img_test = cv::imread("testLab42.jpg");
    if(img_test.empty()){
        std::cout << "Could not read the image" << std::endl;
        std::cout << "ici" << std::endl;
    }
    cv::Size displaySize(600,800);
    cv::Mat resized_img_test;
    cv::resize(img_test,resized_img_test,displaySize);
    cv::imshow("Test image",resized_img_test);
    /*int k = cv::waitKey(0);
    if(k == 'q'){
        cv::destroyAllWindows();
    }*/
    cv::Mat gray_img;
    cv::cvtColor(resized_img_test,gray_img,cv::COLOR_BGR2GRAY);
    cv::imshow("Gray test image",gray_img);
    /*k = cv::waitKey(0);
    if(k == 'q'){
        cv::destroyAllWindows();
    }*/
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::imshow("Binary image", binary_img);
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