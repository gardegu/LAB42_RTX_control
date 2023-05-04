#include "ros_interface_umi_rtx/node_camera.hpp"

void Camera::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));
}

void Camera::init_camera(){
    //The following code is just a test and won't be used in the project
    Mat img_test = imread("testLab42.jpg");
    if(img_test.empty()){
        std::cout << "Could not read the image" << std::endl;
    }
    Size displaySize(600,800);
    Mat resized_img_test;
    resize(img_test,resized_img_test,displaySize);
    imshow("Test image",resized_img_test);
    int k = waitKey(0);
    if(k == 'q'){
        destroyAllWindows();
    }
    Mat gray_img = cvtColor(resized_img_test,COLOR_BGR2GRAY);
    imshow("Gray test image",gray_img);
    k = waitKey(0);
    if(k == 'q'){
        destroyAllWindows();
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