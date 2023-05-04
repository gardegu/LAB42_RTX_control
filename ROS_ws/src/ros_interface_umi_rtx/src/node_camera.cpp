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
    //cv::imshow("Test image",resized_img);

    /*
    cv::Rect roi(130, 110,470,650); //Regio Of Interest
    cv::Mat cropped_img = resized_img(roi);
    //cv::imshow("Cropped image",cropped_img);

    cv::Mat gray_img;
    cv::cvtColor(cropped_img,gray_img,cv::COLOR_BGR2GRAY);
    //cv::imshow("Gray test image",gray_img);
    */

    cv::Mat gray_img;
    cv::cvtColor(resized_img,gray_img,cv::COLOR_BGR2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::imshow("Binary image", binary_img);

    cv::Mat struc = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::Mat opened_img;
    cv::morphologyEx(binary_img, opened_img, cv::MORPH_OPEN, struc,cv::Point(-1,-1), 1);
    cv::imshow("Opened binary image",opened_img);

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