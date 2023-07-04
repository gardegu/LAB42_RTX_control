#include "ros_interface_umi_rtx/node_camera.hpp"

void Camera::init_interfaces(){
    m_cx = 0;
    m_cy = 0;

    stereo = cv::StereoSGBM::create(min_disp,num_disp,blockSize,iP1,iP2,disp12MaxDiff,0,uniquenessRatio,speckleWindowSize,speckleRange);

    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));

    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
    coord_publisher = this->create_publisher<geometry_msgs::msg::Point>("processed_position",10);
    angles_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("processed_angles",10);
    disparity_publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity_image",10);
}

void Camera::init_camera(){
    //cap.open(0); // webcam
    //cap.open(4) // robot's camera
    cap.open(4);

    if (!cap.isOpened()) {
        std::cout << "ERROR! Unable to open camera" << std::endl;
    }

    // Stereo Calibration of the ZED M device
    stereo_calibration();

    // Computing rectification parameters
    stereo_rectification();

    m_frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    m_frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    //std::cout << "init done" << std::endl;
}

void Camera::timer_callback(){
    geometry_msgs::msg::Point coord_msg;
    geometry_msgs::msg::Vector3 angles_msg;

    cap.read(frame);

    stereo_split_views();

    m_frame_width_left = frameLeft.cols;
    m_frame_height_left = frameLeft.rows;

    // Remplacer frame par frameLeft dans le cas où on utilise la caméra stéréo

    cv::Mat hsv_img;
    //cv::cvtColor(frame,hsv_img,cv::COLOR_BGR2HSV);
    cv::cvtColor(frameLeft,hsv_img,cv::COLOR_BGR2HSV);


    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if(contours.empty()){
        //std::cout << "Cannot detect the target" << std::endl;

        //cv::circle(frame,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);
        cv::circle(frameLeft,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);

        //cv::line(frame,cv::Point (m_frame_width/2 - 25,m_frame_height/2),cv::Point (m_frame_width/2 + 25,m_frame_height/2),cv::Scalar(255,255,255),2);
        //cv::line(frame,cv::Point (m_frame_width/2,m_frame_height/2 - 25),cv::Point (m_frame_width/2,m_frame_height/2 + 25),cv::Scalar(255,255,255),2);
        cv::line(frameLeft,cv::Point (m_frame_width_left/2 - 25,m_frame_height_left/2),cv::Point (m_frame_width_left/2 + 25,m_frame_height_left/2),cv::Scalar(255,255,255),2);
        cv::line(frameLeft,cv::Point (m_frame_width_left/2,m_frame_height_left/2 - 25),cv::Point (m_frame_width_left/2,m_frame_height_left/2 + 25),cv::Scalar(255,255,255),2);

        //sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frameLeft).toImageMsg();
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
            //cv::drawContours(frame, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);
            cv::drawContours(frameLeft, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);

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

                //cv::circle(frame,cv::Point(m_frame_width-40,40),20,cv::Scalar(100,50,100),-1);
                cv::circle(frameLeft,cv::Point(m_frame_width_left-40,40),20,cv::Scalar(100,50,100),-1);
            }
        }

        else{
            coord_msg.x = m_cx;
            coord_msg.y = m_cy;
        }

        //cv::circle(frame,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,255,0),-1);
        cv::circle(frameLeft,cv::Point(m_frame_width_left-40,40),20,cv::Scalar(0,255,0),-1);

        //cv::line(frame,cv::Point (m_frame_width/2 - 25,m_frame_height/2),cv::Point (m_frame_width/2 + 25,m_frame_height/2),cv::Scalar(255,255,255),2);
        //cv::line(frame,cv::Point (m_frame_width/2,m_frame_height/2 - 25),cv::Point (m_frame_width/2,m_frame_height/2 + 25),cv::Scalar(255,255,255),2);
        cv::line(frameLeft,cv::Point (m_frame_width_left/2 - 25,m_frame_height_left/2),cv::Point (m_frame_width_left/2 + 25,m_frame_height_left/2),cv::Scalar(255,255,255),2);
        cv::line(frameLeft,cv::Point (m_frame_width_left/2,m_frame_height_left/2 - 25),cv::Point (m_frame_width_left/2,m_frame_height_left/2 + 25),cv::Scalar(255,255,255),2);

        //sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frameLeft).toImageMsg();
        image_publisher->publish(*img_msg);
    }
    coord_publisher->publish(coord_msg);

    angles_msg.x = yaw;
    angles_msg.y = pitch;
    angles_msg.z = roll;
    angles_publisher->publish(angles_msg);

    stereo_get_disparity();
    sensor_msgs::msg::Image::SharedPtr disp_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",disparityMap).toImageMsg();
    disparity_publisher->publish(*disp_msg);

}

void Camera::get_angles(vector<vector<cv::Point>> &contours){
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

void Camera::stereo_calibration(){
    std::cout << "Calibrating the stereo device..." << std::endl;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> cornersLeft;
    std::vector<std::vector<cv::Point2f>> cornersRight;

    cv::Size imageLeftSize, imageRightSize;

    for(int i=1;i<=5;i++){
        //ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx")+"/images/icon.png"
        cv::Mat imageLeft = cv::imread(ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx") + "/CalibrationImages/leftImage0" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);
        cv::Mat imageRight = cv::imread(ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx") + "/CalibrationImages/rightImage0" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);

        if(imageLeft.empty()){
            std::cout << "Error reading the calibration left image " << i << std::endl;
        }
        if(imageRight.empty()){
            std::cout << "Error reading the calibration right image " << i << std::endl;
        }

        imageLeftSize = imageLeft.size();
        imageRightSize = imageRight.size();
        //std::cout << "Left image size: " << imageLeftSize << std::endl;
        //std::cout << "Right image size: " << imageRightSize << std::endl;

        std::vector<cv::Point2f> cornersLeftSub;
        std::vector<cv::Point2f> cornersRightSub;

        bool patternFoundLeft = cv::findChessboardCorners(imageLeft, cv::Size(7,5), cornersLeftSub, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool patternFoundRight = cv::findChessboardCorners(imageRight, cv::Size(7,5), cornersRightSub, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        //std::cout << "Found corners on left image? " << patternFoundLeft << std::endl;
        //std::cout << "Found corners on right image? " << patternFoundRight << std::endl;
        //std::cout << "Number of corners detected in image " << i << " (left) test1: " << cornersLeftSub.size() << std::endl;
        //std::cout << "Number of corners detected in image " << i << " (right) test1: " << cornersRightSub.size() << std::endl;

        if(patternFoundLeft && patternFoundRight){
            cv::cornerSubPix(imageLeft, cornersLeftSub, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::cornerSubPix(imageRight, cornersRightSub, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            cornersLeft.push_back(cornersLeftSub);
            cornersRight.push_back(cornersRightSub);

            std::vector<cv::Point3f> objectPointsImage;

            for (int y = 0; y < cv::Size(7,5).height; y++) {
                for (int x = 0; x < cv::Size(7,5).width; x++) {
                    objectPointsImage.push_back(cv::Point3f(x * m_squareSize, y * m_squareSize, 0));
                }
            }

            objectPoints.push_back(objectPointsImage);
        }

        //cv::drawChessboardCorners(imageLeft, patternSize, cornersLeftSub, patternFoundLeft);
        //cv::drawChessboardCorners(imageRight, patternSize, cornersRightSub, patternFoundRight);
    }

    m_rms_error = cv::stereoCalibrate(objectPoints, cornersLeft, cornersRight, m_cameraMatrixLeft, m_distCoeffsLeft, m_cameraMatrixRight, m_distCoeffsRight, imageLeftSize, m_R, m_T, m_E, m_F);

    std::cout << "Stereo device calibrated\n" << std::endl;
}

void Camera::stereo_rectification(){
    std::cout << "Computing rectification paramters..." << std::endl;

    cv::stereoRectify(m_cameraMatrixLeft, m_distCoeffsLeft, m_cameraMatrixRight, m_distCoeffsRight, cv::Size(1280,720), m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q, cv::CALIB_ZERO_DISPARITY);

    cv::initUndistortRectifyMap(m_cameraMatrixLeft, m_distCoeffsLeft, m_R1, m_P1, cv::Size(1280,720), CV_32FC1, m_map1Left, m_map2Left);
    cv::initUndistortRectifyMap(m_cameraMatrixRight, m_distCoeffsRight, m_R2, m_P2, cv::Size(1280,720), CV_32FC1, m_map1Right, m_map2Right);

    std::cout << "Rectification parameters computed\n" << std::endl;
}

void Camera::stereo_split_views(){
    if(frame.empty()){
        std::cout << "Error reading the frame" << std::endl;
    }

    int width = frame.cols;
    int height = frame.rows;

    frameLeft = frame(cv::Rect(0,0,width/2,height));
    frameRight = frame(cv::Rect(width/2,0,width/2,height));

    cv::resize(frameLeft,frameLeft,cv::Size(1280,720));
    cv::resize(frameRight,frameRight,cv::Size(1280,720));
}

void Camera::stereo_get_disparity(){
    cv::remap(frameLeft,frameLeft,m_map1Left,m_map2Left,cv::INTER_LINEAR);
    cv::remap(frameRight,frameRight,m_map1Right,m_map2Right,cv::INTER_LINEAR);

    stereo->compute(frameLeft,frameRight,disparityMap);

    cv::threshold(disparityMap,disparityMap,0,0,cv::THRESH_TOZERO);

    disparityMap.convertTo(disparityMap,CV_32F,1.0/16.0);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}