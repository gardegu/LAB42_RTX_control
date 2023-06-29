#include "ros_interface_umi_rtx/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));

    objective_publisher  = this->create_publisher<geometry_msgs::msg::Point>("target_position",10);
    angles_publisher  = this->create_publisher<geometry_msgs::msg::Vector3>("target_angles",10);

    position_subscriber = this->create_subscription<geometry_msgs::msg::Point>("processed_position",10,
        std::bind(&Objective_node::get_processed_position, this, _1));
    angles_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("processed_angles",10,
        std::bind(&Objective_node::get_processed_angles, this, _1));
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("processed_image",10,
        std::bind(&Objective_node::get_image, this, _1));
}

void Objective_node::timer_callback(){
    if (!manual_control){
        Lissajou();
        // pitch = processed_pitch;
        // roll = processed_roll;
    }
    t+=dt;

    geometry_msgs::msg::Point position_msg;
    position_msg.x = x;
    position_msg.y = y;
    position_msg.z = z;
    objective_publisher->publish(position_msg);

    geometry_msgs::msg::Vector3 angles_msg;
    angles_msg.x = yaw; 
    angles_msg.y = pitch;
    angles_msg.z = roll;

    angles_publisher->publish(angles_msg);
}

void Objective_node::get_processed_position(const geometry_msgs::msg::Point::SharedPtr msg){
    processed_x = msg->x;
    processed_y = msg->y;
    processed_z = msg->z;
}

void Objective_node::get_processed_angles(const geometry_msgs::msg::Vector3::SharedPtr msg){
    processed_yaw = msg->x;
    processed_pitch = msg->y;
    processed_roll = msg->z;
}

void Objective_node::get_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    frame = cv_ptr->image;
}

void Objective_node::Lissajou(){
    x = 0.2*sin(0.5*t);
    y = 0.3+0.1*sin(0.3*t);
    z = 0.3+0.2*sin(0.2*t);
}

void Objective_node::update_state(double new_x, double new_y, double new_z, double new_roll, double new_pitch){
    x = new_x;
    y = new_y;
    z = new_z;
    roll = new_roll;
    pitch = new_pitch;
}

// int main(int argc, char *argv[]){
//     rclcpp::init(argc,argv);
//     shared_ptr<rclcpp::Node> node = make_shared<Objective_node>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return EXIT_SUCCESS;
// }