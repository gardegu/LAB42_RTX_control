#include "ros_interface_umi_rtx/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));

    objective_publisher  = this->create_publisher<geometry_msgs::msg::Point>("target_position",10);
    angles_publisher  = this->create_publisher<geometry_msgs::msg::Vector3>("target_angles",10);
    grip_publisher  = this->create_publisher<std_msgs::msg::Float32>("target_grip",10);
    mission_publisher  = this->create_publisher<std_msgs::msg::String>("mission",10);

    position_subscriber = this->create_subscription<geometry_msgs::msg::Point>("processed_position",10,
        std::bind(&Objective_node::get_processed_position, this, _1));
    angles_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("processed_angles",10,
        std::bind(&Objective_node::get_processed_angles, this, _1));
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("processed_image",10,
        std::bind(&Objective_node::get_image, this, _1));
}

void Objective_node::timer_callback(){
    double dt1 = 6;
    if (mode != "manual"){
        // Lissajou();
        // TODO replace objective by target position
        if ((t-t0)<dt1){
            x = x0 + (0.2-x0)*(t-t0)/dt1;
            y = y0 + (0.4-y0)*(t-t0)/dt1;
            z = z0 + (0.6-z0)*(t-t0)/dt1;

            pitch = pitch0 + (processed_pitch-pitch0)*(t-t0)/dt1;
            roll = roll0 + (processed_roll-roll0)*(t-t0)/dt1;
            grip = 0.08;
        }

        else if ((t-t0)>=dt1 and (t-t0)<12){
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
            grip = 0.02;
        }

        else if ((t-t0)>=12 and (t-t0)<18){
            x = x0 + (0.-x0)*(t-t0-12)/6;
            y = y0 + (0.5-y0)*(t-t0-12)/6;
            z = z0 + (0.8-z0)*(t-t0-12)/6;

            pitch = pitch0 + (0.-pitch0)*(t-t0-12)/6;
            roll = roll0 + (0.-roll0)*(t-t0-12)/6;
        } 
    }

    else {
        x0 = x;
        y0 = y;
        z0 = z;

        roll0 = roll;
        pitch0 = pitch;

        t0 = t;
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

    std_msgs::msg::Float32 grip_msg;
    grip_msg.data = grip;
    grip_publisher->publish(grip_msg);
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