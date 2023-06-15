#include "ros_interface_umi_rtx/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));
    objective_publisher  = this->create_publisher<geometry_msgs::msg::Point>("target_position",10);
    pitch_publisher = this->create_publisher<std_msgs::msg::Float32>("target_pitch",10);
    roll_publisher = this->create_publisher<std_msgs::msg::Float32>("target_roll",10);
}

void Objective_node::timer_callback(){
    if (!manual_control){
        Lissajou();
    }
    t+=dt;

    geometry_msgs::msg::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    objective_publisher->publish(msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = pitch;
    pitch_publisher->publish(pitch_msg);

    std_msgs::msg::Float32 roll_msg;
    roll_msg.data = roll;
    roll_publisher->publish(roll_msg);
}

void Objective_node::Lissajou(){
    x = 0.3*sin(t);
    y = 0.4+0.1*sin(0.6*t);
    z = 0.3+0.2*sin(t);
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