#include "ros_interface_umi_rtx/node_invkin.hpp"

void InvKin_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&InvKin_node::timer_callback, this));
    pose_subscription = this->create_subscription<geometry_msgs::msg::Point>("target_position",10,
        std::bind(&InvKin_node::get_pose, this, _1));
    angles_publisher  = this->create_publisher<std_msgs::msg::String>("motor_commands",10);
}

void InvKin_node::timer_callback(){
    std_msgs::msg::String msg_angles;

    msg_angles.data = angles2msg();

    angles_publisher->publish(msg_angles);
}

void InvKin_node::get_pose(const geometry_msgs::msg::Point::SharedPtr msg){
    get_angles(msg->x,msg->y);
    targeted_z = msg->z;
}

string InvKin_node::angles2msg(){
    std::stringstream ss;

    ss << "{";
    for (auto it = angles.begin(); it != angles.end(); ++it) {
        ss << it->first << ": " << it->second;
        if (std::next(it) != angles.end()) {
            ss << ", ";
        }
    }
    ss << "}";

    return ss.str();
}

void InvKin_node::get_angles(float x, float y){
    float L = 500; // TODO : update

    float angle_shoulder = acos((pow(x,2)+pow(y,2)-2*pow(L,2))/pow(L,2));
    angles[SHOULDER] = angle_shoulder*180/M_PI;

    float angle_elbow = atan2(y,x) - asin(L*sin(angle_shoulder)/sqrt(pow(x,2)+pow(y,2)));
    angles[ELBOW] = angle_elbow*180/M_PI;
}


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<InvKin_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}