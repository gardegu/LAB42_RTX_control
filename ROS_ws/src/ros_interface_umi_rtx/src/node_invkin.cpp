#include "ros_interface_umi_rtx/node_invkin.hpp"

void InvKin_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&InvKin_node::timer_callback, this));
    pose_subscription = this->create_subscription<geometry_msgs::msg::Point>("target_position",10,
        std::bind(&InvKin_node::get_pose, this, _1));
    angles_publisher  = this->create_publisher<sensor_msgs::msg::JointState>("motor_commands",10);
    // angles_publisher  = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
}

void InvKin_node::timer_callback(){
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->get_clock()->now();
    msg.name = {"shoulder_updown","shoulder_joint","elbow","wrist","wrist_gripper_connection_roll","wrist_gripper_connection_pitch","gripper_left","gripper_right"};
    msg.position = {state[ZED],state[SHOULDER],state[ELBOW],0,0,0,0,0};

    angles_publisher->publish(msg);
}

void InvKin_node::get_pose(const geometry_msgs::msg::Point::SharedPtr msg){
    get_state(msg->x,msg->y,msg->z);
}


void InvKin_node::get_state(float x, float y, float z){
    float L = 500; // TODO : update

    float angle_shoulder = acos((pow(x,2)+pow(y,2)-2*pow(L,2))/pow(L,2));
    state[SHOULDER] = angle_shoulder*180/M_PI;

    float angle_elbow = atan2(y,x) - asin(L*sin(angle_shoulder)/sqrt(pow(x,2)+pow(y,2)));
    state[ELBOW] = angle_elbow*180/M_PI;

    // targeted_z = z;
    // state[ZED] = z;
    state[ZED] = 0.5; // TODO : test value to change
}


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<InvKin_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}