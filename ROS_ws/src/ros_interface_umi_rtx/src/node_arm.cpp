#include "ros_interface_umi_rtx/node_arm.hpp"


using namespace std::placeholders;
using namespace std;


void Arm_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Arm_node::timer_callback, this));

    subscription_commands = this->create_subscription<sensor_msgs::msg::JointState>("motor_commands",10,
        std::bind(&Arm_node::get_commands, this, _1));
    
    pose_subscription = this->create_subscription<geometry_msgs::msg::Pose>("target_pose",10,
        std::bind(&Arm_node::get_pose, this, _1));
    
    grip_subscription = this->create_subscription<std_msgs::msg::Float32>("target_grip",10,
        std::bind(&Arm_node::get_grip, this, _1));

    // publisher_params  = this->create_publisher<std_msgs::msg::String>("motor_params",10);

    if (arm_init_comms(1,2)!=-1){
        cout << "----------Communication initialized----------" << endl;
    }
    else {
        cout << "----------Error in communication initialisation----------" << endl;
    }
}

void Arm_node::timer_callback(){
    // TODO improve time for get_params
    // get_params();

    // // Collect the parameters and publish them
    // std_msgs::msg::String params;

    // params.data = params2msg();

    // publisher_params->publish(params);

    if (commands_motor.size()>0 and !umi_moving() and (x!=targ_x or y!=targ_y or z!=targ_z or yaw!=target_yaw or pitch!=target_pitch or roll!=target_roll or grip!=target_grip)){
        x = targ_x;
        y = targ_y;
        z = targ_z;
        yaw = target_yaw;
        pitch = target_pitch;
        roll = target_roll;
        grip = target_grip;

        set_motors();
        arm_go(NUMERIC,0x1555);
    }
}

void Arm_node::get_commands(const sensor_msgs::msg::JointState::SharedPtr msg){

    vector<double> objective = msg->position;

    commands_motor = {{ZED,objective[0]},
                      {SHOULDER,objective[1]},
                      {ELBOW,objective[2]},
                      {WRIST1,(objective[4]+objective[5])},
                      {WRIST2,(objective[5]-objective[4])}};
}

void Arm_node::get_pose(const geometry_msgs::msg::Pose::SharedPtr msg){
    targ_x = msg->position.x;
    targ_y = msg->position.y;
    targ_z = msg->position.z;

    target_yaw = msg->orientation.x*M_PI/180;
    target_pitch = msg->orientation.y*M_PI/180;
    target_roll = msg->orientation.z*M_PI/180;
}

void Arm_node::get_grip(const std_msgs::msg::Float32::SharedPtr msg){
    commands_motor[GRIP] = msg->data*1000;
    target_grip = msg->data;
}

void Arm_node::set_motors(){
    int key;
    float obj_angle;
    
    for (const auto& pair:commands_motor){
        key = pair.first;
        obj_angle = pair.second;
        if (key!=ZED and key!=GRIP){ //Revolute joints
            full_arm.mJoints[key]->setOrientation(obj_angle);
        }

        else if (key==ZED){
            full_arm.mJoints[ZED]->setZed(pair.second);
        }

        else if (key==GRIP){
            full_arm.mJoints[GRIP]->setGrip(pair.second);
        }
    }
}

void Arm_node::get_params(){
    int value,res;
    motors_params = {};
    
    for (const auto motor:full_arm.mJoints){
        motors_params[motor->m_ID] = {};
        res = motor->get_parameter(CURRENT_POSITION, &value);
        if (res!=-1){
            motors_params[motor->m_ID][CURRENT_POSITION] = value;
        }
    }
}

string Arm_node::params2msg(){
    std::ostringstream oss;

    oss << "{";
    for (const auto& item1 : motors_params) {
        oss << item1.first << ": {";
        for (const auto& item2 : item1.second) {
            oss << "{" << item2.first << "," << item2.second << "}";
            if (item2.first != 15){
                oss << ",";
            }
        }
        oss << "} ";
        if (item1.first!=7){
            oss << ",";
        }
    }
    oss << "}";
    std::string result = oss.str();

    result = result.substr(0, result.length() - 2);
    return result;
    }

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<Arm_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}