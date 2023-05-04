#include "ros_interface_umi_rtx/node_arm.hpp"


using namespace std::placeholders;
using namespace std;

void Arm_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Arm_node::timer_callback, this));
    subscription_commands = this->create_subscription<std_msgs::msg::String>("motor_commands",10,
        std::bind(&Arm_node::get_commands, this, _1));
    publisher_params  = this->create_publisher<std_msgs::msg::String>("motor_params",10);
}

void Arm_node::timer_callback(){
    // Collect the parameters and publish them
    std_msgs::msg::String params;

    params.data = params2msg();

    publisher_params->publish(params);

}

void Arm_node::get_commands(const std_msgs::msg::String::SharedPtr msg){
    stringstream ss(msg->data);
    string pair;

    commands_motor = {};

    while (getline(ss, pair, ';')) {

        istringstream iss(pair);
        int key;

        int value;
        if ((iss >> key >> value)) {
            commands_motor[key] = value;
        }
    }
}

void Arm_node::set_motors(){

}

void Arm_node::get_params(){
    int value,res;
    for (int motor=0; motor<NUMBER_OF_MOTORS; motor++){
        motors_params[motor] = {};
        for (int param=0; param<NUMBER_OF_DATA_CODES; param++){
            res = arm_read(motor,param,&value);
            if (res!=-1){
                motors_params[motor][param] = value;
            }
        }
    }
}

string Arm_node::params2msg(){
    std::ostringstream oss;

    for (const auto& item1 : motors_params) {
        oss << item1.first << ": {";
        for (const auto& item2 : item1.second) {
            oss << item2.first << ":" << item2.second << "; ";
        }
        oss << "}; ";
    }
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