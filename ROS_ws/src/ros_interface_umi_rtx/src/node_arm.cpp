#include "ros_interface_umi_rtx/node_arm.hpp"


using namespace std::placeholders;
using namespace std;


void Arm_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Arm_node::timer_callback, this));

    subscription_commands = this->create_subscription<sensor_msgs::msg::JointState>("motor_commands",10,
        std::bind(&Arm_node::get_commands, this, _1));

    publisher_params  = this->create_publisher<std_msgs::msg::String>("motor_params",10);

    if (arm_init_comms(1,2)!=-1){
        cout << "----------Communication initialized----------" << endl;
    }
    else {
        cout << "----------Error in communication initialisation----------" << endl;
    }
}

void Arm_node::timer_callback(){
    get_params();

    // Collect the parameters and publish them
    std_msgs::msg::String params;

    params.data = params2msg();

    publisher_params->publish(params);
}

void Arm_node::get_commands(const sensor_msgs::msg::JointState::SharedPtr msg){
    // stringstream ss(msg->data);
    // string pair;

    vector<double> objective = msg->position;

    commands_motor = {{ZED,objective[0]},
                      {SHOULDER,objective[1]},
                      {ELBOW,objective[2]}};
    // TODO : add the other joints
}

void Arm_node::set_motors(){
    map<int,float> conv_map = {{ELBOW,CONV_ELBOW},
                               {SHOULDER,CONV_SHOULDER},
                               {WRIST1,CONV_W},
                               {WRIST2,CONV_W},
                               {YAW,CONV_YAW}};
    float conv_ticks_to_deg ;
    
    int key,sign_error;
    float obj_angle,motor_angle,delta;

    for (const auto& pair:commands_motor){
        key = pair.first;
        obj_angle = pair.second;

        conv_ticks_to_deg = conv_map[key];
        motor_angle = conv_ticks_to_deg*motors_params[key][CURRENT_POSITION];

        delta = obj_angle-motor_angle;
        sign_error = delta/abs(delta);

        full_arm.mJoints[key]->setOrientation(sign_error);
    }
}

void Arm_node::get_params(){
    int value,res;
    motors_params = {};
    
    for (const auto motor:full_arm.mJoints){
        motors_params[motor->m_ID] = {};
        // cout << motor->m_ID << endl;
        for (int PID=0; PID<NUMBER_OF_DATA_CODES; PID++){
            res = motor->get_parameter(PID, &value);
            if (res!=-1){
                motors_params[motor->m_ID][PID] = value;
            }
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