#include "ros_interface_umi_rtx/node_simu.hpp"

void Simu_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Simu_node::timer_callback, this));
    invkin_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("motor_commands",10,
        std::bind(&Simu_node::get_commands, this, _1));
    simu_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
}

void Simu_node::timer_callback(){
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();

    int num_joints = joint_list.size();
    msg.position = vector<double>(num_joints,0.0);
    msg.name = vector<string>(num_joints);

    for (int i = 0; i < num_joints; i++){
        string name=joint_list[i];
        msg.name[i]=name;
        msg.position[i] = free_joints[name]["position"];
    }
    simu_publisher->publish(msg);
}

void Simu_node::init_urdf(){
    // string urdf_file = "./ROS_ws/src/ros_interface_umi_rtx/urdf/umi_rtx.urdf";
    ifstream infp(urdf_file);

    if (infp.is_open()) {
        string description((istreambuf_iterator<char>(infp)), istreambuf_iterator<char>());
        infp.close();

        xml_document<> doc;
        doc.parse<0>(const_cast<char*>(description.c_str()));

        xml_node<>* robotNode = doc.first_node("robot");
        if (!robotNode) {
            cout << "Error: <robot> balise not found in the description." << endl;
            return;
        }

        free_joints.clear();
        joint_list.clear();


        // Browse <joint> tags
        for (xml_node<>* jointNode = robotNode->first_node("joint"); jointNode; jointNode = jointNode->next_sibling("joint")) {
            // Extract joint attributes (type, name, limit, etc.)
            string jtype = jointNode->first_attribute("type")->value();
            string name = jointNode->first_attribute("name")->value();

            // Check that the joint is fixed
            if (jtype == "fixed" || jtype == "floating" || jtype == "planar") {
                continue;
            }

            joint_list.push_back(name);
            // Initialize joint based on attributes
            double minval = 0.0;
            double maxval = 0.0;

            if (jtype == "continuous") {
                minval = -M_PI;
                maxval = M_PI;
            } 
            else {
                xml_node<>* limitNode = jointNode->first_node("limit");
                if (limitNode) {
                    minval = stod(limitNode->first_attribute("lower")->value());
                    maxval = stod(limitNode->first_attribute("upper")->value());
                } else {
                    cout << name << " n'est pas fixe ni continu, mais les limites ne sont pas spécifiées !" << endl;
                    continue;
                }
            }

            // Handling mimic tags
            xml_node<>* mimic_tags = robotNode->first_node("mimic");
            if (mimic_tags){
                xml_node<>* tag = &mimic_tags[0];
                map<string, double> entry;
                entry["parent"] = stod(tag->first_attribute("joint")->value());

                xml_attribute<>* multiplier = tag->first_attribute("multiplier");
                xml_attribute<>* offset = tag->first_attribute("offset");
                if (multiplier) {
                    entry["factor"] = stod(multiplier->value());
                }
                if (offset) {
                    entry["offset"] = stod(offset->value());
                }

                dependent_joints[name] = entry;
                continue;
            }

            auto it = dependent_joints.find(name);
            if (it != dependent_joints.end()) { // name in dependant_joints
                continue;
            }

            double zeroval;
            auto it2 = zeros.find(name);
            if (it2 != zeros.end()){
                zeroval = zeros[name];
            }
            else if (minval>0 or maxval<0){
                zeroval = (maxval + minval)/2;
            }
            else{
                zeroval = 0;
            }

            // Initialize the joint in the appropriate data structure
            map<string, double> joint = {{"min",minval},
                                         {"max",maxval},
                                         {"zero",zeroval},
                                         {"position",zeroval}};


            if (jtype == "continuous") {
                joint["continuous"] = 1.0;
            }
            free_joints[name] = joint;
        }
    } 

    else {
        cout << "Impossible d'ouvrir le fichier urdf" << endl;
    }
}



void Simu_node::get_commands(const sensor_msgs::msg::JointState::SharedPtr msg){

    // Zed
    string name = msg->name[0];
    map<string, double> *joint = &free_joints[name];
    (*joint)["position"] = min(max(msg->position[0],(*joint)["min"]),(*joint)["max"]);


    for (int i=1; i<msg->name.size(); i++){ // Revolute joints
        name = msg->name[i];
        joint = &free_joints[name];

        (*joint)["position"] = min(max(msg->position[i]*M_PI/180,(*joint)["min"]),(*joint)["max"]);
    }
}


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<Simu_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}