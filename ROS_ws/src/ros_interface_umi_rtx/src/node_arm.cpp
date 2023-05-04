#include "rclcpp/rclcpp.hpp"
#include "ros_interface_umi_rtx/node_arm.hpp"

using namespace std::placeholders;
using namespace std;

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<Turret>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}