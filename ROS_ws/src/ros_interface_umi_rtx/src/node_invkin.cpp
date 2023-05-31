#include "ros_interface_umi_rtx/node_invkin.hpp"

void InvKin_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&InvKin_node::timer_callback, this));
    pose_subscription = this->create_subscription<geometry_msgs::msg::Point>("target_position",10,
        std::bind(&InvKin_node::get_pose, this, _1));
    angles_publisher  = this->create_publisher<sensor_msgs::msg::JointState>("motor_commands",10);
}

void InvKin_node::timer_callback(){
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->get_clock()->now();
    msg.name = {"shoulder_updown","shoulder_joint","elbow","wrist","wrist_gripper_connection_roll","wrist_gripper_connection_pitch","gripper_left","gripper_right"};
    msg.position = {state[ZED],state[SHOULDER],state[ELBOW],state[YAW],0,0,0,0};

    angles_publisher->publish(msg);
}

void InvKin_node::get_pose(const geometry_msgs::msg::Point::SharedPtr msg){
    get_state(msg->x,msg->y,msg->z);
}


void InvKin_node::get_state(float x, float y, float z){
    // float L = 0.252; // TODO
    // float L = 500;

    // float angle_shoulder = acos((pow(x,2)+pow(y,2)-2*pow(L,2))/pow(L,2));
    // state[SHOULDER] = angle_shoulder*180/M_PI;

    // float angle_elbow = atan2(y,x) - asin(L*sin(angle_shoulder)/sqrt(pow(x,2)+pow(y,2)));
    // state[ELBOW] = angle_elbow*180/M_PI;

    // // targeted_z = z;
    // // state[ZED] = z;
    // state[ZED] = 0.5; // TODO test value to change

    
    
    // const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z));
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.3, 0.3, 0.5)); // TODO put real values

    // TODO : change neutral config for q (not sure finally)
    Eigen::VectorXd q = pinocchio::neutral(model);
    
    J.setZero();
    
    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);
    pinocchio::Data::Matrix6 JJt;
    for (int i=0;;i++)
    {
        pinocchio::forwardKinematics(model,data,q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();

        if(err.norm() < eps or i>=IT_MAX)
        {
            break;
        }

        pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);
        
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model,q,v*DT);
    }
    // cout << "before :" << q.transpose() << endl;

    correct_angle(q(1,0));
    correct_angle(q(2,0));
    correct_angle(q(3,0));

    state[SHOULDER] = q(1,0)*180/M_PI;
    state[ELBOW] = q(2,0)*180/M_PI;
    state[YAW] = q(3,0)*180/M_PI;

    // cout << "after :" << q.transpose() << endl;
    // cout << "###################" << endl;

}

void InvKin_node::correct_angle(double &angle){
    angle = fmod(angle , 2*M_PI);

    if (angle>M_PI){
        angle -= 2*M_PI;
    }

    else if (angle<-M_PI){
        angle += 2*M_PI;
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<InvKin_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}