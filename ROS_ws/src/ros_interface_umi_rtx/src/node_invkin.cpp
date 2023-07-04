#include "ros_interface_umi_rtx/node_invkin.hpp"

void InvKin_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&InvKin_node::timer_callback, this));
    position_subscription = this->create_subscription<geometry_msgs::msg::Point>("target_position",10,
        std::bind(&InvKin_node::get_position, this, _1));
    angles_subscription = this->create_subscription<geometry_msgs::msg::Vector3>("target_angles",10,
        std::bind(&InvKin_node::get_angles, this, _1));
    grip_subscription = this->create_subscription<std_msgs::msg::Float32>("target_grip",10,
        std::bind(&InvKin_node::get_grip, this, _1));
    angles_publisher  = this->create_publisher<sensor_msgs::msg::JointState>("motor_commands",10);
}

void InvKin_node::timer_callback(){
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->get_clock()->now();
    msg.name = {"shoulder_updown","shoulder_joint","elbow","wrist","wrist_gripper_connection_roll","wrist_gripper_connection_pitch","gripper_left","gripper_right"};
    msg.position = {state[ZED],state[SHOULDER],state[ELBOW],state[YAW],state[ROLL],state[PITCH],0.05-target_grip/2,-0.05+target_grip/2};

    angles_publisher->publish(msg);
}

void InvKin_node::get_position(const geometry_msgs::msg::Point::SharedPtr msg){
    double x,y,z;
    // Frame coordinates to avoid positions that are too far apart
    x = max(-0.6,min(0.6,msg->x));
    y = max(0.1,min(0.69,msg->y));
    z = max(0.1,min(0.7,msg->z));
    double theta = atan2(y,x);
    double r = sqrt(pow(x,2)+pow(y,2));
    double rmax = 0.69;
    if (r>rmax){
        x = rmax*cos(theta);
        y = rmax*sin(theta);
    }

    get_state(x,y,z);
}

void InvKin_node::get_angles(const geometry_msgs::msg::Vector3::SharedPtr msg){
    target_yaw = msg->x*M_PI/180;
    target_pitch = msg->y*M_PI/180;
    target_roll = msg->z*M_PI/180;
}

void InvKin_node::get_grip(const std_msgs::msg::Float32::SharedPtr msg){
    target_grip = msg->data;
}

void InvKin_node::get_state(double x, double y, double z){    

    // If coordinates didn't change, we don't compose the inverse kinematics
    if (x!=last_x or y!=last_y or z!=last_z or target_yaw!=last_yaw or target_pitch!=last_pitch or target_roll!=last_roll){ // Avoid calculation when the position doesn't change
        last_x = x;
        last_y = y;
        last_z = z;
        last_yaw = target_yaw;
        last_pitch = target_pitch;
        last_roll = target_roll;
        
        // z -= 0.455; // Adapt to the z-origin of the urdf file
        z -= 0.626;
        double yaw=atan2(y,x), roll=target_roll, pitch=target_pitch;
        // double yaw=target_yaw, roll=target_roll, pitch=target_pitch;

        // Translation to put the target at the tip of the hand
        x -= L*cos(pitch)*cos(yaw);
        y -= L*cos(pitch)*sin(yaw);
        z += L*sin(pitch);

        Eigen::Matrix3d mat_yaw, mat_roll, mat_pitch;
        mat_yaw << cos(yaw),-sin(yaw),0,
                   sin(yaw), cos(yaw),0,
                   0       , 0       ,1;

        mat_roll << 1, 0        ,0         ,
                    0, cos(roll),-sin(roll),
                    0, sin(roll), cos(roll);

        mat_pitch <<  cos(pitch), 0, sin(pitch),
                      0         , 1, 0         ,
                     -sin(pitch), 0, cos(pitch);
        
        Eigen::Vector3d pos(x,y,z);

        const pinocchio::SE3 oMdes(mat_yaw*mat_pitch*mat_roll, pos); 

        q = pinocchio::neutral(model);
        if (state.size()>=4){ // Initial state = last state processed
            q(0,0) = state[ZED];
            q(1,0) = state[SHOULDER]*M_PI/180;
            q(2,0) = state[ELBOW]*M_PI/180;
            q(3,0) = state[YAW]*M_PI/180;
            q(4,0) = state[ROLL]*M_PI/180;
            q(5,0) = state[PITCH]*M_PI/180;
        }

        J.setZero();

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

        correct_angle(q);
        state[ZED] = q(0,0);
        state[SHOULDER] = q(1,0);
        state[ELBOW] = q(2,0);
        state[YAW] = q(3,0);
        state[ROLL] = q(4,0);
        state[PITCH] = q(5,0);
    }

}

// Put the angles in [-180°,180]
void InvKin_node::correct_angle(Eigen::VectorXd &q){

    for (int i=1; i<q.size(); i++){
        q(i,0) = fmod(q(i,0) , 2*M_PI);

        if (q(i,0)>M_PI){
            q(i,0) -= 2*M_PI;
        }

        else if (q(i,0)<-M_PI){
            q(i,0) += 2*M_PI;
        }
        q(i,0) = q(i,0)*180/M_PI;
    }
    
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<InvKin_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}