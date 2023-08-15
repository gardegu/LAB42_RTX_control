#include "ros_interface_umi_rtx/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));

    pose_publisher  = this->create_publisher<geometry_msgs::msg::Pose>("target_pose",10);
    grip_publisher  = this->create_publisher<std_msgs::msg::Float32>("target_grip",10);

    pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>("processed_pose",10,
        std::bind(&Objective_node::get_processed_pose, this, _1));
    processed_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("processed_image",10,
        std::bind(&Objective_node::get_processed_image, this, _1));
    depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("depth_image",10,
        std::bind(&Objective_node::get_depth_image, this, _1));
}

void Objective_node::timer_callback(){
    double dt1 = 8;
    /*
    We follow a simple trajectory between each step :
        - Open the grip and goes to the target
        - Close the grip to grab the object
        - Go back to initial place
        - Wait
        - Same trajectory but we put the object back at his place
    */
    if (mode != "manual"){  
        // Lissajou();
        // x = processed_x;
        // y = processed_y;
        // z = processed_z;
        // yaw = atan2(y,x)*180/M_PI;
        
        if (!is_initialized){
            target_x = processed_x;
            target_y = processed_y;
            target_z = processed_z;

            final_x = 0.3;
            final_y = 0.39;
            final_z = 0.2;
            is_initialized=true;
        }
        
        if ((t-t0)<dt1){
            x = x0 + (target_x-x0)*(t-t0)/dt1;
            y = y0 + (target_y-y0)*(t-t0)/dt1;
            z = z0 + (target_z-z0)*(t-t0)/dt1;

            pitch = pitch0 + (90.-pitch0)*(t-t0)/dt1;
            roll = roll0 + (0.-roll0)*(t-t0)/dt1;
            grip = 0.08;
        }

        else if ((t-t0)>=dt1 and (t-t0)<14){
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
            grip = 0.02;
        }

        else if ((t-t0)>=14 and (t-t0)<20){
            x = x0 + (0.-x0)*(t-t0-14)/6;
            y = y0 + (0.5-y0)*(t-t0-14)/6;
            z = z0 + (0.8-z0)*(t-t0-14)/6;

            pitch = pitch0 + (0.-pitch0)*(t-t0-14)/6;
            roll = roll0 + (0.-roll0)*(t-t0-14)/6;
        } 

        else if ((t-t0)>=20 and (t-t0)<26){
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
        } 

        else if ((t-t0)>=26 and (t-t0)<40){
            x = x0 + (final_x-x0)*(t-t0-26)/6;
            y = y0 + (final_y-y0)*(t-t0-26)/6;
            z = z0 + (final_z-z0)*(t-t0-26)/6;

            pitch = pitch0 + (0.-pitch0)*(t-t0-26)/6;
            roll = roll0 + (0.-roll0)*(t-t0-26)/6;
        } 

        else if ((t-t0)>=40 and (t-t0)<44){
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
            grip = 0.08;
        }

        else if ((t-t0)>=44 and (t-t0)<50){
            x = x0 + (0.-x0)*(t-t0-44)/6;
            y = y0 + (0.5-y0)*(t-t0-44)/6;
            z = z0 + (0.8-z0)*(t-t0-44)/6;

            pitch = pitch0 + (0.-pitch0)*(t-t0-44)/6;
            roll = roll0 + (0.-roll0)*(t-t0-44)/6;
            grip = 0.08 + (0.02-0.08)*(t-t0-44)/6;
        } 
    }

    else {
        is_initialized=false;
        /*
        If the arm is controlled manually we adapt the origin pose of the automatical procedure of grab mode
        */
        x0 = x;
        y0 = y;
        z0 = z;

        roll0 = roll;
        pitch0 = pitch;

        t0 = t;
    }
    t+=dt;

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = z;

    pose_msg.orientation.x = yaw; 
    pose_msg.orientation.y = pitch;
    pose_msg.orientation.z = roll;

    pose_publisher->publish(pose_msg);

    std_msgs::msg::Float32 grip_msg;
    grip_msg.data = grip;
    grip_publisher->publish(grip_msg);
}

void Objective_node::get_processed_pose(const geometry_msgs::msg::Pose::SharedPtr msg){
    processed_x = msg->position.x;
    processed_y = msg->position.y;
    processed_z = msg->position.z;

    processed_yaw = msg->orientation.x;
    processed_pitch = msg->orientation.y;
    processed_roll = msg->orientation.z;
}

void Objective_node::get_processed_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    processed_frame = cv_ptr->image;
}

void Objective_node::get_depth_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    depth_frame = cv_ptr->image;
}

void Objective_node::update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip){
    x = new_x;
    y = new_y;
    z = new_z;
    yaw = new_yaw;
    pitch = new_pitch;
    roll = new_roll;
    grip = new_grip;
}