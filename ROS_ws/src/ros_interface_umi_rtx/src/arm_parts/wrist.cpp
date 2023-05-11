#include "ros_interface_umi_rtx/arm_parts/wrist.h"


Wrist::Wrist(Joint *W1, Joint *W2){
    m_W1 = W1;
    m_W2 = W2;
}

Wrist::Wrist(){
}

void Wrist::rotate(const float increment_angle){
    int increment_ticks = increment_angle/CONV_W;

    arm_write(WRIST1,NEW_POSITION,increment_ticks/2);
    arm_write(WRIST2,NEW_POSITION,-increment_ticks/2);
}

void Wrist::turn(const float increment_angle){
    int increment_ticks = increment_angle/CONV_W;

    arm_write(WRIST1,NEW_POSITION,increment_ticks);
    arm_write(WRIST2,NEW_POSITION,increment_ticks);
}