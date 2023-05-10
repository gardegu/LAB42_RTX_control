#include "ros_interface_umi_rtx/arm_parts/wrist.h"


Wrist::Wrist(Joint *W1, Joint *W2){
    m_W1 = W1;
    m_W2 = W2;
}