#ifndef __WRIST_H__
#define __WRIST_H__

#include "ros_interface_umi_rtx/arm_parts/joint.h"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"

class Wrist {
public:
    Wrist(Joint *W1, Joint *W2);
    Wrist();

    void rotate(const float increment_angle); // Rotation on itself
    void turn(const float increment_angle); // Rotation on the arm

private:
    Joint *m_W1, *m_W2;
};

#endif