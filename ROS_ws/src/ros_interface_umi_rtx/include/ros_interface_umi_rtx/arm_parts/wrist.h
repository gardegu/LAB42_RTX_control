#ifndef __WRIST_H__
#define __WRIST_H__

#include "ros_interface_umi_rtx/arm_parts/forearm.h"
#include "ros_interface_umi_rtx/arm_parts/joint.h"

class Wrist {
public:
    Wrist(Joint *W1, Joint *W2);
    Wrist();

private:
    Joint *m_W1, *m_W2;
};

#endif