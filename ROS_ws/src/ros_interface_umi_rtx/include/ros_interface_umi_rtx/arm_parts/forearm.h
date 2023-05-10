#ifndef __FOREARM_H__
#define __FOREARM_H__

#include <vector>
#include "ros_interface_umi_rtx/arm_parts/joint.h"


class ForeArm {
public:
    ForeArm(int id, Joint *parent_joint, Joint *child_joint);
    ForeArm();
    
    const float m_length = 0.3; // TODO : put the right length
    int m_ID;

    Joint* get_parentJoint() const;
    Joint* get_childJoint() const;

    void set_parentJoint(Joint *parent);
    void set_childJoint(Joint *child);


private:

    Joint *m_parent_joint=nullptr, *m_child_joint=nullptr;
    
};

#endif