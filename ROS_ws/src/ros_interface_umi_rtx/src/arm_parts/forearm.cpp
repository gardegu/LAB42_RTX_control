#include "ros_interface_umi_rtx/arm_parts/forearm.h"

ForeArm::ForeArm(int id, Joint *parent_joint, Joint *child_joint){
    m_ID = id;
    m_parent_joint = parent_joint;
    m_child_joint = child_joint;

    m_parent_joint->setChild(this);
    m_child_joint->setParent(this);
}

ForeArm::ForeArm(){
}

Joint* ForeArm::get_parentJoint() const {
    return m_parent_joint;
}

Joint* ForeArm::get_childJoint() const {
    return m_child_joint;
}

void ForeArm::set_parentJoint(Joint *parent){
    m_parent_joint = parent;
}

void ForeArm::set_childJoint(Joint *child){
    m_child_joint = child;
}