#include "ros_interface_umi_rtx/arm_parts/arm.h"

Arm::Arm(){
    for (int id=0; id<NUMBER_OF_MOTORS; id++){
        Joint motor(id);
        addJoint(&motor);
    }
    initArm();
}

void Arm::addJoint(Joint *joint){
    mJoints.push_back(joint);
}

Joint* Arm::getJoint(int index) const{
    return mJoints[index];
}

void Arm::initArm(){
    FA1.set_parentJoint(mJoints[1]);
    FA1.set_childJoint(mJoints[0]);
    FA1.m_ID = 1;

    FA2.set_parentJoint(mJoints[0]);
    FA2.set_childJoint(mJoints[5]);
    FA2.m_ID = 2;

    FA3.set_parentJoint(mJoints[5]);
    FA3.set_childJoint(mJoints[6]);
    FA3.m_ID = 3;

    W = Wrist(mJoints[3],mJoints[4]);
}
