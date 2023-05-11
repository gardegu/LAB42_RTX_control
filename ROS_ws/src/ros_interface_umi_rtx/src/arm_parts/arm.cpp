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
    FA1.set_parentJoint(mJoints[SHOULDER]);
    FA1.set_childJoint(mJoints[ELBOW]);
    FA1.m_ID = 1;

    FA2.set_parentJoint(mJoints[ELBOW]);
    FA2.set_childJoint(mJoints[YAW]);
    FA2.m_ID = 2;

    // This one is probably wrong
    FA3.set_parentJoint(mJoints[YAW]);
    FA3.set_childJoint(mJoints[GRIP]);
    FA3.m_ID = 3;

    W = Wrist(mJoints[WRIST1],mJoints[WRIST2]);
}

vector<int> Arm::getMotorState(int ID){
    Joint* motor = mJoints[ID];

    vector<int> state;

    int value;

    for (int PID=0; PID<NUMBER_OF_DATA_CODES; PID++){
        if (motor->get_parameter(PID,&value)){
            state.push_back(value);
        }
    }
    return state;

}
