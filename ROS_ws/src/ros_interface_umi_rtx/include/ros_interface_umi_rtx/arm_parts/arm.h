#ifndef __ARM_H__
#define __ARM_H__

#include <vector>
#include <map>

#include "ros_interface_umi_rtx/arm_parts/joint.h"
#include "ros_interface_umi_rtx/arm_parts/forearm.h"
#include "ros_interface_umi_rtx/arm_parts/wrist.h"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"

using namespace std;


class Arm {
public:
    Arm();

    void addJoint(Joint *joint);
    Joint* getJoint(int index) const;

    void initArm();

    vector<int> getMotorState(int ID);

private:
    string mName;
    vector<Joint*> mJoints;

    ForeArm FA1,FA2,FA3;
    Wrist W;
};

#endif