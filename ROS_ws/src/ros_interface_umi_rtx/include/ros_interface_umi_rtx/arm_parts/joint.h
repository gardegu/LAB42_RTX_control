#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include "ros_interface_umi_rtx/arm_parts/forearm.h"

#include "ros_interface_umi_rtx/umi-drivers/armraw.h"
#include "ros_interface_umi_rtx/umi-drivers/armlib.h"
#include "ros_interface_umi_rtx/umi-drivers/rtx.h"


using namespace std;


// To avoid cyclic dependency
class ForeArm;
class Joint {
public:
    Joint(int ID);
    Joint();

    void setChild(ForeArm* child);
    void setParent(ForeArm* parent);

    ForeArm* getChild();
    ForeArm* getParent();

    vector<float> getOrientation() const;
    vector<float> getPosition() const;

    void setOrientation(const float increment_angle);

    const string getName() const;

    int m_ID;

    int get_parameter(int PID, int *value);

private:
    string m_Name;
    vector<int> m_parameters;
    vector<float> m_orientation, m_position;

    ForeArm *Child=nullptr, *Parent=nullptr;

};


#endif