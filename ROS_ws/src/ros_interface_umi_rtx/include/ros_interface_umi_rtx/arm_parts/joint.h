#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <vector>

#include "ros_interface_umi_rtx/arm_parts/forearm.h"

using namespace std;

class Joint {
public:
    Joint(int ID);

    void setChild(ForeArm* child);
    void setParent(ForeArm* parent);

    ForeArm* getChild();
    ForeArm* getParent();

    vector<float> getOrientation() const;
    vector<float> getPosition() const;

    void setOrientation(const int increment);
    void setPosition(const int increment);

    const string getName() const;

    int m_ID;

private:
    string m_Name;
    vector<int> m_parameters;
    vector<float> m_orientation, m_position;

    ForeArm *Child=nullptr, *Parent=nullptr;

};


#endif