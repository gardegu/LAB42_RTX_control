#include "ros_interface_umi_rtx/arm_parts/joint.h"


using namespace std;

Joint::Joint(int ID){
    m_ID = ID;
}

void Joint::setChild(ForeArm* child){
    Child = child;
}

void Joint::setParent(ForeArm* parent){
    Parent = parent;
}

ForeArm* Joint::getChild(){
    return Child;
}

ForeArm* Joint::getParent(){
    return Parent;
}

vector<float> Joint::getOrientation() const{
    return m_orientation;
}

vector<float> Joint::getPosition() const{
    return m_position;
}

const string Joint::getName() const{
    return m_Name;
}

int Joint::get_parameter(int PID, int *value){
    return arm_read(m_ID, PID, &value);
}