#include "ros_interface_umi_rtx/arm_parts/joint.h"


using namespace std;

Joint::Joint(int ID){
    m_ID = ID;
}

Joint::Joint(){

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
    return arm_read(m_ID, PID, value);
}

void Joint::setOrientation(const float increment_angle){
    map<int,float> conv_map = {{ELBOW,CONV_ELBOW},
                            {SHOULDER,CONV_SHOULDER},
                            {WRIST1,CONV_W},
                            {WRIST2,CONV_W},
                            {YAW,CONV_YAW}};
    float conv_ticks_to_deg = conv_map[m_ID];
    int increment_ticks = increment_angle/conv_ticks_to_deg;

    arm_write(m_ID,NEW_POSITION,increment_ticks);
}

void Joint::setZed(const float zed){

    // cout << "zed :" << zed << endl;
    int increment_ticks = -(zed)/CONV_ZED;
    // cout << "ticks :" << increment_ticks << endl;
    arm_write(m_ID,NEW_POSITION,increment_ticks);
}