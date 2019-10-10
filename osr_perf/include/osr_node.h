#ifndef OSR_NODE_H
#define OSR_NODE_H

#include <ros/ros.h>
#include <osr_msgs/Commands.h>
#include <osr_msgs/Joystick.h>
#include <osr_msgs/Encoder.h>

#include "../include/robot.h"

using namespace std;
using namespace ros;

class osr_node
{
public:
    osr_node(NodeHandle &nh);
    ~osr_node();

private:
    Publisher m_cmdsPub;
    Subscriber m_jsSub;
    Subscriber m_encSub;

    void jsCallback(const osr_msgs::Joystick& msg);
    void encCallback(const osr_msgs::Encoder& msg);

    int32_t enc[4];
    robot r;

};
#endif
