#ifndef OSR_MC_NODE_H
#define OSR_MC_NODE_H

#include <ros/ros.h>
#include <osr_msgs/Commands.h>
#include <osr_msgs/Status.h>
#include <osr_msgs/Encoder.h>

#include "../include/motorcontrollers.h"

using namespace std;
using namespace ros;

class osr_mc_node
{
public:
    osr_mc_node(NodeHandle &nh);
    ~osr_mc_node();

private:
    Subscriber m_cmdsSub;
    Publisher m_statusPub;
    Publisher m_encPub;

    void cmdCallback(const osr_msgs::Commands& msg);
    void publishStatus();
    void publishEnc();

    MotorControllers mc;

};
#endif
