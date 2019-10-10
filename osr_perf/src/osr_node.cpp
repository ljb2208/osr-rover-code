#include <ros/ros.h>
#include "../include/osr_node.h"
#include "../include/robot.h"

using namespace std;
using namespace ros;


// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "osr_node");
  NodeHandle node;

  osr_node osr_node(node);

  int publish_freq;

  node.param<int>("publish_freq", publish_freq, 1);

  ros::Rate r(publish_freq);
  while (ros::ok())
  {     
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


osr_node::osr_node(NodeHandle &nh)
{
    // setup commands publisher
    m_cmdsPub = nh.advertise<osr_msgs::Commands>("robot_commands", 1);

    // setup subscribers
    m_jsSub = nh.subscribe("joystick", 5,
                              &osr_node::jsCallback,
                              this);

    m_encSub = nh.subscribe("encoder", 5,
                            &osr_node::encCallback,
                            this);

    
    // fix - get from ros param
    r.setDistances(7.254,10.5,10.5,10.073);

    // fix - get from ros param
    int mins[4] = {550,420,360,325};
    int max[4] = {1480,1755,1240,1230};

    r.setEncoderValues(mins, max);

}

osr_node::~osr_node()
{

}

void osr_node::jsCallback(const osr_msgs::Joystick& msg)
{
    int32_t vel_cmds[6];
    int32_t ticks[4];
    r.generateCommands(msg.vel, msg.steering, enc, vel_cmds, ticks);

    osr_msgs::Commands cmd_msg;

    for (int i=6; i > 0; i--)
    {
      cmd_msg.drive_motor.push_back(static_cast<int64_t>(vel_cmds[i]));
    }

    for (int i=4; i > 0; i--)
    {
      cmd_msg.corner_motor.push_back(static_cast<int64_t>(ticks[i]));
    }

    m_cmdsPub.publish(cmd_msg);
}

void osr_node::encCallback(const osr_msgs::Encoder& msg)
{
  if (msg.abs_enc.size() < 4)
  {
    ROS_ERROR_STREAM("Encoder message has less than 4 elements.\r\n");
    return;
  }

  for (int i=0; i < 4; i++)
  {
    enc[i] = static_cast<int32_t>(msg.abs_enc.at(i));
  }
}