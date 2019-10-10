#include <ros/ros.h>
#include "../include/osr_mc_node.h"


using namespace std;
using namespace ros;


// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "osr_mc_node");
  NodeHandle node;

  osr_mc_node osr_mc_node(node);

  int publish_freq;

//   joy_sub = rospy.Subscriber("/joystick",Joystick, joy_callback)
// 	enc_sub  = rospy.Subscriber("/encoder", Encoder, enc_callback)
// 	rate = rospy.Rate(10)
// 	#time_sync = message_filters.TimeSynchronizer([joy_sub, mc_sub],10)
// 	#time_sync.registerCallback(callback)

// 	pub = rospy.Publisher("/robot_commands", Commands, queue_size = 1)

  node.param<int>("publish_freq", publish_freq, 1);

  

  ros::Rate r(publish_freq);
  while (ros::ok())
  {     
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


osr_mc_node::osr_mc_node(NodeHandle &nh)
{
    mc.connect();

    // setup commands publisher
    m_statusPub = nh.advertise<osr_msgs::Commands>("status", 1);
    m_encPub = nh.advertise<osr_msgs::Encoder>("encoder", 1);

    // setup subscribers
    m_cmdsSub = nh.subscribe("commands", 5,
                            &osr_mc_node::cmdCallback,
                            this);

    Rate r(10); 

    int counter = 0;

    while (ok())
    {
        publishEnc();

        if (counter == 10)
        {
            publishStatus();
            counter = 0;
        }

        spinOnce();
        r.sleep();
        counter++;
    }
}

void osr_mc_node::publishStatus()
{
    osr_msgs::Status msg;
    msg.battery = static_cast<int64_t>(mc.getBattery(0));

    int addr_count = mc.getAddrCount();
    uint16_t temps[addr_count];
    mc.getTemp(temps);

    for (int i=0; i < addr_count; i++)
    {
        msg.temp[i] = static_cast<int64_t>(temps[i]);
    }

    int16_t currs[addr_count * 2];

    mc.getCurrents(currs);

    for (int i=0; i < addr_count * 2; i++)
    {
        msg.current[i] = static_cast<int64_t>(currs[i]);
    }

    uint16_t errors[addr_count];
    mc.getErrors(errors);

    for (int i=0; i < addr_count; i++)
    {
        msg.error_status[i] = static_cast<int64_t>(errors[i]);
    }

    m_statusPub.publish(msg);
}

void osr_mc_node::publishEnc()
{
    osr_msgs::Encoder msg;

    uint8_t encVals[4];
    mc.getCornerEnc(encVals);

    for (int i=4; i > 0; i--)
    {
        msg.abs_enc.push_back(static_cast<int64_t>(encVals[i]));
    }

    m_encPub.publish(msg);
}

osr_mc_node::~osr_mc_node()
{

}

void osr_mc_node::cmdCallback(const osr_msgs::Commands& msg)
{
    int size = msg.corner_motor.size();
    int32_t ticks[4];

    for (int i=0; i < size; i++)
    {
        ticks[i] = static_cast<int32_t>(msg.corner_motor.at(i));
    }

    mc.cornerToPosition(ticks);

    size = msg.drive_motor.size();

    for (int i=0; i < size; i++)
    {
        mc.sendSignedDutyAccel(i, static_cast<int32_t>(msg.drive_motor.at(i)));
    }
}
