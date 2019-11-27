#!/usr/bin/env python
import time
from osr_msgs.msg import Joystick, Commands, Encoder, RunStop
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
import tf


global pub
global odomBroadcaster

global encs
global encs_prev

encs = [0]*6
encs_prev = [0]*6

global x
global y
global th

global mpt 
global last_time
global current_time
global wheel_track

global base_frame

x = 0.0
y = 0.0
th = 0.0

# meters per tick
mtp = 0.000579 
# distance between wheels
wheel_track = 0.455

base_frame = "base_link"


def calculateOdometry():
    global last_time
    global current_time
    global encs
    global encs_prev
    global wheel_track
    global x
    global y
    global th

    global pub
    global odomBroadcaster
    global base_frame
    
    dt = (current_time - last_time).to_sec()

    d_left = mtp * (encs[1] - encs_prev[1])
    d_right = mtp * (encs[4] - encs_prev[4])

    dxy_avg = (d_left + d_right) / 2.0
    dth = (d_right - d_left) / wheel_track

    vxy = dxy_avg / dt
    vth = dth / dt

    if (dxy_avg != 0):
        dx = cos(dth) * dxy_avg
        dy = -sin(dth) * dxy_avg
        x += (cos(th) * dx - sin(th) * dy)
        y += (sin(th) * dy + cos(th) * dy)

    if (dth != 0):
        th += dth

    quaternion = Quaternion()
    quaternion.x  = 0.0
    quaternion.y  = 0.0
    quaternion.z = sin(th / 2.0)
    quaternion.w = cos(th / 2.0)

    odomBroadcaster.sendTransform(
        (x, y, 0),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        current_time,
        base_frame,
        "odom"
    )

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = base_frame
    odom.header.stamp = current_time
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = quaternion
    odom.twise.twist.linear.x = vxy
    odom.twist.twist.linear.y = o
    odom.twist.twist.andular.z = vth

    pub.publish(odom)

    last_time = current_time
    encs_prev = encs
    

def enc_callback(message):	
	global encs
	encs = message.rel_enc


if __name__ == '__main__':
	rospy.init_node('osr_odometry')
	rospy.loginfo("Starting the osr odometry node")
	global pub	
    global odomBroadcaster
	enc_sub  = rospy.Subscriber("/encoder", Encoder, enc_callback)	
	pub = rospy.Publisher("/odom", Odometry, queue_size = 1)
    odomBroadcaster = TransformBroadcaster() 

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        calculateOdometry()
	
	rate.sleep()	


