#!/usr/bin/env python
import time
from osr_msgs.msg import Joystick, Commands, Encoder, RunStop
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
import tf
import math


global pub
global odomBroadcaster
global enc_valid

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


# meters per tick based on 152 mm wheel diameter and 18140.79 ticks per revolution
mtp = 0.000026322
# distance between wheels
wheel_track = 0.455

base_frame = "base_footprint"


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

    # rospy.loginfo("d_left: " + str(d_left) + " d_right: " + str(d_right) + " dt: " + str(dt) + " 1:" + str(encs[1]) + " 1 pre: " + str(encs_prev[1]) +
    #     " 4: " + str(encs[4]) + " 4 pre: " + str(encs_prev[4]))

    dxy_avg = (d_left + d_right) / 2.0
    dth = (d_right - d_left) / wheel_track

    vxy = dxy_avg / dt
    vth = dth / dt

    if (dxy_avg != 0):
        dx = math.cos(dth) * dxy_avg
        dy = -math.sin(dth) * dxy_avg
        x += (math.cos(th) * dx - math.sin(th) * dy)
        y += (math.sin(th) * dy + math.cos(th) * dy)
        # rospy.loginfo("dx: " + str(dx) + "dy: " + str(dy) + " dxy_avg: " + str(dxy_avg))

    if (dth != 0):
        th += dth

    

    quaternion = Quaternion()
    quaternion.x  = 0.0
    quaternion.y  = 0.0
    quaternion.z = math.sin(th / 2.0)
    quaternion.w = math.cos(th / 2.0)

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
    odom.twist.twist.linear.x = vxy
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = vth

    pub.publish(odom)

    last_time = current_time
    encs_prev = encs
    

def enc_callback(message):	
    global encs
    global enc_valid
    encs = message.rel_enc
    enc_valid = True



if __name__ == '__main__':
    global pub	
    global odomBroadcaster
    global enc_valid

    rospy.init_node('osr_odometry')
    rospy.loginfo("Starting the osr odometry node")	
    enc_sub  = rospy.Subscriber("/encoder", Encoder, enc_callback)	
    pub = rospy.Publisher("/odom", Odometry, queue_size = 1)
    odomBroadcaster = tf.TransformBroadcaster() 

    enc_valid = False

    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        if (enc_valid):            
            calculateOdometry()	

        rate.sleep()	


