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
d4 = 0.2559

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
    global d4

    global pub
    global odomBroadcaster
    global base_frame
    
    dt = (current_time - last_time).to_sec()

    d_left = mtp * (encs[1] - encs_prev[1])
    d_right = mtp * (encs[4] - encs_prev[4])
    rospy.loginfo("d left: " + str(d_left) + " d right: " + str(d_right))

    # rospy.loginfo("d_left: " + str(d_left) + " d_right: " + str(d_right) + " dt: " + str(dt) + " 1:" + str(encs[1]) + " 1 pre: " + str(encs_prev[1]) +
    #     " 4: " + str(encs[4]) + " 4 pre: " + str(encs_prev[4]))
    
    dth = (d_right - d_left) / wheel_track

    # calculate radius of turn
    if (dth != 0 and d_left != 0 and d_right != 0):
        lv = d4 + d_left / d_right * d4
        # print ("lv: " + str(lv))
        r = lv / (1  - (d_left / d_right))
    else:
        r = 0

    dist = (d_left + d_right) / 2

    # calculate angle change    
    if (r != 0):
        d_theta = dist / -r
    else:
        d_theta = 0

    # calculate centre of turn circle
    x_orig = x + r * math.cos(th)
    y_orig = y + r * math.sin(th)

    # calculate new co-ordinates    
    x_new = x_orig + (x - x_orig) * math.cos(d_theta) - (y - y_orig) * math.sin(d_theta)
    y_new = y_orig + (x - x_orig) * math.sin(d_theta) + (y - y_orig) * math.cos(d_theta)

    #calculate change in x,y values
    dx = x_new - x
    dy = y_new - y     


    th += d_theta

    if (th > (math.pi * 2)):
        th -= (math.pi * 2)
    elif (th < (-math.pi * 2)):
        th += (math.pi * 2)


    x = x_new
    y = y_new

    # convert to ros co-ords
    x_ros = y
    y_ros = -x

    vx_ros = dy / dt
    vy_ros = -dx / dt

    vth = d_theta /dt

    rospy.loginfo("x orig: " + str(x_orig) + " y orig: " + str(y_orig) + " d_theta: " + str(d_theta) + " dist: " + str(dist))
    rospy.loginfo("x new: " + str(x_new) + " y new: " + str(y_new) + " theta: " + str(th) + " r: " + str(r))

    quaternion = Quaternion()
    quaternion.x  = 0.0
    quaternion.y  = 0.0
    quaternion.z = math.sin(th / 2.0)
    quaternion.w = math.cos(th / 2.0)

    odomBroadcaster.sendTransform(
        (x_ros, y_ros, 0),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        current_time,
        base_frame,
        "odom"
    )

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = base_frame
    odom.header.stamp = current_time
    odom.pose.pose.position.x = x_ros
    odom.pose.pose.position.y = y_ros
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = quaternion
    odom.twist.twist.linear.x = vx_ros
    odom.twist.twist.linear.y = vy_ros
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.z = vth

    pub.publish(odom)

    last_time = current_time
    encs_prev = encs
    

def enc_callback(message):	
    global encs
    global enc_valid
    global current_time

    encs = message.rel_enc
    enc_valid = True

    current_time = rospy.Time.now()

    calculateOdometry()



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
    # current_time = rospy.Time.now()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # current_time = rospy.Time.now()

        # if (enc_valid):            
        #     calculateOdometry()	

        rate.sleep()	


