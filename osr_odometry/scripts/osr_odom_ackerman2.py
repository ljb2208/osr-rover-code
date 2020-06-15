#!/usr/bin/env python
import time
from osr_msgs.msg import Joystick, Commands, Encoder, RunStop
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
import tf
import math
import numpy

class Odometry2():
    def __init__(self, baseFrame, wheelTrack, mpt, d4, maxTickPerSec, pubTF=False):
        self.encValid = False
        self.priorTime = rospy.Time.now()
        self.priorEncs = [0,0,0,0,0,0]
        self.mpt = mpt
        self.pubTF = pubTF
        
        # distance between wheels
        self.wheelTrack = wheelTrack
        self.d4 = d4
        self.baseFrame = baseFrame
        self.maxTickPerSec = maxTickPerSec

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.odomPub = rospy.Publisher("/odom", Odometry, queue_size = 1)    
        
        if self.pubTF:
            self.odomBroadcaster = tf.TransformBroadcaster() 

        self.twistCovar = numpy.diag([0.001, 0.001, 0.001, 0.1, 0.1, 0.1]).ravel()
        self.poseCovar = numpy.diag([0.001, 0.001, 0.001, 0.1, 0.1, 0.1]).ravel()


    def onEncoderMessage(self, message):
        self.calculateOdometry(message)

    def isValid(self, message):
        dencLeft = abs(message.rel_enc[1] - self.priorEncs[1])
        dencRight = abs(message.rel_enc[4] - self.priorEncs[4])

        dt = self.getElapsedTime(message.header.stamp)

        if (dencLeft/dt) > self.maxTickPerSec:
            rospy.logwarn("Invalid relative encoder value on left wheel. No odom calculated")
            return False
        
        if (dencRight/dt) > self.maxTickPerSec:
            rospy.logwarn("Invalid relative encoder value on right wheel. No odom calculated")
            return False

        return True

    def publishTransform(self, x, y, quaternion, timestamp):        
        self.odomBroadcaster.sendTransform(
            (x, y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            timestamp,
            self.baseFrame,
            "odom")

    def publishOdomMessage(self, x, y, vx, vy, vth, quaternion, timestamp):
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.baseFrame
        odom.header.stamp = timestamp
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.covariance = self.poseCovar
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = self.twistCovar        

        self.odomPub.publish(odom)

    def getElapsedTime(self, timestamp, save=False):
        dt = (timestamp - self.priorTime).to_sec()

        if save:
            self.priorTime = timestamp

        return dt

    def calculateTurnRadius(self, dLeft, dRight):
        dlr = dLeft - dRight

         # calculate radius of turn
        if dlr != 0 and dLeft != 0 and dRight != 0:
            lv = self.d4 + dLeft / dRight * self.d4
            # print ("lv: " + str(lv))
            r = lv / (1  - (dLeft / dRight))
        else:
            r = 0

        dist = (dLeft + dRight) / 2

        # calculate angle change    
        if (r != 0):
            dTheta = dist / -r
        else:
            dTheta = 0

        return r, dTheta

    def calculateOdometry(self, message):
        currentTime = message.header.stamp
        encs = message.rel_enc                

        if not self.isValid(message):
            return

        dt = self.getElapsedTime(currentTime, save=True)

        dLeft = self.mpt * (encs[1] - self.priorEncs[1])
        dRight = self.mpt * (encs[4] - self.priorEncs[4])
    
        # dth = (dRight - dLeft) / self.wheelTrack

        radius, dTheta = self.calculateTurnRadius(dLeft, dRight)

        # calculate centre of turn circle
        xOrig = self.x + radius * math.cos(self.th)
        yOrig = self.y + radius * math.sin(self.th)

        # calculate new co-ordinates    
        xNew = xOrig + (self.x - xOrig) * math.cos(dTheta) - (self.y - yOrig) * math.sin(dTheta)
        yNew = yOrig + (self.x - xOrig) * math.sin(dTheta) + (self.y - yOrig) * math.cos(dTheta)

        #calculate change in x,y values
        dx = xNew - self.x
        dy = yNew - self.y     

        self.th += dTheta

        if (self.th > (math.pi * 2)):
            self.th -= (math.pi * 2)
        elif (self.th < (-math.pi * 2)):
            self.th += (math.pi * 2)

        self.x = xNew
        self.y = yNew

        # convert to ros co-ords
        xRos = self.y
        yRos = -self.x

        vxRos = dy / dt
        vyRos = -dx / dt

        vth = dTheta /dt

        quaternion = self.getQuaternion(self.th)
        
        if self.pubTF:
            self.publishTransform(xRos, yRos, quaternion, currentTime)
            
        self.publishOdomMessage(xRos, yRos, vxRos, vyRos, vth, quaternion, currentTime)   

        self.priorEncs = encs    
        

    def getQuaternion(self, th):
        quaternion = Quaternion()
        quaternion.x  = 0.0
        quaternion.y  = 0.0
        quaternion.z = math.sin(th / 2.0)
        quaternion.w = math.cos(th / 2.0)

        return quaternion


if __name__ == '__main__':    

    rospy.init_node('osr_odometry2')
    rospy.loginfo("Starting the osr odometry2 node")	

    baseFrame = rospy.get_param("/odometry/base_frame_id", "base_link")
    # mpt = rospy.get_param("/odometry/mpt", 0.000026322)
    mpt = rospy.get_param("/odometry/mpt",  0.000045777)     
    wheelTrack = rospy.get_param("/odometry/wheel_track", 0.455)
    d4 = rospy.get_param("/odometry/d4", 0.2559)
    maxTickPerSec = rospy.get_param("/odometry/maxTickPerSec", 8000)        
    publishTF = rospy.get_param("~publishTF", False)

    odom = Odometry2(baseFrame, wheelTrack, mpt, d4, maxTickPerSec, pubTF=publishTF)
    encSub  = rospy.Subscriber("/encoder", Encoder, odom.onEncoderMessage)	    

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():        
        rate.sleep()	


