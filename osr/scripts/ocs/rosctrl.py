from tkinter import *
from tkinter import ttk
import rospy
import rosnode
import roslaunch
import queue
import threading
import gc
import os
import math
import subprocess
import time
from osr_msgs.msg import RunStop, Status, Encoder
from nav_msgs.msg import Odometry
from functools import partial

class rosLoop():
    def __init__(self, ocsQueue, rosQueue):
        self.ocsQueue = ocsQueue
        self.rosQueue = rosQueue
        self.setupPublishers()
        self.setupSubscribers()
        self.setupRosQueueCallbacks()
        self.running = True

    def setupRosQueueCallbacks(self):
        self.rqC = []
        self.rqC.append((type(RunStop()), partial(self.runStopPub.publish)))

    def setupPublishers(self):
        self.runStopPub = rospy.Publisher('runstop', RunStop, queue_size=1)

    def setupSubscribers(self):
        self.runStopSub = rospy.Subscriber("/runstop", RunStop, self.queueRosMsg)
        self.status = rospy.Subscriber("/status", Status, self.queueRosMsg)
        self.encoder = rospy.Subscriber("/encoder", Encoder, self.queueRosMsg)
        self.odom = rospy.Subscriber("/odom", Odometry, self.queueRosMsg)
        self.visOdom = rospy.Subscriber("/tracking_camera/odom/sample", Odometry, self.queueRosMsg)
        self.kfOdom = rospy.Subscriber("/robot_pose_ekf/odom_combined", Odometry, self.queueRosMsg)

    def queueRosMsg(self, msg):
        self.ocsQueue.put(["Msg", msg])                

    def process(self):        
        while (not self.rosQueue.empty()):            
            rosItem = self.rosQueue.get()                 

            for i in self.rqC:                
                if i[0] == type(rosItem):                    
                    i[1](rosItem)       
                    break     

    def runMainLoop(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and self.running:                        
            r.sleep()
            self.process()          