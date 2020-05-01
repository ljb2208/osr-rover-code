#!/usr/bin/env python

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
from geometry_msgs.msg import PoseWithCovarianceStamped
from functools import partial
from ocs.launch import *
from ocs.rosctrl import *
from ocs.frames import *
from ocs.odomframe import *

FONT_LABEL = "Arial 11 bold"


class OCS():

    def __init__(self, inQueue, outQueue):
        self.main = Tk()
        self.running = True
        self.inQueue = inQueue
        self.outQueue = outQueue
        self.timerCallbacks = []
        self.timer = None
        
        self.buildMainWindow()
        
        self.setupInboundMessageCBs()

    def setupInboundMessageCBs(self):       
        self.ibC = []
        self.ibC.append((type(RunStop()), partial(self.ctrlFrame.onRunStopMsg)))
        self.ibC.append((type(Status()), partial(self.statusFrame.onStatusMsg)))
        self.ibC.append((type(Encoder()), partial(self.roverFrame.onEncoderMsg)))
        self.ibC.append((type(Odometry()), partial(self.odomFrame.onOCSOdomMsg)))        
        self.ibC.append((type(PoseWithCovarianceStamped()), partial(self.odomFrame.onOCSGeometryMsg)))

    def buildMainWindow(self):
        rosMasterURI = os.getenv("ROS_MASTER_URI")

        if len(rosMasterURI) < 1:
            rosMasterURI = "(No ROS Master Set)"

        self.main.title("OSR OCS - " + rosMasterURI)    
        try:            
            pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), "osr.gif")
            self.main.tk.call('wm', 'iconphoto', self.main._w, PhotoImage(file=pth))            
        except TclError as err:
            print("Error loading icon")
            print(str(err))

        self.main.protocol("WM_DELETE_WINDOW", self.close)   

        self.mainFrame = Frame(self.main)        
        self.mainFrame.grid(row=0, column=0, sticky=NSEW)                             

        self.roverFrame = RoverFrame(self, self.mainFrame)
        self.roverFrame.grid(row=0, column=0, sticky=NE)

        self.odomFrame = OdometryFrame(self, self.mainFrame)
        self.odomFrame.grid(row=0, column=1, sticky=NSEW)
        
        self.statusFrame = StatusFrame(self, self.mainFrame)
        self.statusFrame.grid(row=1, column=0, columnspan=2, sticky=S+E+W)                

        self.ctrlFrame = ControlFrame(self, self.mainFrame)                
        self.ctrlFrame.grid(row=0, column=2, sticky=N+E+S, rowspan=2)
        
        self.mainFrame.grid_columnconfigure(1, weight=1)   
        self.mainFrame.grid_rowconfigure(0, weight=1)   

        self.main.grid_columnconfigure(0, weight=1)
        self.main.grid_rowconfigure(0, weight=1)   

    def close(self):        
        self.running = False

    def processMessages(self):

        if not self.running:
            return

        while not self.inQueue.empty():
            msg = self.inQueue.get()

            if msg[0] == "Quit":
                self.running = False
            elif msg[0] == "Msg":
                for i in self.ibC:                
                    if i[0] == type(msg[1]):                    
                        i[1](msg[1])               
                        break 

    def setTimerCallback(self, cb):
        self.timerCallbacks.append(cb)
                         
    def runTimers(self):
        for cb in self.timerCallbacks:
            cb()

    def updateTimer(self):
        if self.timer is None:
            self.timer = time.time()
            return
        
        timeNow = time.time()

        if (timeNow - self.timer) > 1.:
            self.timer = timeNow
            self.runTimers()


    def runMainLoop(self):        
        if self.running:
            self.main.update_idletasks()            
            self.main.update()                  
            self.processMessages()      
            self.updateTimer()
            return True
        else:
            return False        

    def exitApp(self):
        self.main.destroy()


 


global rloop

def runROSThread(ocsQueue, rosQueue):
    global rloop
    rloop = rosLoop(ocsQueue, rosQueue)
    rloop.runMainLoop()
    
if __name__ == '__main__':	    
    global rloop
    rospy.init_node('osr_ocs')
    rospy.loginfo('osr ocs started')    
    
    ocsQueue = queue.Queue()
    rosQueue = queue.Queue()

    # start ros threa
    rosThread = threading.Thread(target=runROSThread, args=(ocsQueue, rosQueue))
    rosThread.start()

    ocs = OCS(ocsQueue, rosQueue)

    running = True
    
    while running:
        running = ocs.runMainLoop()

        if running:
            running = rosThread.is_alive()
    
    rloop.running = False    
    ocs.running = False
    ocs.exitApp()    

    rospy.loginfo('osr ocs exiting....')    
