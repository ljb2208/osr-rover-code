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
from functools import partial

FONT_LABEL = "Arial 11 bold"

class LaunchCommand():
    def __init__(self, launchFile, cb):
        self.launchFile = launchFile
        self.cb = cb
        self.launchThread = None

    def launch(self):
        self.launchThread = threading.Thread(target=self.runLaunch)
        self.launchThread.start()

    def stop(self):
        if not self.launchThread.is_alive():
            return
        

    def runCallback(self, retCode, exc):        
        rospy.loginfo("cb: " + str(retCode))

        if exc is not None:
            rospy.loginfo("exception: " + str(exc))

        self.launchThread = None
        self.cb(retCode, exc)

    def runLaunch(self):
        retCode = -1

        try:
            retCode = subprocess.call(["roslaunch", "osr_bringup", self.launchFile], bufsize=4096)
            self.runCallback(retCode, None)
        except Exception as exc:
            self.runCallback(retCode, exc)

class LaunchItem():
    def __init__(self, btn, launchFile, launchObj):
        self.btn = btn
        self.launchFile = launchFile
        self.launchObj = launchObj
        self.retCode = 0
        self.exc = None
        self.active = False

    def processLaunchRequest(self):
        if self.launchObj is None:
            self.btn["background"] = "green"
            self.btn["activebackground"] = "green"
            self.launchObj = LaunchCommand(self.launchFile, self.launchCallback)
            self.active = True
            self.launchObj.launch()
        else:
            self.btn["background"] = "grey"
            self.btn["activebackground"] = "grey"
            self.launchObj = None

    def updateStatus(self):
        color = "grey"
        if self.active and self.launchObj is None:
            if self.exc is not None:
                color = "red"
            elif self.retCode != 0:
                color = "red"
        
            self.btn["background"] = color
            self.btn["activebackground"] = color
            self.launchObj = None
            self.active = False



    def launchCallback(self, retCode, exc):
        self.retCode = retCode
        self.exc = exc
        self.launchObj = None

class LaunchOption():
    def __init__(self, desc, launchFile, groupId):
        self.desc = desc
        self.launchFile = launchFile
        self.groupId = groupId