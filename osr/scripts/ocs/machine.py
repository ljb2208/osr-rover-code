#!/usr/bin/env python

import threading
import gc
import os
import math
import subprocess
import time
import rospy

class Machine():
    def __init__(self, machineName, userName):
        self.sshName = userName + "@" + machineName
        self.machineName = machineName
        self.userName = userName
        self.retVal = 0
        self.exc = None
        self.alive = None

    def runThread(self, command, cb):
        retval = -1
        retExc = None
        try:
            retval = subprocess.run(command, bufsize=4096, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)                        
        except Exception as exc:
            retval = None
            retExc = exc

        cb(retval, retExc)
        
        self.launchThread = None

    def runCommand(self, command, cb):
        threadArgs = [command,cb]
        self.launchThread = threading.Thread(target=self.runThread, args=threadArgs)
        self.launchThread.start()

    def onAliveCallback(self, retVal, exc):                
        if retVal is not None and exc is None:
            if self.machineName in str(retVal.stdout):
                self.alive = True
            else:
                self.alive = False
        else:
            self.alive = False

    def onShutdownCallback(self, retVal, exc):
        rospy.loginfo("Shutdown. " + str(retVal) + ": " + str(exc))


    def isAlive(self):        
        command = ["ssh", self.sshName, "hostname"]
        self.runCommand(command, self.onAliveCallback)        

    def shutDown(self):        
        command = ["ssh", self.sshName, "sudo", "shutdown", "-h", "now"]
        self.runCommand(command, self.onShutdownCallback)        


    