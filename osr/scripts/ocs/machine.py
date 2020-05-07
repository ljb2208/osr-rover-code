#!/usr/bin/env python

from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from tkinter import Toplevel
import threading
import gc
import os
import math
import subprocess
import time
from datetime import datetime
import rospy
import locale
import re
import string
from ocs.frames import FONT_LABEL

class Machine():
    def __init__(self, machineName, userName, powerMgmt):
        self.sshName = userName + "@" + machineName
        self.machineName = machineName
        self.userName = userName
        self.retVal = 0
        self.exc = None
        self.alive = None
        self.hasStats = None
        self.powerMgmt = powerMgmt
        self.lastUpdateTime = None

    def runThread(self, command, cb):
        retval = -1
        retExc = None
        try:
            retval = subprocess.run(command, bufsize=4096, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)                        
        except Exception as exc:
            retval = None
            retExc = exc

        cb(retval, retExc)
        
        self.launchThread = None

    def runCommand(self, command, cb):
        threadArgs = [command,cb]
        self.launchThread = threading.Thread(target=self.runThread, args=threadArgs)
        self.launchThread.start()

    def onShutdownCallback(self, retVal, exc):
        rospy.loginfo("Shutdown. " + str(retVal) + ": " + str(exc))
    

    def shutDown(self):                
        if (messagebox.askokcancel("Shutdown", "Confirm shutdown of " + self.machineName)):
            command = ["ssh", self.sshName, "sudo", "shutdown", "-h", "now"]
            self.runCommand(command, self.onShutdownCallback)        

    def buildControls(self, parentCtrl, startRow, startCol):
        self.machineBtn = Button(parentCtrl, text = self.machineName, width=9, font=FONT_LABEL)                            
        self.machineBtn.grid(row=startRow, column=startCol, sticky=NW)                

        lbl3 = Label(parentCtrl, text="CPU Info", font="Arial 10")
        lbl3.grid(row=startRow+1, column=startCol, sticky=NW)
        self.labelCPU = Label(parentCtrl, text="")
        self.labelCPU.grid(row=startRow+1, column=startCol+1, columnspan=3, sticky=NW)
        lbl4 = Label(parentCtrl, text="Mem Info")
        lbl4.grid(row=startRow+2, column=startCol, sticky=NW)
        self.labelMem = Label(parentCtrl, text="")
        self.labelMem.grid(row=startRow+2, column=startCol+1, columnspan=3, sticky=NW)
        lbl5 = Label(parentCtrl, text="Wifi Info")
        lbl5.grid(row=startRow+3, column=startCol, sticky=NW)
        self.labelWifi = Label(parentCtrl, text="")
        self.labelWifi.grid(row=startRow+3, column=startCol+1, columnspan=3, sticky=NW)

        if self.powerMgmt:
            lbl6 = Label(parentCtrl, text="Power")
            lbl6.grid(row=startRow+4, column=startCol, sticky=NW)
            self.labelPower = Label(parentCtrl, text="")
            self.labelPower.grid(row= startRow+4, column=startCol+1, columnspan=3, sticky=NW)
            return startRow + 5

        return startRow + 4

    def onCompStatsMsg(self, msg):        
        self.labelCPU["text"] = msg.cpu1 + ":" + msg.cpu2 + ":" + msg.cpu3
        self.labelMem["text"] = msg.mem_pct + "%"
        self.labelWifi["text"] = msg.link_quality + " " + msg.bit_rate
        
        if self.powerMgmt:
            self.labelPower["text"] = msg.power_level

        self.lastUpdateTime = time.time()

    def updateStatus(self):
        color = "red"

        if self.lastUpdateTime is not None and (time.time() - self.lastUpdateTime) < 10:
            color = "green"

        self.machineBtn["background"] = color
        self.machineBtn["activebackground"] = color


class MachineFrame(Frame):
    def __init__(self, parentCtrl, ocs):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.machineList = [Machine("xavier-osr", "lbarnett", True), Machine("rover-osr", "lbarnett", False)]
        self.machineLabel = []
        self.machineButton = []
        self.machineStatus = []        
        self.timerCount = 0

        self.buildControls()
        self.ocs.setTimerCallback(self.onTimer)

    def buildControls(self):                         
        i = 0
        for machine in self.machineList:
            i = machine.buildControls(self, i, 0)
            
        self.grid_columnconfigure(0, weight=1)


    def onTimer(self):        
        self.timerCount += 1        
        
        if self.timerCount == 5:
            for machine in self.machineList:
                machine.updateStatus()
            self.timerCount = 0

    def onCompStatsMsg(self, msg):
        host = msg.host_name

        for machine in self.machineList:
            if machine.machineName == host:
                machine.onCompStatsMsg(msg)
                return
    
