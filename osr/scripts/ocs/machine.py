#!/usr/bin/env python

from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import threading
import gc
import os
import math
import subprocess
import time
import rospy
import locale
import re
import string
from ocs.frames import FONT_LABEL

class Machine():
    def __init__(self, machineName, userName):
        self.sshName = userName + "@" + machineName
        self.machineName = machineName
        self.userName = userName
        self.retVal = 0
        self.exc = None
        self.alive = None
        self.hasStats = None
        self.machineStats = MachineStats()

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
    

    def onStatsCallback(self, retVal, exc):
        if retVal is not None and exc is None:
            self.machineStats.parseStats(str(retVal.stdout))                    
            self.hasStats = True
            self.alive = True
        else:
            self.alive = False

        # print("stats: " + str(self.machineStats))

    def getMachineStats(self):
        command = ["ssh", "-t", self.sshName, "top -n 1"]
        self.runCommand(command, self.onStatsCallback)

    def isAlive(self):        
        command = ["ssh", self.sshName, "hostname"]
        self.runCommand(command, self.onAliveCallback)        

    def shutDown(self):                
        if (messagebox.askokcancel("Shutdown", "Confirm shutdown of " + self.machineName)):
            command = ["ssh", self.sshName, "sudo", "shutdown", "-h", "now"]
            self.runCommand(command, self.onShutdownCallback)        

class MachineStats():
    def __init__(self):
        self.memTotal = ""
        self.memFree = ""        
        self.memPct = 0.
        self.statTime = ""
        self.cpu = ["", "", ""]

    def __repr__(self):
        return self.memTotal + " : " + self.memFree + " : " + str(self.cpu) + " : " + self.statTime

    def parseStats(self, stats):
        lines = stats.split('\n')
        self.parseCPUStats(lines)
        self.parseMemStats(lines)
        self.parseTime(lines)


    def escape_ansi(self, info):        
        r = ""
        dataStarted = False

        for c in info:            
            if ord(c) == 32:
                dataStarted = True
            elif ord(c) < 28:
                dataStarted = False
            
            if dataStarted:
                r += c            
        return r        

    def parseCPUStats(self, stats):
        for line in stats:
            if "load average:" in line:
                ind = line.find("load average:")                
                line = line[ind+13:]                
                cpuInfo = line.split(",")                

                sz = len(cpuInfo)

                if sz > len(self.cpu):
                    sz = len(self.cpu)                
                
                for i in range(sz):                    
                    cpuStr = self.escape_ansi(cpuInfo[i]).strip()                    
                    # if '\x1b' in cpuStr:
                    #     jind = cpuStr.find('\x1b')
                    #     cpuStr = cpuStr[:jind]                    
                    self.cpu[i] = str(cpuStr)

                return

    def parseMemStats(self, stats):        
        for line in stats:
            if "KiB Mem" in line:
                sInd = line.find("KiB Mem :")
                eInd = line.find("total")
                
                totMem = line[sInd+9:eInd]                
                self.memTotal = self.escape_ansi(totMem).strip()
                                
                sInd = line.find("total,")
                eInd = line.find("free")

                freeMem = line[sInd+6:eInd]                
                self.memFree = self.escape_ansi(freeMem).strip()

                try:
                    totMem = float(self.memTotal)
                    totFree = float(self.memFree)
                    self.memPct = int((totMem - totFree) / totMem * 100)
                except:
                    self.memPct = -1

                return
    
    def parseTime(self, stats):
        for line in stats:
            if "top -" in line:
                sInd = line.find("top -")
                eInd = line.find("up")
                line = line[sInd + 6:eInd]
                self.statTime = line.strip()
                return
                
                

class MachineFrame(Frame):
    def __init__(self, parentCtrl, ocs):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.machineList = [Machine("xavier-osr", "lbarnett"), Machine("rover-osr", "lbarnett")]
        self.machineLabel = []
        self.machineButton = []
        self.machineStatus = []        
        self.timerCount = 0

        self.buildControls()
        self.ocs.setTimerCallback(self.onTimer)

    def buildControls(self):                 
        lbl = Label(self, text="Machines", font=FONT_LABEL)
        lbl.grid(row=0, column=0, columnspan=2, sticky="NW")

        i = 1
        for machine in self.machineList:
            lbl = Label(self, text = machine.machineName, width=8)            
            btnCheck = Button(self, text="C", command=machine.isAlive)
            btnStats = Button(self, text="S", command=machine.getMachineStats)
            btn = Button(self, text="X", command=machine.shutDown)
            lbl.grid(row=i, column=0, sticky=NW)
            btnCheck.grid(row=i, column=1, sticky=NE)
            btnStats.grid(row=i, column=2, sticky=NE)
            btn.grid(row=i, column=3, sticky=NE)
            self.machineLabel.append(lbl)
            self.machineButton.append(btn)
            self.machineStatus.append(None)
            i += 1

        lbl1 = Label(self, text="Stats", font=FONT_LABEL)
        lbl1.grid(row=i, column=0, sticky=NW)
        lbl2 = Label(self, text="Machine")
        lbl2.grid(row=i+1, column=0, sticky=NW)
        self.labelMachine = Label(self, text="")
        self.labelMachine.grid(row=i+1, column=1, columnspan=3, sticky=NW)
        lbl3 = Label(self, text="CPU Info")
        lbl3.grid(row=i+2, column=0, sticky=NW)
        self.labelCPU = Label(self, text="")
        self.labelCPU.grid(row=i+2, column=1, columnspan=3, sticky=NW)
        lbl4 = Label(self, text="Mem Info")
        lbl4.grid(row=i+3, column=0, sticky=NW)
        self.labelMem = Label(self, text="")
        self.labelMem.grid(row=i+3, column=1, columnspan=3, sticky=NW)
        lbl4 = Label(self, text="Stats at")
        lbl4.grid(row=i+4, column=0, sticky=NW)
        self.labelStatsTime = Label(self, text="")
        self.labelStatsTime.grid(row=i+4, column=1, columnspan=3, sticky=NW)

        self.grid_columnconfigure(0, weight=1)


    def onTimer(self):
        self.timerCount += 1

        for i in range(len(self.machineList)):
            if self.machineStatus[i] != self.machineList[i].alive:
                color = "red"
                if self.machineList[i].alive == True:
                    color = "green"
                
                self.machineLabel[i]["background"] = color
                self.machineLabel[i]["activebackground"] = color
                self.machineStatus[i] = self.machineList[i].alive
            
            if self.machineList[i].hasStats:
                self.labelMachine["text"] = self.machineList[i].machineName
                
                self.labelCPU["text"] = str(self.machineList[i].machineStats.cpu[0]) + " : " + str(self.machineList[i].machineStats.cpu[1]) + " : " + str(self.machineList[i].machineStats.cpu[2])
                self.labelMem["text"] = str(self.machineList[i].machineStats.memPct) + "%"
                self.labelStatsTime["text"] = str(self.machineList[i].machineStats.statTime)
                self.machineList[i].hasStats = False
            
        
        if self.timerCount == 5:
            self.timerCount = 0


