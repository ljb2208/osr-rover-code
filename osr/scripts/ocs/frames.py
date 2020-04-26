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
from ocs.launch import *
from ocs.rosctrl import *

FONT_LABEL = "Arial 11 bold"

class ImageFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs

class LaunchFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.launchOptions = []
        self.launchOptions.append(LaunchOption("Launch OSR", "osr.launch", 0))
        self.launchOptions.append(LaunchOption("Launch Full", "xavier_full.launch", 0))        
        self.launchOptions.append(LaunchOption("Launch XIMEA", "ximea.launch", 1))
        self.launchOptions.append(LaunchOption("Mount USB", "mount_usb.launch", 2))
        self.launchOptions.append(LaunchOption("Record", "record.launch", 3))
        self.launchOptions.append(LaunchOption("Unmount USB", "unmount_usb.launch", 4))
        self.launchItems = {}
        self.buildControls()
        self.ocs.setTimerCallback(self.onTimer)

    def buildControls(self):
        launchLabel = Label(self, text="Launch Options", font=FONT_LABEL, justify="left")
        launchLabel.grid(row=0, column=0)

        currRow = 1
        for launchOption in self.launchOptions:            
            btn = Button(self, text=launchOption.desc, command=partial(self.launch, launchOption.launchFile, launchOption.groupId), width=12)
            self.launchItems[launchOption.launchFile] = LaunchItem(btn, launchOption.launchFile, None)
            btn.grid(row=currRow, column=0)
            currRow += 1

    # check to see if any other launch files from same group are running
    def checkLaunchGroup(self, launchFile, groupId):
        for l in self.launchOptions:            
            if l.groupId == groupId and not l.launchFile == launchFile:
                if self.launchItems[l.launchFile].launchObj is not None:
                    return False
        
        return True

    def launch(self, launchFile, groupId):        
        if not self.checkLaunchGroup(launchFile, groupId):
            return

        rospy.loginfo("Process launch request " + launchFile)
        self.launchItems[launchFile].processLaunchRequest()

    def onTimer(self):        
        for lItem in self.launchItems.values():            
            lItem.updateStatus()        


class RoverFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.enc_angles = [0, 0, 0, 0]
        self.enc_min = [155, 90, 175, 424]
        self.enc_max = [1321, 1244, 1366, 1565]
        self.enc_mid = [0,0,0,0]
        self.enc_len = [0,0,0,0]
        self.getEncValues()	
        self.wheels = []        
        self.wheelWidth = 15
        self.wheelHeight = 30
        self.wheelCoords = [(20, 20), (20, 145), (115, 20), (115,145)]

        self.buildControls()
        self.drawRover()

    def getEncValues(self):
        for i in range(len(self.enc_min)):
            self.enc_len[i] = self.enc_max[i] - self.enc_min[i]
            self.enc_mid[i] = self.enc_len[i] /2

    def buildControls(self):
        self.canv = Canvas(self, background="white", width=200, height=250)
        self.canv.grid(row=0, column=0, sticky=NW)

    def rotatePoint(self, cx, cy, px, py, sinAngle, cosAngle):
        nx = cx + (px - cx) * cosAngle + (cy - py) * sinAngle
        ny = cy + (py - cy) * cosAngle + (cx - px) * sinAngle        

        return nx, ny

    def rotateRectangle(self, rect, coords,  angle):
        angle = angle * (math.pi / 180)

        # centre coords of rectangle
        x = coords[0] + self.wheelWidth/2
        y = coords[1] + self.wheelHeight/2

        sinAngle = math.sin(angle)
        cosAngle = math.cos(angle)

        x1, y1 = self.rotatePoint(x, y, coords[0], coords[1], sinAngle, cosAngle)        
        x2, y2 = self.rotatePoint(x, y, coords[0] + self.wheelWidth, coords[1], sinAngle, cosAngle)        
        x3, y3 = self.rotatePoint(x, y, coords[0] + self.wheelWidth, coords[1] + self.wheelHeight, sinAngle, cosAngle)        
        x4, y4 = self.rotatePoint(x, y, coords[0], coords[1] + self.wheelHeight, sinAngle, cosAngle)        

        self.canv.coords(rect, x1, y1, x2, y2, x3, y3, x4, y4)


    def updateRover(self):
        for i in range(len(self.wheelCoords)):
            self.rotateRectangle(self.wheels[i], self.wheelCoords[i], self.enc_angles[i])

    def drawRover(self):
        self.canv.create_rectangle(50, 65, 100, 130, fill="black") # body        
        self.canv.create_rectangle(5, 80, 20, 110, fill="black") # left middle wheel        
        self.canv.create_rectangle(130, 80, 145, 110, fill="black") # right middle wheel        
        self.canv.create_rectangle(20, 93, 50, 97, fill="grey") # left middle axle
        self.canv.create_rectangle(100, 93, 130, 97, fill="grey") # right middle axle
        self.canv.create_rectangle(24, 50, 32, 144, fill="grey") # left arm
        self.canv.create_rectangle(119, 50, 127, 144, fill="grey") # right arm
        self.canv.create_rectangle(24, 115, 127, 120, fill="grey") # torsion bar
        self.canv.create_oval(70, 75, 80, 85, fill="grey") # cam mount
        
        
        # create corner wheels
        for w in self.wheelCoords:
            x = w[0]
            y = w[1]
            self.wheels.append(self.canv.create_polygon(x, y, x+self.wheelWidth, y, x + self.wheelWidth, y+self.wheelHeight, x, y+self.wheelHeight, fill="black")) # left front wheel

    def onEncoderMsg(self, msg):
        for i in range(len(self.enc_mid)):
            self.enc_angles[i] = (msg.abs_enc[i] - self.enc_mid[i]) / self.enc_mid[i] * 45

        self.updateRover()



class StatusFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.batteryLevels = (15.2, 14.8)
        self.rcTempLevels = (35, 40)
        self.rcAmpLevels = (5, 10)

        self.rcLabel = []
        self.rcAmp = []
        self.rcTemp = []
        
        self.buildControls()            
    
    def buildControls(self):        
        batt = Label(self, text="Battery", font=FONT_LABEL)
        self.batteryValue = Label(self, text="0.0", borderwidth=2, relief="groove", bg="orange", width=4)        
        batt.grid(row=0, column=0)
        self.batteryValue.grid(row=0, column=1)    

        roboLabel = Label(self, text="Roboclaw", anchor=W, font=FONT_LABEL)
        roboLabel.grid(row=1, column=0)

        amps = Label(self, text="Amps", anchor=W, font=FONT_LABEL)
        amps.grid(row=2, column=0)

        temp = Label(self, text="Temp", anchor=W, font=FONT_LABEL)
        temp.grid(row=3, column=0)
        
        col = 1
        for i in range(5):
            self.rcLabel.append(Label(self, text="rc" + str(i+1), width=3, font=FONT_LABEL))
            self.rcAmp.append(Label(self, text="0.0", width=4, bg="orange", borderwidth=2, relief="groove"))
            self.rcAmp.append(Label(self, text="0.0", width=4, bg="orange", borderwidth=2, relief="groove"))
            self.rcTemp.append(Label(self, text="0.0", width=4, bg="orange", borderwidth=2, relief="groove"))

            self.rcLabel[i].grid(row=1, column=col, columnspan=2)
            self.rcAmp[i*2].grid(row=2, column=col)        
            self.rcAmp[i*2 + 1].grid(row=2, column=(col+1))   
            self.rcTemp[i].grid(row=3, column=col, columnspan=2)

            col += 2       

    def updateValue(self, ctrl, value, valueLevels):
        ctrl["text"]= str(value)

        color = "green"
        valueAmber = valueLevels[0]
        valueRed = valueLevels[1]

        if valueAmber > valueRed:
            if value < valueRed:
                color = "red"
            elif value < valueAmber:
                color = "orange"
        else:
            if value > valueRed:
                color = "red"
            elif value > valueAmber:
                color = "orange"

        ctrl["bg"] = color
                

    def updateValues(self, msg):
        self.updateValue(self.batteryValue, float(msg.battery)/10, self.batteryLevels)

        for i in range(10):
            self.updateValue(self.rcAmp[i], float(msg.current[i])/10, self.rcAmpLevels)        

        for i in range(5):
            self.updateValue(self.rcTemp[i], float(msg.temp[i])/10, self.rcTempLevels)
        

    def onStatusMsg(self, msg):        
        self.updateValues(msg)

class NodeList(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl)
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.nodes = list()        
        self.requiredNodes = ("/joystick", "/motorcontroller", "/osr_ocs")            

        self.buildControls()        
    
    def buildControls(self):        
        self.nodeListCtrl = Listbox(self, listvariable=self.nodes)
        self.nodeListCtrl.grid(row=0, column=0, sticky=W)
        self.getRunningNodes()
        

    def getRunningNodes(self):
        nodes = rosnode.get_node_names()        

        self.nodeListCtrl.delete(0, END)

        for n in nodes:
            if not "/ros" in n:
                index = self.nodeListCtrl.size()
                self.nodeListCtrl.insert(index, n)    

                if n in self.requiredNodes:
                    self.nodeListCtrl.itemconfigure(index, background="green")
                else:
                    self.nodeListCtrl.itemconfigure(index, background="")

        # check required nodes and add if not running
        for rn in self.requiredNodes:   
            if rn not in nodes:
                index =  self.nodeListCtrl.size()
                self.nodeListCtrl.insert(index, rn)                            
                self.nodeListCtrl.itemconfigure(index, background="red")            


class ControlFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.stop = False
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.runStop = False
        self.auto = False

        self.buildControls()

    def buildControls(self):
        self.runStopBtn = Button(self, text="Run/Stop", width=8, command=self.onRunStop)
        self.autoBtn = Button(self, text="Auto Mode", width=8, command=self.onAuto)
        nodeLabel = Label(self, text="Node List", justify="left", font=FONT_LABEL)        
        self.launchFrame = LaunchFrame(self.ocs, self)
        self.nodeList = NodeList(self.ocs, self)        

        self.runStopBtn.grid(row=0, column=0, sticky=NW)
        self.autoBtn.grid(row=0, column=1, sticky=NE)        
        self.launchFrame.grid(row=1, column=0, columnspan=2, sticky=N+W+E)
        nodeLabel.grid(row=2, column=0, sticky=W)
        self.nodeList.grid(row=3, column=0, columnspan=2, sticky=N+W+E)        

        # add quit button    
        button = Button(self, text="Quit", width=10, command=self.ocs.close)
        button.grid(row=4, column=1, sticky=SE)

        self.grid_rowconfigure(3, weight=1)


    def onRunStop(self):
        self.runStop = not self.runStop
        self.sendRunStopMsg()

    def onAuto(self):
        self.auto = not self.auto
        self.sendRunStopMsg()

    def sendRunStopMsg(self):
        rs = RunStop()
        rs.run = self.runStop
        rs.auto = self.auto

        self.ocs.outQueue.put(rs)

    def setColor(self, ctrl, value):
        if value:
            ctrl["bg"] = "green"
            ctrl["activebackground"] = "green"
        else:
            ctrl["bg"] = "red"
            ctrl["activebackground"] = "red"

    def onRunStopMsg(self, msg):
        self.runStop =  msg.run
        self.auto = msg.auto

        self.setColor(self.runStopBtn, self.runStop)
        self.setColor(self.autoBtn, self.auto)