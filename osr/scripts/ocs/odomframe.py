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
from functools import partial
from ocs.launch import *
from ocs.rosctrl import *
import random

FONT_LABEL = "Arial 11 bold"

class OdometryPoint():
    def __init__(self, point, ctrl, lineCtrl = None, visOdom=False):
        self.point = point
        self.ctrl = ctrl
        self.lineCtrl = lineCtrl
        self.x = 0
        self.y = 0
        self.visOdom = visOdom

class OdometryFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.odomPoints = []
        self.visOdomPoints = []
        self.pointRadius = 2        
        self.zoom=1
        self.xCenter = 100
        self.yCenter = 100
        self.priorOdomPoint = None
        self.priorVisOdomPoint = None
        self.connectLines = True
        self.connectLinesVar = IntVar()
        self.connectLinesVar.set(1)
        self.odom = True
        self.odomVar = IntVar()
        self.odomVar.set(self.odom)
        self.visOdom = False
        self.visOdomVar = IntVar()
        self.visOdomVar.set(self.visOdom)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)        
        self.buildControls()                

    def onCanvasResizeEvent(self, event):
        self.xCenter = event.width / 2
        self.yCenter = event.height / 2        
        self.redraw()


    def buildControls(self):        
        self.odomCanvas = Canvas(self)        
        self.odomCanvas.grid(row=0, column=0, columnspan=4, sticky=NSEW)    
        self.odomCanvas.bind('<Configure>', self.onCanvasResizeEvent)
        self.controlsFrame = Frame(self, borderwidth=2, relief="groove")
        self.controlsFrame.grid(row=0, column=0, sticky=N+E+S)


        zinBtn = Button(self.controlsFrame, text="+", width="2", command=self.zoomIn)
        zoutBtn = Button(self.controlsFrame, text="-", width="2", command=self.zoomOut)
        clCheck = Checkbutton(self.controlsFrame, text="Lines", variable=self.connectLinesVar, command=self.toggleConnectLines)
        clOdom = Checkbutton(self.controlsFrame, text="Odom", variable=self.odomVar, command=self.toggleOdom)
        clVisOdom = Checkbutton(self.controlsFrame, text="Vis. Odom", variable=self.visOdomVar, command=self.toggleVisOdom)

        btn = Button(self.controlsFrame, text="Test", command=self.testButton)
        zinBtn.grid(row=0, column=0, sticky="NE")
        zoutBtn.grid(row=1, column=0, sticky="NE")
        clCheck.grid(row=2, column=0, sticky="NW")
        clOdom.grid(row=3, column=0, sticky="NW")
        clVisOdom.grid(row=4, column=0, sticky="NW")
        
        btn.grid(row=5, column=0)

    def toggleConnectLines(self):
        if self.connectLinesVar.get() == 1:
            self.connectLines = True
        else:
            self.connectLines = False

        self.redraw()

    def toggleOdom(self):
        if self.odomVar.get() == 1:
            self.odom = True
        else:
            self.odom = False

        self.redraw()

    def toggleVisOdom(self):
        if self.visOdomVar.get() == 1:
            self.visOdom = True
        else:
            self.visOdom = False

        self.redraw()
    
    def testButton(self):
        random.seed()

        # generate test points
        for i in range(10):
            x = i * 10
            y = 0
            pt = OdometryPoint((x, y), None)

            if self.isNewOdomPoint(pt):
                self.drawPoint(pt)
                self.priorOdomPoint = pt
            

    def zoomIn(self):
        self.zoom *= 2
        self.redraw()

    def zoomOut(self):
        self.zoom /= 2
        self.redraw()

    def redraw(self):
        self.priorOdomPoint = None

        for pt in self.odomPoints:
            self.deletePoint(pt)

            if self.odom:
                self.drawPoint(pt)        
                self.priorOdomPoint = pt

        for pt in self.visOdomPoints:
            self.deletePoint(pt)

            if self.visOdom:
                self.drawPoint(pt)        
                self.priorVisOdomPoint = pt

    def getPointOnCanvas(self, x, y):
        x = (x * self.zoom) + self.xCenter
        y = (y * self.zoom) + self.yCenter
        
        return x, y, x - self.pointRadius, y - self.pointRadius, x + self.pointRadius, y + self.pointRadius

    def isNewOdomPoint(self, point):
        if self.priorOdomPoint is not None and point.point[0] == self.priorOdomPoint.point[0] and point.point[1] == self.priorOdomPoint.point[1]:
            return False

        self.odomPoints.append(point)
        return True

    def isNewVisOdomPoint(self, point):
        if self.priorVisOdomPoint is not None and point.point[0] == self.priorVisOdomPoint.point[0] and point.point[1] == self.priorVisOdomPoint.point[1]:
            return False

        self.visOdomPoints.append(point)
        return True

    def deletePoint(self, point):
        if point.ctrl is not None:            
            self.odomCanvas.delete(point.ctrl)
            point.ctrl = None

        if point.lineCtrl is not None:
            self.odomCanvas.delete(point.lineCtrl)
            point.lineCtrl = None

    def drawPoint(self, point):
        point.x, point.y, x1, y1, x2, y2  = self.getPointOnCanvas(point.point[0], point.point[1])        

        if self.connectLines and self.priorOdomPoint is not None:
            point.lineCtrl = self.odomCanvas.create_line(point.x, point.y, self.priorOdomPoint.x, self.priorOdomPoint.y)

        color = "blue"

        if point.visOdom:
            color = "red"

        point.ctrl = self.odomCanvas.create_oval(x1, y1, x2, y2, fill=color)                    
    
    def removePoint(self, point):
        if point.ctrl is None:
            return

    def onOCSOdomMsg(self, msg):
        topic = msg._connection_header['topic']

        if topic == "/odom":
            point = OdometryPoint(msg.PoseWithCovariance.Pose.Point, None)
            if self.isNewOdomPoint(point) and self.odom:
                self.drawPoint(point)
                self.priorOdomPoint = point
        else:
            point = OdometryPoint(msg.PoseWithCovariance.Pose.Point, None, visOdom=True)
            if self.isNewVisOdomPoint(point) and self.visOdom:
                self.drawPoint(point)
                self.priorVisOdomPoint = point
        



        
        
    

    