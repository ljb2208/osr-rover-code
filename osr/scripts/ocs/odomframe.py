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
    def __init__(self, point, ctrl):
        self.point = point
        self.ctrl = ctrl

class OdometryFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.points = []
        self.pointRadius = 2        
        self.zoom=1
        self.xCenter = 100
        self.yCenter = 100
        self.priorPoint = None

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)        
        self.buildControls()                

    def onCanvasResizeEvent(self, event):
        self.xCenter = event.width / 2
        self.yCenter = event.height / 2        
        self.redraw()


    def buildControls(self):        
        self.odomCanvas = Canvas(self)        
        self.odomCanvas.grid(row=0, column=0, columnspan=3, sticky=NSEW)    
        self.odomCanvas.bind('<Configure>', self.onCanvasResizeEvent)
        zinBtn = Button(self, text="+", width="2", command=self.zoomIn)
        zoutBtn = Button(self, text="-", width="2", command=self.zoomOut)
        zinBtn.grid(row=1, column=0, sticky="NE")
        zoutBtn.grid(row=1, column=1, sticky="NE")
        btn = Button(self, text="Test", command=self.testButton)
        btn.grid(row=1, column=2)
    
    def testButton(self):
        random.seed()

        # generate test points
        for i in range(50):
            x = int((random.random() - 0.5) * 200)
            y = int((random.random() - 0.5) * 200) 
            pt = OdometryPoint((x, y), None)

            if self.isNewPoint(pt):
                self.drawPoint(pt)
                self.priorPoint = pt
            

    def zoomIn(self):
        self.zoom *= 2
        self.redraw()

    def zoomOut(self):
        self.zoom /= 2
        self.redraw()

    def redraw(self):
        for pt in self.points:
            self.drawPoint(pt)        

    def getPointOnCanvas(self, x, y):
        x = (x * self.zoom) + self.xCenter
        y = (y * self.zoom) + self.yCenter
        
        return x - self.pointRadius, y - self.pointRadius, x + self.pointRadius, y + self.pointRadius

    def isNewPoint(self, point):
        if self.priorPoint is not None and point.point[0] == self.priorPoint.point[0] and point.point[1] == self.priorPoint.point[1]:
            return False

        self.points.append(point)
        return True

    def drawPoint(self, point):
        if point.ctrl is not None:            
            self.odomCanvas.delete(point.ctrl)

        x1, y1, x2, y2  = self.getPointOnCanvas(point.point[0], point.point[1])        
        point.ctrl = self.odomCanvas.create_oval(x1, y1, x2, y2, fill="blue")                
        
    
    def removePoint(self, point):
        if point.ctrl is None:
            return

    def onOCSOdomMsg(self, msg):
        point = OdometryPoint(msg.PoseWithCovariance.Pose.Point, None)
        self.drawPoint(point)



        
        
    

    