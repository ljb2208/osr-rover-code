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
import random

FONT_LABEL = "Arial 11 bold"

class OdometryPoint():
    def __init__(self, point, ctrl, lineCtrl = None, pointType=0):
        self.point_x = point.x * 100 # scale to cms
        self.point_y = point.y * 100
        self.point_z = point.z * 100
        self.ctrl = ctrl
        self.lineCtrl = lineCtrl
        self.x = 0
        self.y = 0
        self.pointType = pointType

class OdometryFrame(Frame):
    def __init__(self, ocs, parentCtrl):
        super().__init__(parentCtrl, borderwidth=2, relief="groove")
        self.parentCtrl = parentCtrl
        self.ocs = ocs
        self.odomPoints = []
        self.visOdomPoints = []
        self.kfOdomPoints = []
        self.pointRadius = 2        
        self.zoom=1
        self.xCenter = 100
        self.yCenter = 100
        self.priorOdomPoint = None
        self.priorVisOdomPoint = None
        self.priorKfOdomPoint = None
        self.connectLines = True
        self.connectLinesVar = IntVar()
        self.connectLinesVar.set(1)
        self.odom = True
        self.odomVar = IntVar()
        self.odomVar.set(self.odom)
        self.visOdom = False
        self.visOdomVar = IntVar()
        self.visOdomVar.set(self.visOdom)
        self.kfOdom = False
        self.kfOdomVar = IntVar()
        self.kfOdomVar.set(self.kfOdom)

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
        clKfOdom = Checkbutton(self.controlsFrame, text="KF. Odom", variable=self.kfOdomVar, command=self.toggleKfOdom)

        btn = Button(self.controlsFrame, text="Test", command=self.testButton)
        zinBtn.grid(row=0, column=0, sticky="NE")
        zoutBtn.grid(row=1, column=0, sticky="NE")
        clCheck.grid(row=2, column=0, sticky="NW")
        clOdom.grid(row=3, column=0, sticky="NW")
        clVisOdom.grid(row=4, column=0, sticky="NW")
        clKfOdom.grid(row=5, column=0, sticky="NW")
        
        btn.grid(row=6, column=0)

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

    def toggleKfOdom(self):
        if self.kfOdomVar.get() == 1:
            self.kfOdom = True
        else:
            self.kfOdom = False

        self.redraw()
    
    def testButton(self):
        random.seed()

        # generate test points
        for i in range(10):
            x = i * 10
            y = 0
            pt = OdometryPoint((x, y), None)

            if self.isNewPoint(pt, self.priorOdomPoint, self.odomPoints):
                self.drawPoint(pt, self.priorOdomPoint)
                self.priorOdomPoint = pt
            

    def zoomIn(self):
        self.zoom *= 2
        self.redraw()

    def zoomOut(self):
        self.zoom /= 2
        self.redraw()

    def redraw(self):
        self.priorOdomPoint = None
        self.priorVisOdomPoint = None
        self.priorKfOdomPoint = None

        for pt in self.odomPoints:
            self.deletePoint(pt)

            if self.odom:
                self.drawPoint(pt, self.priorOdomPoint)        
                self.priorOdomPoint = pt

        for pt in self.visOdomPoints:
            self.deletePoint(pt)

            if self.visOdom:
                self.drawPoint(pt, self.priorVisOdomPoint)        
                self.priorVisOdomPoint = pt

        for pt in self.kfOdomPoints:
            self.deletePoint(pt)

            if self.kfOdom:
                self.drawPoint(pt, self.priorKfOdomPoint)        
                self.priorKfOdomPoint = pt

    def getPointOnCanvas(self, x, y):
        x = (x * self.zoom) + self.xCenter
        y = (y * self.zoom) + self.yCenter
        
        return x, y, x - self.pointRadius, y - self.pointRadius, x + self.pointRadius, y + self.pointRadius
   
    def isNewPoint(self, point, priorPoint, pointsList):
        if priorPoint is not None and point.point_x == priorPoint.point_x and point.point_y == priorPoint.point_y:
            return False

        pointsList.append(point)
        return True

    def deletePoint(self, point):
        if point.ctrl is not None:            
            self.odomCanvas.delete(point.ctrl)
            point.ctrl = None

        if point.lineCtrl is not None:
            self.odomCanvas.delete(point.lineCtrl)
            point.lineCtrl = None

    def drawPoint(self, point, priorPoint):
        point.x, point.y, x1, y1, x2, y2  = self.getPointOnCanvas(point.point_x, point.point_y)        

        if self.connectLines and priorPoint is not None:
            point.lineCtrl = self.odomCanvas.create_line(point.x, point.y, priorPoint.x, priorPoint.y)

        color = "blue"

        if point.pointType == 1:
            color = "red"
        elif point.pointType == 2:            
            color = "green"

        point.ctrl = self.odomCanvas.create_oval(x1, y1, x2, y2, fill=color)                    
    
    def removePoint(self, point):
        if point.ctrl is None:
            return

    def onOCSOdomMsg(self, msg):
        topic = msg._connection_header['topic']

        if topic == "/odom":
            point = OdometryPoint(msg.pose.pose.position, None)
            if self.isNewPoint(point, self.priorOdomPoint, self.odomPoints) and self.odom:
                self.drawPoint(point, self.priorOdomPoint)
                self.priorOdomPoint = point
        else:
            point = OdometryPoint(msg.pose.pose.position, None, pointType=1)
            if self.isNewPoint(point, self.priorVisOdomPoint, self.visOdomPoints) and self.visOdom:
                self.drawPoint(point, self.priorVisOdomPoint)
                self.priorVisOdomPoint = point
    
    def onOCSGeometryMsg(self, msg):
        point = OdometryPoint(msg.pose.pose.position, None, pointType=2)

        if self.isNewPoint(point, self.priorKfOdomPoint, self.kfOdomPoints) and self.kfOdom:
            self.drawPoint(point, self.priorKfOdomPoint)
            self.priorKfOdomPoint = point




        
        
    

    