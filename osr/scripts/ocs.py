#!/usr/bin/env python

from tkinter import *
from tkinter import ttk
import rospy
import rosnode
import queue
import threading
import gc
import os
import math
from osr_msgs.msg import RunStop, Status, Encoder
from functools import partial

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
        self.wheelCoords = [(10, 10), (10, 150), (125, 10), (125,150)]


        self.buildControls()
        self.drawRover()

    def getEncValues(self):
        for i in range(len(self.enc_min)):
            self.enc_len[i] = self.enc_max[i] - self.enc_min[i]
            self.enc_mid[i] = self.enc_len[i] /2

    def buildControls(self):
        self.canv = Canvas(self.parentCtrl)
        self.canv.grid(row=0, column=0)

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
        batt = Label(self, text="Battery")
        self.batteryValue = Label(self, text="0.0", borderwidth=2, relief="groove", bg="orange", width=4)        
        batt.grid(row=0, column=0)
        self.batteryValue.grid(row=0, column=1)    

        roboLabel = Label(self, text="Roboclaw", anchor=W, )
        roboLabel.grid(row=1, column=0)

        amps = Label(self, text="Amps", anchor=W, )
        amps.grid(row=2, column=0)

        temp = Label(self, text="Temp", anchor=W)
        temp.grid(row=3, column=0)
        
        col = 1
        for i in range(5):
            self.rcLabel.append(Label(self, text="rc" + str(i+1), width=3))
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
        self.runStopBtn = Button(self, text="Run/Stop", width=10, command=self.onRunStop)
        self.autoBtn = Button(self, text="Auto Mode", width=10, command=self.onAuto)
        self.nodeList = NodeList(self.ocs, self)        

        self.runStopBtn.grid(row=0, column=0, sticky=NW)
        self.autoBtn.grid(row=0, column=1, sticky=NE)
        self.nodeList.grid(row=1, column=0, columnspan=2, sticky=W+E)

        # add quit button    
        button = Button(self, text="Quit", width=10, command=self.ocs.close)
        button.grid(row=2, column=1, sticky=SE)


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


class OCS():

    def __init__(self, inQueue, outQueue):
        self.main = Tk()
        self.running = True
        self.inQueue = inQueue
        self.outQueue = outQueue
        
        self.buildMainWindow()
        
        self.setupInboundMessageCBs()

    def setupInboundMessageCBs(self):       
        self.ibC = []
        self.ibC.append((type(RunStop()), partial(self.ctrlFrame.onRunStopMsg)))
        self.ibC.append((type(Status()), partial(self.statusFrame.onStatusMsg)))
        self.ibC.append((type(Status()), partial(self.roverFrame.onEncoderMsg)))
    
    def buildCtrlFrame(self):
        self.ctrlFrame = ControlFrame(self, self.main)                
        self.ctrlFrame.grid(row=0, column=1, sticky=NE)
        

    def buildMainWindow(self):
        rosMasterURI = os.getenv("ROS_MASTER_URI")

        if len(rosMasterURI) < 1:
            rosMasterURI = "(No ROS Master Set)"

        self.main.title("OSR OCS - " + rosMasterURI)    
        try:            
            self.main.tk.call('wm', 'iconphoto', self.main._w, PhotoImage(file='/home/lbarnett/catkin_ws/src/osr-rover-code/osr/scripts/osr.gif'))            
        except TclError as err:
            print("Error loading icon")
            print(str(err))

        self.main.protocol("WM_DELETE_WINDOW", self.close)   

        self.mainFrame = Frame(self.main)        
        self.mainFrame.grid(row=0, column=0, sticky=NSEW)                
        
        self.main.grid_columnconfigure(0, weight=1)
        self.main.grid_rowconfigure(0, weight=1)        

        self.roverFrame = RoverFrame(self, self.mainFrame)
        self.roverFrame.grid(row=0, column=0, sticky=N+E+W)
        
        self.statusFrame = StatusFrame(self, self.mainFrame)
        self.statusFrame.grid(row=1, column=0, sticky=S+E+W)                

        self.buildCtrlFrame() 

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
                         


    def runMainLoop(self):
        while self.running:
            self.main.update_idletasks()            
            self.main.update()                  
            self.processMessages()      
        
        self.main.destroy()        


class rosLoop():
    def __init__(self, ocsQueue, rosQueue):
        self.ocsQueue = ocsQueue
        self.rosQueue = rosQueue
        self.setupPublishers()
        self.setupSubscribers()
        self.setupRosQueueCallbacks()

    def setupRosQueueCallbacks(self):
        self.rqC = []
        self.rqC.append((type(RunStop()), partial(self.runStopPub.publish)))

    def setupPublishers(self):
        self.runStopPub = rospy.Publisher('runstop', RunStop, queue_size=1)

    def setupSubscribers(self):
        self.runStopSub = rospy.Subscriber("/runstop", RunStop, self.queueRosMsg)
        self.status = rospy.Subscriber("/status", Status, self.queueRosMsg)
        self.encoder = rospy.Subscriber("/encoder", Encoder, self.queueRosMsg)

    def queueRosMsg(self, msg):
        self.ocsQueue.put(["Msg", msg])                

    def process(self):        
        while (not self.rosQueue.empty()):            
            rosItem = self.rosQueue.get()                 

            for i in self.rqC:                
                if i[0] == type(rosItem):                    
                    i[1](rosItem)       
                    break     


global ocs

def runDashboard(ocsQueue, rosQueue):
    global ocs
    ocs = OCS(ocsQueue, rosQueue)
    ocs.runMainLoop()    
    gc.collect()
    rospy.loginfo("runDashboard is exiting")

if __name__ == '__main__':	
    global ocs
    rospy.init_node('osr_ocs')
    rospy.loginfo('osr ocs started')    

    ocsQueue = queue.Queue()
    rosQueue = queue.Queue()

    rloop = rosLoop(ocsQueue, rosQueue)
    
    ocsThread = threading.Thread(target=runDashboard, args=(ocsQueue, rosQueue))
    ocsThread.start()

    r = rospy.Rate(20)
    while ocsThread.is_alive() and not rospy.is_shutdown():                        
        r.sleep()
        rloop.process()           

    if ocsThread.is_alive():
        ocsQueue.put(("Quit", None))
        rospy.loginfo("join thread")
        ocsThread.join()

    

    rospy.loginfo('osr ocs exiting....')    
