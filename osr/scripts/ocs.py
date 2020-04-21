#!/usr/bin/env python

from tkinter import *
from tkinter import ttk
import rospy
import queue
import threading
import gc
from osr_msgs.msg import RunStop
from functools import partial

class runStop(Button):

    def __init__(self, parent, parentCtrl, text, width):
        self.stop = False
        self.parent = parent
        super().__init__(parentCtrl, text = text, width=width, command=self.onRunStop)

    def onRunStop(self):        
        rs = RunStop()
        rs.run = not self.stop
        rs.auto = False

        self.parent.outQueue.put(rs)

    def onRunStopChange(self, value):
        self.stop = value

        if self.stop:
            self["bg"] = "green"
            self["activebackground"] = "green"
        else:
            self["bg"] = "red"
            self["activebackground"] = "red"
        

    def onRunStopMsg(self, value):        
        self.onRunStopChange(value.run)


class OCS():

    def __init__(self, inQueue, outQueue):
        self.main = Tk()
        self.running = True
        self.inQueue = inQueue
        self.outQueue = outQueue
        
        self.buildMainWindow()
        self.buildCtrlFrame() 
        self.setupInboundMessageCBs()

    def setupInboundMessageCBs(self):       
        self.ibC = []
        self.ibC.append((type(RunStop()), partial(self.rs.onRunStopMsg)))
    
    def buildCtrlFrame(self):
        self.rs = runStop(self, self.ctrlFrame, "RunStop", 10)
        self.rs.grid(row=0, column=0, sticky=NE)

        # add quit button    
        button = Button(self.ctrlFrame, text="Quit", width=10, command=self.close)
        button.grid(row=1, column=0, sticky=SE)

    def buildMainWindow(self):
        self.main.title("OSR OCS")    
        self.main.protocol("WM_DELETE_WINDOW", self.close)   

        self.mainFrame = Frame(self.main)
        self.ctrlFrame = Frame(self.main)        

        self.mainFrame.grid(row=0, column=0, sticky=NSEW)        
        self.ctrlFrame.grid(row=0, column=1, sticky=N+S+W)
        

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
        self.runStopSub = rospy.Subscriber("/runstop", RunStop, self.onRunStopMsg)

    def queueRosMsg(self, msg):
        self.ocsQueue.put(["Msg", msg])

    def onRunStopMsg(self, value):   
        self.queueRosMsg(value)     
        

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
