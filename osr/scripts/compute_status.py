#!/usr/bin/env python
import rospy
import time
import subprocess
import string
from osr_msgs.msg import CompStatus

class MachineStats():
    def __init__(self):
        self.hostName = ""
        self.memTotal = ""
        self.memFree = ""        
        self.memPct = 0.
        self.statTime = ""
        self.cpu = ["", "", ""]
        self.ssid = ""
        self.bitRate = ""
        self.linkQuality = ""

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

    def parseWifiStats(self, stats):
        for line in stats:
            if "ESSID:" in line:
                sInd = line.find("ESSID:")
                eInd = line.find('"',  sInd+7)

                self.ssid = line[sInd+7: eInd]

            elif "Bit Rate=" in line:
                sInd = line.find("Bit Rate=")
                eInd = line.find('Tx',  sInd+9)

                self.bitRate = line[sInd+9: eInd]

            elif "Link Quality" in line:
                sInd = line.find("Link Quality=")
                eInd = line.find("Signal",  sInd+13)

                self.linkQuality = line[sInd+13: eInd]


    
    def parseTime(self, stats):
        for line in stats:
            if "top -" in line:
                sInd = line.find("top -")
                eInd = line.find("up")
                line = line[sInd + 6:eInd]
                self.statTime = line.strip()
                return

    def parseHostName(self, stats):
        self.hostName = stats

    def getMessage(self):
        msg = CompStatus()
        msg.host_name = self.hostName
        msg.link_quality = self.linkQuality
        msg.ssid = self.ssid
        msg.bit_rate = self.bitRate
        msg.mem_total = self.memTotal
        msg.mem_free = self.memFree
        msg.mem_pct = self.memPct
        msg.stat_time = self.statTime
        msg.cpu1 = self.cpu[0]
        msg.cpu2 = self.cpu[1]
        msg.cpu3 = self.cpu[2]

        return msg

class ComputerStatus():
    def __init__(self, compName):
        self.compName = compName
        self.machineStats = MachineStats()
        self.pub = rospy.Publisher(compName + "/comp_status", CompStatus)

    def runCommand(self, command):
        retval = -1

        try:
            retval = subprocess.run(command, bufsize=4096, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)                        
        except Exception:
            retval = None            

        return retval

    def getHostName(self):
        command = ["hostname"]
        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseHostName(str(retval.stdout))

    def getWifiStats(self):
        command = ["sudo", "iwconfig"]

        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseWifiStats(str(retval.stdout))

    def getCompStats(self):
        command = ["top -n 1"]

        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseStats(str(retval.stdout))        

    def publishStatus(self):
        self.getHostName()
        self.getWifiStats()
        self.getCompStats()
        
        self.pub.publish(self.machineStats.getMessage())

if __name__ == '__main__':
    rospy.init_node("comp_status")

    freq = rospy.get_param("~freq", 0.2)
    compName = rospy.get_param("~comp_name", "xavier-osr")

    comp = ComputerStatus(compName)
    r = rospy.Rate(freq)

    while not rospy.is_shutdown():
        r.sleep()
        comp.publishStatus()






