#!/usr/bin/env python2
from dynamicvoronoi import DynamicVoronoi
import rospy
from utils import Utils

class Map():
    def __init__(self):
        self.map = None
        self.cellSize = 1.
        self.valid = False
        self.voronoi = DynamicVoronoi()
        self.width = 0
        self.height = 0    
    
    def setMap(self, msg):
        self.valid = True

        self.width = msg.info.width
        self.height = msg.info.height
        
        binMap = Utils.createListOfLists(self.width, self.height, False)

        for x in range(self.width):
            for y in range(self.height):                                
                val = msg.data[y * self.width + x]

                if val > 0:
                    binMap[x][y] = True        
                

        self.voronoi.initializeMap(self.width, self.height, binMap)
        self.voronoi.update()
        self.voronoi.vizualize()

        rospy.loginfo("setMap complete")