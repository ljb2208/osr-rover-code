#!/usr/bin/env python2

import rospy
from constants import *
from ompl import base as ob
from ompl import geometric as og

class HybridAStar():
    def __init__(self, configurationSpace, dubinsLookup, visualization):        
        self.configurationSpace = configurationSpace
        self.dubinsLookup = dubinsLookup
        self.visualization = visualization

    def runAlgo(self, nStart, nGoal, nodes3D, nodes2D, width, height):
        self.nStart = nStart
        self.nGoal = nGoal
        self.nodes3D = nodes3D
        self.nodes2D = nodes2D
        self.width = width
        self.height = height

        dir = 6 # TODO support parameterization

        if not REVERSE:
            dir = 3

        iterations = 0
        
        d = rospy.Duration(0.003)

    def updateH(self):
        dubinsCost = 0.
        reedsSheppCost = 0.
        twoDCost = 0.
        twoDOffset = 0.

        if DUBINS:
            dbStart = ob.SE2StateSpace()
        elif REVERSE:

        if TWOD:



