#!/usr/bin/env python2

import rospy
import tf_conversions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from map import Map
from constants import *
from nodes import *
from visualize import Visualize
from path import PathFinder
from smoother import Smoother

class Goal():
    def __init__(self, map):
        self.x = 0.
        self.y = 0.
        self.t = 0.
        self.map = map
        self.valid = False

    def setGoal(self, msg):
        self.x = msg.pose.position.x / self.map.cellSize
        self.y = msg.pose.position.y / self.map.cellSize        
        roll, pitch, self.t = tf_conversions.transformations.euler_from_quaternion(msg.pose.orientation)
        self.valid = True

class Pose():
    def __init__(self, map):
        self.x = 0.
        self.y = 0.
        self.t = 0.
        self.map = map
        self.valid = False

    def setPose(self, msg):
        self.x = msg.pose.pose.position.x / self.map.cellSize
        self.y = msg.pose.pose.position.y / self.map.cellSize
        roll, pitch, self.t = tf_conversions.transformations.euler_from_quaternion(msg.pose.pose.orientation)
        self.valid = True
        

class Planner():
    def __init__(self, manual):
        self.map = Map()
        self.manual = manual

        self.goal = Goal(self.map)
        self.pose = Pose(self.map)

        self.visualization = Visualize()
        self.smoother = Smoother()

        self.path = PathFinder()
        self.smoothedPath = PathFinder(smoothed=True)


        self.subGoal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.onGoal)
        self.subInitPose = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.onInitialPose)
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.onMap)
        

    def onGoal(self, msg):        
        rospy.loginfo("Received new goal")
        self.goal.setGoal(msg)

        if self.manual:
            self.plan()

    def onInitialPose(self, msg):
        rospy.loginfo("Received initial pose")

        self.pose.setPose(msg)

        if self.manual:
            self.plan()

    def onMap(self, msg):
        rospy.loginfo("Received map")

        self.map.setMap(msg)

        if self.manual:
            self.plan()

    def canPlan(self):
        if self.map.valid and self.goal.valid and self.map.valid:
            return True
        
        return False

    def plan(self):
        if not self.canPlan():
            rospy.loginfo("Cannot plan yet")

        width = self.map.width
        height = self.map.height
        depth = HEADINGS
        length = width * height * depth

        nodes3D = [None] * length
        nodes2D = [None] * (width * height)

        nGoal = Node3D(self.goal.x, self.goal.y, self.goal.t, 0, 0, None, None)
        nStart = Node3D(self.pose.x, self.pose.y, self.pose.t, 0, 0, None, None)

        t0 = rospy.Time.now()

        # clear visualization
        self.visualization.clear()
        self.path.clear()
        self.smoothedPath.clear()

        # find path
        solution = []

        # trace the plan
        self.smoother.tracePath(solution)
        
        # create updated path
        self.path.updatePath(self.smoother.getPath())

        # smooth path
        self.smoother.smoothPath(self.map.voronoi)

        self.smoothedPath.updatePath(self.smoother.getPath())
        
        t1 = rospy.Time.now()
        d = rospy.Duration(t1 - t0)

        rospy.loginfo("Planning time (ms): " + str(d))

        self.path.publishPath()
        self.path.publishPathNodes()
        self.path.publishPathVehicles()

        self.smoothedPath.publishPath()
        self.smoothedPath.publishPathNodes()
        self.smoothedPath.publishPathVehicles()

        self.visualization.publishNode3DCosts(nodes3D, width, height, depth)
        self.visualization.publishNode2DCosts(nodes2D, width, height)        
        


