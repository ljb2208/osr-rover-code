#!/usr/bin/env python2

import rospy
import tf_conversions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from map import Map

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