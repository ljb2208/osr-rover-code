#!/usr/bin/env python2
import rospy
from nav_msgs.msg import OccupancyGrid


class CollisionDetection():
    def __init__(self):
        self.grid = OccupancyGrid()
    