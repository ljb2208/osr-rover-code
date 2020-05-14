#!/usr/bin/env python2

import rospy
import sys
from planner import Planner

if __name__ == '__main__':
    rospy.init_node('osr_planner')

    rospy.loginfo("Python version: " + str(sys.version))

    planner = Planner(True)

    rospy.spin()