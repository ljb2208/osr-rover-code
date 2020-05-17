import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from constants import *
from nodes import Node3D
import tf

class PathFinder():
    def __init__(self, smoothed=False):
        self.smoothed = smoothed
        self.pathTopic = "/path"
        self.pathNodesTopic = "/pathNodes"
        self.pathVehicleTopic = "/pathVehicles"
        self.path = Path()
        self.pathNodes = MarkerArray()
        self.pathVehicles = MarkerArray()

        if smoothed:
            self.pathTopic = "/sPath"
            self.pathNodesTopic = "/sPathNodes"
            self.pathVehicleTopic = "/sPathVehicles"

        self.pubPath = rospy.Publisher(self.pathTopic, Path, 1)
        self.pubPathNodes = rospy.Publisher(self.pathNodesTopic, MarkerArray, 1)
        self.pubPathVehicles = rospy.Publisher(self.pathVehicleTopic, MarkerArray, 1)

        self.path.header.frame_id = "path"

    def publishPath(self):
        self.pubPath.publish(self.path)

    def publishPathNodes(self):
        self.pubPathNodes.publish(self.pathNodes)

    def publishPathVehicles(self):
        self.pubPathVehicles.publish(self.pathVehicles)

    def clear(self):
        node = Node3D(0, 0, 0, 0, 0, None, None)
        self.path.poses.clear()
        self.pathNodes.markers.clear()
        self.pathVehicle.markers.clear()
        self.addNode(node, 0)
        self.addVehicle(node, 1)
        self.publishPath()
        self.publishPathNodes()
        self.publishPathVehicles()
    
    def updatePath(self, nodesPath):
        self.path.header.stamp = rospy.Time.now()
        k = 0

        for nodePath in nodesPath:
            self.addSegment(nodePath)
            self.addNode(nodePath, k)
            k += 1
            self.addVehicle(nodePath, k)
            k += 1

    def addSegment(self, node):
        vertex = PoseStamped()
        vertex.pose.position.x = node.x * CELL_SIZE
        vertex.pose.position.y = node.y * CELL_SIZE
        vertex.pose.position.z = 0
        vertex.pose.orientation.x = 0
        vertex.pose.orientation.y = 0
        vertex.pose.orientation.z = 0
        vertex.pose.orientation.w = 0
        self.path.poses.append(vertex)

    def addNode(self, node, i):
        pathNode = Marker()

        if i == 0:
            pathNode.action = 3

        pathNode.header.frame_id = "path"
        pathNode.header.stamp = rospy.Time(0)
        pathNode.id = i
        pathNode.type = 7 # sphere
        pathNode.scale.x = 0.1
        pathNode.scale.y = 0.1
        pathNode.scale.z = 0.1
        pathNode.color.a = 1.0

        if self.smoothed:
            pathNode.color.r = COLOR_PINK[0]
            pathNode.color.g = COLOR_PINK[1]
            pathNode.color.b = COLOR_PINK[2]
        else:
            pathNode.color.r = COLOR_PURPLE[0]
            pathNode.color.g = COLOR_PURPLE[1]
            pathNode.color.b = COLOR_PURPLE[2]

        pathNode.pose.position.x = node.x * CELL_SIZE
        pathNode.pose.position.y = node.y * CELL_SIZE
        self.pathNodes.append(pathNode)

    def addVehicle(self, node, i)
        pathVehicle = Marker()

        if i == 1:
            pathVehicle.action = 3

        pathVehicle.header.frame_id = "path"
        pathVehicle.header.stamp = rospy.Time(0)
        pathVehicle.id = i
        pathVehicle.type = 1 # cube
        pathVehicle.scale.x = LENGTH - BLOATING * 2
        pathVehicle.scale.y = WIDTH - BLOATING * 2
        pathVehicle.scale.z = 1
        pathVehicle.color.a = 0.1

        if self.smoothed:
            pathVehicle.color.r = COLOR_ORANGE[0]
            pathVehicle.color.g = COLOR_ORANGE[1]
            pathVehicle.color.b = COLOR_ORANGE[2]
        else:
            pathVehicle.color.r = COLOR_TEAL[0]
            pathVehicle.color.g = COLOR_TEAL[1]
            pathVehicle.color.b = COLOR_TEAL[2]

        pathVehicle.pose.position.x = node.x * CELL_SIZE
        pathVehicle.pose.position.y = node.y * CELL_SIZE
        pathVehicle.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, node.t)
        self.pathVehicles.append(pathVehicle)


    
    