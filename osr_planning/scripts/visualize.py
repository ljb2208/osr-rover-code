#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from constants import *
import math
import tf
from utils import Utils
from colorgradient import ColorGradient

class Visualize():
    def __init__(self):
        self.pubNode3D = rospy.Publisher("/visualizeNodes3DPose", PoseStamped, queue_size=100)
        self.pubNodes3D = rospy.Publisher("/visualizeNodes3DPoses", PoseArray, queue_size=100)
        self.pubNodes3DReverse = rospy.Publisher("/visualizeNodes3DPosesReverse", PoseArray, queue_size=100)
        self.pubNodes3DCosts = rospy.Publisher("/visualizeNodes3DCosts", MarkerArray, queue_size=100)
        self.pubNode2D = rospy.Publisher("/visualizeNodes2DPose", PoseStamped, queue_size=100)
        self.pubNodes2D = rospy.Publisher("/visualizeNodes2DPoses", PoseArray, queue_size=100)
        self.pubNodes2DCosts = rospy.Publisher("/visualizeNodes2DCosts", MarkerArray, queue_size=100)

        self.poses3D = PoseArray()
        self.poses3D.header.frame_id = "path"

        self.poses3Dreverse = PoseArray()
        self.poses3Dreverse.header.frame_id = "path"

        self.poses2D = PoseArray()
        self.poses2D.header.frame_id = "path"

        self.heatMapGradient = ColorGradient()

    def clear(self):
        self.poses3D.poses.clear()
        self.poses3Dreverse.poses.clear()
        self.poses2D.poses.clear()

        costCubes3D = MarkerArray()
        costCube3D = Marker()

        # clear the costs heat map
        costCube3D.header.frame_id = "path"
        costCube3D.header.stamp = rospy.Time.now()
        costCube3D.id = 0
        costCube3D.action = 3

        costCubes3D.markers.append(costCube3D)
        self.pubNodes3DCosts.publish(costCubes3D)

        costCubes2D = MarkerArray()
        costCube2D = Marker()

        # clear the 2D costs heat map
        costCube2D.header.frame_id = "path"
        costCube2D.header.stamp = rospy.Time.now()
        costCube2D.id = 0
        costCube2D.action = 3

        costCubes2D.markers.append(costCube2D)
        self.pubNodes2DCosts.publish(costCubes2D)

    def publishNode3DPose(self, node):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "path"
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.header.seq = 0
        poseMsg.pose.position.x = node.x * CELL_SIZE
        poseMsg.pose.position.y = node.y * CELL_SIZE

        
        if node.prim < 3:
            # forward
            poseMsg.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, node.t)
        else:
            #reverse
            poseMsg.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, node.t + math.pi)

        self.pubNode3D.publish(poseMsg)

    def publishNode3DPoses(self, node):

        poseMsg = Pose()
        poseMsg.position.x = node.x * CELL_SIZE
        poseMsg.position.y = node.y * CELL_SIZE

        if node.prim < 3:
            #forward
            poseMsg.orientation = tf.transformations.quaternion_from_euler(0, 0, node.t)
            self.poses3D.poses.append(poseMsg)
            self.poses3D.header.stamp = rospy.Time.now()            
            self.pubNodes3D.publish(self.poses3D)
        else:
            # reverse
            poseMsg.orientation = tf.transformations.quaternion_from_euler(0, 0, node.t + math.pi)
            self.poses3D.poses.append(poseMsg)
            self.poses3D.header.stamp = rospy.Time.now()            
            self.pubNodes3DReverse.publish(self.poses3D)

    def publishNode2DPose(self, node):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "path"
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.header.seq = 0
        poseMsg.pose.position.x = (node.x + 0.5) * CELL_SIZE
        poseMsg.pose.position.y = (node.y + 0.5) * CELL_SIZE
        poseMsg.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

        self.pubNode2D.publish(poseMsg)

    def publishNode2DPoses(self, node):
        if not node.discovered():
            return
        
        poseMsg = Pose()
        poseMsg.position.x = (node.x + 0.5) * CELL_SIZE
        poseMsg.position.y = (node.y + 0.5) * CELL_SIZE
        poseMsg.orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

        self.poses2D.poses.append(poseMsg)
        self.poses2D.header.stamp = rospy.Time.now()

        self.pubNodes2D.publish(self.poses2D)

    def publishNode3DCosts(self, nodes, width, height, depth):
        costCubes = MarkerArray()
        costCube = Marker()

        min = 1000.
        max = 0.
        count = 0
        once = True

        values = Utils.createList(width * height, 0.)

        #determine min and max values
        for i in range(0, (width * height)):
            values[i] = 1000
            
            for k in depth:
                idx = k * width * height + i

                if nodes[idx].isClosed() or nodes[idx].isOpen():
                    values[i] = nodes[idx].c

                if values[i] > 0 and values[i] < min:
                    min = values[i]

                if values[i] > 0 and values[i] > max and values[i] != 1000:
                    max = values[i]

        # paint the cubes
        for i in range(0, (width * height)):
            if values[i] != 1000:
                count += 1

                if once:
                    costCube.action = 3
                    once = False
                else:
                    costCube.action = 0

                costCube.header.frame_id = "path"
                costCube.header.stamp = rospy.Time.now()
                costCube.id = i
                costCube.type = 1 # cube
                values[i] = (values[i] - min) / (max - min)
                costCube.scale.x = CELL_SIZE
                costCube.scale.y = CELL_SIZE
                costCube.scale.z = 0.1
                costCube.scale.a = 0.5                
                costCube.color.r, costCube.color.g, costCube.color.b = self.heatMapGradient.getColorAtValue(values[i])

                costCube.pose.position.x = (i % width + 0.5) * CELL_SIZE
                costCube.pose.position.y = ((i / width) % height + 0.5) * CELL_SIZE
                costCubes.markers.append(costCube)

        self.pubNodes3DCosts.publish(costCubes)


    def publishNode2DCosts(self, nodes, width, height):
        costCubes = MarkerArray()
        costCube = Marker()

        min = 1000.
        max = 0.
        count = 0
        once = True

        values = Utils.createList(width * height, 0.)

        #determine min and max values
        for i in range(0, (width * height)):
            values[i] = 1000            

            if nodes[i].isDiscovered():
                values[i] = nodes[i].c

            if values[i] > 0 and values[i] < min:
                min = values[i]

            if values[i] > 0 and values[i] > max:
                max = values[i]

        # paint the cubes
        for i in range(0, (width * height)):
            if nodes[i].isDiscovered:
                count += 1

                if once:
                    costCube.action = 3
                    once = False
                else:
                    costCube.action = 0

                costCube.header.frame_id = "path"
                costCube.header.stamp = rospy.Time.now()
                costCube.id = i
                costCube.type = 1 # cube
                values[i] = (values[i] - min) / (max - min)
                costCube.scale.x = CELL_SIZE
                costCube.scale.y = CELL_SIZE
                costCube.scale.z = 0.1
                costCube.scale.a = 0.5                
                costCube.color.r, costCube.color.g, costCube.color.b = self.heatMapGradient.getColorAtValue(values[i])

                costCube.pose.position.x = (i % width + 0.5) * CELL_SIZE
                costCube.pose.position.y = ((i / width) % height + 0.5) * CELL_SIZE
                costCubes.markers.append(costCube)

        self.pubNodes2DCosts.publish(costCubes)

