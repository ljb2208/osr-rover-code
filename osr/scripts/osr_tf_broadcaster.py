#!/usr/bin/env python  
   
import rospy
import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped

class OSRTf():
    def __init__(self, parentFrame, childFrame, x, y, z, roll, pitch, yaw):
        self.parentFrame = parentFrame
        self.childFrame = childFrame

        self.msg = TransformStamped()
        self.msg.header.frame_id = parentFrame
        self.msg.child_frame_id = childFrame
        self.msg.transform.translation.x = x
        self.msg.transform.translation.y = y
        self.msg.transform.translation.z = z

        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

        self.msg.transform.rotation.x = q[0]
        self.msg.transform.rotation.y = q[1]
        self.msg.transform.rotation.z = q[2]
        self.msg.transform.rotation.w = q[3]      

    def getTFMsg(self):        
        self.msg.header.stamp = rospy.Time.now()
        return msg

    

class TfBroadCaster():
    def __init__(self):
        tfList = [OSRTf("base_link", "imu_link", 0.12, 0, 0.50, 0, 0, 0)]
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

    def publishTFs(self):
        for tfItem in tfList:
            self.tfBroadcaster.publishTransform(tfItem.getTFMsg())



if __name__ == "__main__":
    rospy.init_node("osr_tf_broadcaster")

    freq = rospy.get_param("~freq", 50)
    rate = rospy.Rate(freq)

    tfBroadcaster = TfBroadCaster()

    while not rospy.is_shutdown:
        tfBroadcaster.publishTFs()
        rate.sleep()



