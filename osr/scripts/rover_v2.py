#!/usr/bin/env python
import time
from osr_msgs.msg import Joystick, Commands, Encoder, RunStop
import rospy
from robot_v2 import RobotV2
import message_filters


global encs
global osr
global runstop
global auto
global valid_encs
osr = RobotV2()
runstop = False
auto = False

def joy_callback(message):		
	global auto
	global runstop
	cmds = Commands()

	if (auto == False):
		out_cmds = osr.generateCommands(message.vel,message.steering,message.rotation)	
	else:
		return

	# if runstop false, then assume zero velocity and steering
	# if (runstop == False):
	# 	out_cmds = osr.generateCommands(0,0,encs)	


	cmds.drive_motor  = out_cmds	
	
	try:		
		pub.publish(cmds)		
	except:
		pass

def runstop_callback(message):
	global runstop
	global auto
	runstop = message.run
	auto = message.auto

if __name__ == '__main__':
	rospy.init_node('roverv2')
	rospy.loginfo("Starting the roverv2 node")
	global pub
	joy_sub = rospy.Subscriber("/joystick",Joystick, joy_callback)	
	runstop_sub = rospy.Subscriber("/runstop", RunStop, runstop_callback)
	rate = rospy.Rate(10)
	#time_sync = message_filters.TimeSynchronizer([joy_sub, mc_sub],10)
	#time_sync.registerCallback(callback)

	pub = rospy.Publisher("/robot_commands", Commands, queue_size = 1)
	
	rate.sleep()
	rospy.spin()


