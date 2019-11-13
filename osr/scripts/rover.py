#!/usr/bin/env python
import time
from osr_msgs.msg import Joystick, Commands, Encoder, RunStop
import rospy
from robot import Robot
import message_filters


global encs
global osr
global runstop
global valid_encs
osr = Robot()
encs = [0]*4
runstop = False
valid_encs = False

def joy_callback(message):
	global encs
	global runstop
	cmds = Commands()

	if (valid_encs):
		out_cmds = osr.generateCommands(message.vel,message.steering,encs)	
	else:
		return

	# if runstop false, then assume zero velocity and steering
	# if (runstop == False):
	# 	out_cmds = osr.generateCommands(0,0,encs)	


	cmds.drive_motor  = out_cmds[0]
	cmds.corner_motor = out_cmds[1]

	rospy.logdebug("Requested commands: " + str(out_cmds[1]) + " encs: " + str(encs))
	try:		
		pub.publish(cmds)		
	except:
		pass

def enc_callback(message):
	global encs
	global valid_encs
	temp = [0]*4
	valid = True
	for i in range(4):
		temp[i] = message.abs_enc[i] + 1
		if (temp[i] < 1):
			valid = False

	encs = temp
	valid_encs = valid

def runstop_callback(message):
	global runstop
	runstop = message.run

if __name__ == '__main__':
	rospy.init_node('rover')
	rospy.loginfo("Starting the rover node")
	global pub
	joy_sub = rospy.Subscriber("/joystick",Joystick, joy_callback)
	enc_sub  = rospy.Subscriber("/encoder", Encoder, enc_callback)
	runstop_sub = rospy.Subscriber("/runstop", RunStop, runstop_callback)
	rate = rospy.Rate(10)
	#time_sync = message_filters.TimeSynchronizer([joy_sub, mc_sub],10)
	#time_sync.registerCallback(callback)

	pub = rospy.Publisher("/robot_commands", Commands, queue_size = 1)
	
	rate.sleep()
	rospy.spin()


