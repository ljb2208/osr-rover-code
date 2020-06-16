#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from osr_msgs.msg import Joystick, RunStop
from std_msgs.msg import Int64MultiArray
import math

global mode
global last
global last_runstop
global counter
global runstop
global auto
global last_auto

mode,counter = 0,0
last = time.time()
last_runstop = time.time()
last_auto = time.time()
runstop = False
auto = False


def callback(data):
	global mode
	global counter
	global last
	global last_runstop
	global runstop
	global auto
	global last_auto

	publish_runstop = False

	joy_out = Joystick()

	y =  data.axes[1]	
	x =-data.axes[3]
	lt = data.axes[2]
	rt = data.axes[5]

	now = time.time()

	if (data.buttons[0] == 1):
		if (now - last_runstop > 0.75):
			publish_runstop = True
			runstop = not runstop
			last_runstop = time.time()

	if (data.buttons[1] == 1):
		if (now - last_auto > 0.75):
			publish_runstop = True
			auto = not auto
			last_auto = time.time()	

	dpad = data.buttons[11:]
	if 1 in dpad: mode = dpad.index(1)
		
	last = time.time()		

	if (publish_runstop):
		rs = RunStop()
		rs.run = runstop
		rs.auto = auto
		runstop_pub.publish(rs)

	rot = rotate(x, rt)
	cmd = two_joy(x,y,lt)
	joy_out = Joystick()
	joy_out.rotation = rot

	if rot == 0:
		joy_out.vel = cmd[0]
		joy_out.steering = cmd[1]
	else:
		joy_out.vel = 0
		joy_out.steering = 0

	joy_out.mode = mode
	joy_out.connected = True
	pub.publish(joy_out)


def rotate(x, rt):
	if rt > 0:
		return 0

	x *= 100
	return int(x)

def two_joy(x,y,lt):
	boost = 0
	if lt <= 0:
		boost = 50*-lt
	if y >=0: y = (y * 50) + boost
	else: y = (y *50) - boost
	x *= 100
	return (int(y),int(x))

if __name__ == '__main__':
	global pub
	global runstop_pub
	
	rospy.init_node('joystick')
	rospy.loginfo('joystick started')

	sub = rospy.Subscriber("/joy", Joy, callback)
	pub = rospy.Publisher('joystick', Joystick, queue_size=1)
	runstop_pub = rospy.Publisher('runstop', RunStop, queue_size=1)

	rospy.spin()
