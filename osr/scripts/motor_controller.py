#!/usr/bin/env python
import time
import rospy
from osr_msgs.msg import Commands, Encoder, Status, RunStop
from roboclaw_wrapper import MotorControllers

global mutex
global runstop
runstop = False
mutex = False
motorcontrollers = MotorControllers()

def callback(cmds):
	global mutex	
	global runstop	

	rospy.logdebug("Commands: " + str(cmds))
	rospy.logdebug("Runstop: " + str(runstop))
	while mutex:
		time.sleep(0.001)
		#print "cmds are being buffered"
	mutex = True
	# PUT THIS BACK IN
	motorcontrollers.cornerToPosition(cmds.corner_motor)
	for i in range(6):
		# PUT THIS BACK IN
		#motorcontrollers.sendMotorDuty(i,cmds.drive_motor[i])
		if (runstop == False):
			motorcontrollers.sendSignedDutyAccel(i,0)	
		else:
			motorcontrollers.sendSignedDutyAccel(i,cmds.drive_motor[i])
		pass
	mutex = False

def runstop_callback(message):
	global runstop
	runstop = message.run

def shutdown():
	rospy.loginfo("killing motors")
	motorcontrollers.killMotors()

if __name__ == "__main__":

	rospy.init_node("motor_controller")	
	rospy.loginfo("Starting the motor_controller node")
	rospy.on_shutdown(shutdown)
	runstop_sub = rospy.Subscriber("/runstop", RunStop, runstop_callback)
	sub = rospy.Subscriber("/robot_commands",Commands,callback)
	enc_pub = rospy.Publisher("/encoder", Encoder, queue_size =1)
	status_pub = rospy.Publisher("/status", Status, queue_size =1)

	freq = rospy.get_param("~freq", 20)

	rate = rospy.Rate(freq)

	status = Status()
	enc   = Encoder()
	
	enc.abs_enc      		=[1000]*4
	enc.abs_enc_angles 		=[-100]*4
	status.battery          = 0
	status.temp         	=[0]*5
	status.current      	=[0]*10
	status.error_status 	=[0]*5
	
	counter = 0
	while not rospy.is_shutdown():

		while mutex:
			time.sleep(0.001)
		mutex = True
		enc.header.stamp = rospy.Time.now()
		enc.abs_enc = motorcontrollers.getCornerEnc()
		enc.rel_enc = motorcontrollers.getDriveEnc()		
		# enc.header.stamp = rospy.Time.now() 
		
		#mc_data.abs_enc_angles = motorcontrollers.getCornerEncAngle()
		if (counter >= 10):
			
			status.battery = motorcontrollers.getBattery()
			status.temp = motorcontrollers.getTemp()
			status.current = motorcontrollers.getCurrents()
			status.error_status = motorcontrollers.getErrors()
			status_pub.publish(status)
			counter = 0
			
		mutex = False
		enc_pub.publish(enc)
		counter += 1
		rate.sleep()
	
