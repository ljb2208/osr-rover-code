#!/usr/bin/env python
from roboclaw import Roboclaw
import time
import serial
import math
import rospy

class MotorControllersV2(object):

	'''
	Motor class contains the methods necessary to send commands to the motor controllers

	for the corner and drive motors. There are many other ways of commanding the motors

	from the RoboClaw, we suggest trying to write your own Closed loop feedback method for

	the drive motors!

	'''
	def __init__(self):
		## MAKE SURE TO FIX CONFIG.JSON WHEN PORTED TO THE ROVER!
		#self.rc = Roboclaw( config['CONTROLLER_CONFIG']['device'],
		#					config['CONTROLLER_CONFIG']['baud_rate']
		#					)
		rospy.loginfo( "Initializing motor controllers")
		#self.rc = Roboclaw( rospy.get_param('motor_controller_device', "/dev/serial0"),
		#					rospy.get_param('baud_rate', 115200))
		self.rc = Roboclaw("/dev/ttyS0",115200)
		self.rc.Open()
		self.accel           = [0]    * 10
		self.qpps            = [None] * 10
		self.err             = [None] * 5
		
		address_raw = rospy.get_param('motor_controller_addresses')
		address_list = (address_raw.split(','))
		self.address = [None]*len(address_list)
		for i in range(len(address_list)):
			self.address[i] = int(address_list[i])

		version = 1
		for address in self.address:
			rospy.loginfo("Attempting to talk to motor controller: " + str(address))
			version = version & self.rc.ReadVersion(address)[0]
			rospy.loginfo("Motor controller version: " + str(version))
		if version != 0:
			rospy.loginfo("Sucessfully connected to RoboClaw motor controllers")
		else:
			rospy.logerr("Unable to establish connection to Roboclaw motor controllers")
			raise Exception("Unable to establish connection to Roboclaw motor controllers")
		self.killMotors()
		for address in self.address:
			self.rc.ResetEncoders(address)
		
		for address in self.address:
			self.rc.WriteNVM(address)

		for address in self.address:
			self.rc.ReadNVM(address)
		'''
		voltage = self.rc.ReadMainBatteryVoltage(0x80)[1]/10.0
		if voltage >= rospy.get_param('low_voltage',11):
			print "[Motor__init__] Voltage is safe at: ",voltage, "V"
		else:
			raise Exception("Unsafe Voltage of" + voltage + " Volts")
		'''
		i = 0

		for address in self.address:
			self.qpps[i]    = self.rc.ReadM1VelocityPID(address)[4]
			self.accel[i]   = int(self.qpps[i]*2)
			self.qpps[i+1]  = self.rc.ReadM2VelocityPID(address)[4]
			self.accel[i+1] = int(self.qpps[i]*2)
			i+=2
		accel_max = 655359
		accel_rate = 0.5
		self.accel_pos = int((accel_max /2) + accel_max * accel_rate)
		self.accel_neg = int((accel_max /2) - accel_max * accel_rate)
		self.errorCheck()				
		self.drive_enc = [None] * 10		
						
		time.sleep(2)
		self.killMotors()				


	def sendMotorDuty(self, motorID, speed):
		'''
		Wrapper method for an easier interface to control the drive motors,

		sends open-loop commands to the motors

		:param int motorID: number that corresponds to each physical motor
		:param int speed: Speed for each motor, range from 0-127

		'''
		#speed = speed/100.0
		#speed *= 0.5
		addr = self.address[int(motorID/2)]
		if speed > 0:
			if not motorID % 2: command = self.rc.ForwardM1
			else:               command = self.rc.ForwardM2
		else:
			if not motorID % 2: command = self.rc.BackwardM1
			else:               command = self.rc.BackwardM2

		speed = abs(int(speed * 127))

		return command(addr,speed)

	def sendSignedDutyAccel(self,motorID,speed):
		addr = self.address[int(motorID/2)]

		if speed >0: accel = self.accel_pos
		else: accel = self.accel_neg

		if not motorID % 2: command = self.rc.DutyAccelM1
		else:               command = self.rc.DutyAccelM2

		speed = int(32767 * speed/100.0)
		return command(addr,accel,speed)

	def getDriveEnc(self):
		drive_enc = []
		for i in range(10):
			index = int(math.ceil((i+1)/2.0)-1)
			if not (i % 2):
				drive_enc.append(self.rc.ReadEncM1(self.address[index])[1])
			else:
				drive_enc.append(self.rc.ReadEncM2(self.address[index])[1])
		self.drive_enc = drive_enc
		return drive_enc		

	def getBattery(self):
		return self.rc.ReadMainBatteryVoltage(self.address[0])[1]
		
	def getTemp(self):
		temp = [None] * 5
		for i in range(5):
			temp[i] = self.rc.ReadTemp(self.address[i])[1]
		return temp
	
	def getCurrents(self):
		currents = [None] * 10
		for i in range(5):
			currs = self.rc.ReadCurrents(self.address[i])
			currents[2*i] = currs[1]
			currents[(2*i) + 1] = currs[2]
		return currents

	def getErrors(self):
		return self.err

	def killMotors(self):
		'''
		Stops all motors on Rover
		'''
		for i in range(5):
			self.rc.ForwardM1(self.address[i],0)
			self.rc.ForwardM2(self.address[i],0)

	def errorCheck(self):
		'''
		Checks error status of each motor controller, returns 0 if any errors occur
		'''

		for i in range(len(self.address)):
			self.err[i] = self.rc.ReadError(self.address[i])[1]
		for error in self.err:
			if error:
				self.killMotors()
				#self.writeError()
				rospy.loginfo("Motor controller Error: " + str(error))
		return 1

	def writeError(self):
		'''
		Writes the list of errors to a text file for later examination
		'''

		f = open('errorLog.txt','a')
		errors = ','.join(str(e) for e in self.err)
		f.write('\n' + 'Errors: ' + '[' + errors + ']' + ' at: ' + str(datetime.datetime.now()))
		f.close()




