
#!/usr/bin/env python
import rospy
import time
import math

class RobotV2():
	'''
	Robot class contains all the math and motor control algorithms to move the rover

	In order to call command the robot the only method necessary is the sendCommands() method

	with drive velocity and turning amount


	'''
	def __init__(self):		
		self.maxAngle = float(rospy.get_param('max_angle', 50.))
		self.wheelBase = float(rospy.get_param('wheel_base', 268.))
		self.wheelTrack = float(rospy.get_param('wheel_track', 440.))

		self.angleInc = math.radians(self.maxAngle) / 250.

		self.frIndex = 0
		self.brIndex = 2
		self.flIndex = 5
		self.blIndex = 8

	def calculateRotationVelocity(self, r):
		# returns velocity for rotations in place
		vel = [0] * 10		
		vel[self.frIndex] = -r
		vel[self.brIndex] = -r
		vel[self.flIndex] = r
		vel[self.blIndex] = r
		
		return vel	
		
	def calculateVelocity(self,v,r):
		'''
		Returns a list of speeds for each individual drive motor based on current turning radius

		:param int v: Drive speed command range from -100 to 100
		:param int r: Current turning radius range from -250 to 250

		'''
		vel = [0]*10

		if (v == 0):
			return vel

		if (abs(r) <= 5):
			return vel


		# calculate turn radius
		radius = math.tan(abs(r) * self.angleInc) / self.wheelBase

		innerRadius = radius - self.wheelTrack/2
		outerRadius = radius + self.wheelTrack/2

		ratio = innerRadius / outerRadius

		if r > 0:
			vel[self.frIndex] = v * ratio
			vel[self.brIndex] = v * ratio
			vel[self.flIndex] = v
			vel[self.blIndex] = v
		else:
			vel[self.frIndex] = v
			vel[self.brIndex] = v
			vel[self.flIndex] = v * ratio
			vel[self.blIndex] = v * ratio

		return vel
			
	def generateCommands(self,v,r,rotation):
		'''
		Driving method for the Rover, rover will not do any commands if any motor controller
		throws an error

		:param int v: driving velocity command, % based from -100 (backward) to 100 (forward)
		:param int r: driving turning radius command, % based from -100 (left) to 100 (right)
		:param int rotation: rotation command for turning in place % based from -100 (left) to 100 (right)

		'''
		#fix params list above ^^
		#cur_radius = self.approxTurningRadius(self.getCornerDeg(encs))
		if rotation != 0:
			velocity = self.calculateRotationVelocity(rotation)
			rospy.loginfo("Rotation velocity: " + str(velocity))
		else:
			velocity   = self.calculateVelocity(v,r)

		rospy.loginfo("velocity: " + str(velocity))		

		return velocity


