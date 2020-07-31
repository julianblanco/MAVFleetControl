from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, VelocityNedYaw,PositionNedYaw)
import numpy as np
import asyncio
def cart2pol(x, y):
	rho = np.sqrt(x**2 + y**2)
	phi = np.arctan2(y, x)
	return(rho, phi)

def pol2cart(rho, phi):
	x = rho * np.cos(phi)
	y = rho * np.sin(phi)
	return(x, y)

def angular_diff(p1,p2):
	diff = p2 - p1
	if diff > 180:
		diff = diff -360
	else:
		if diff < -180:
			diff = diff +360
	# print(f"{diff}diff")
	return diff
def saturate(lower,upper,value):
	if(value>upper):
		value = upper
	if(value<lower):
		value = lower
	return value
def distance_between(p1,p2):
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return np.sqrt(squared_dist)

class Circle:

	def __init__(self, velocity: float = 1.0, radius: float =1.0 ,angle: float = 0.0,direction:str = 'cw',numloops: int = 1,tolerance: int =2):
		self.velocity = velocity
		self.radius = radius
		self.angle = angle# self.fleet = []
		self.desiredLoops = numloops
		self.currentLoops = 0
		self.tolerance = tolerance
		self.otherside_flag =1
		if direction == 'ccw':
			self.direction = True
		else:
			self.direction = False
		# self.master = None

	async def __call__(self, drone):

		await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])

		await drone.start_offboard()
		print("-- Starting Circle")
	
		#set aircraft to flip at 300 deg/s to the right (roll)
		async for position_ned in drone.conn.telemetry.position_velocity_ned():
			start_position = np.array([position_ned.position.north_m,position_ned.position.east_m])#,position_ned.position.down_m])
			start_height = position_ned.position.down_m
			# print('hi1')
			break

		circle_center = np.array([start_position[0] + self.radius,start_position[1]])
		circle_otherside = np.array([start_position[0] + 2*self.radius,start_position[1]])
		# print('hi')


		async for position_ned in drone.conn.telemetry.position_velocity_ned():
			currentposn = np.array([position_ned.position.north_m,position_ned.position.east_m])
			angle_to_center = np.arctan2(*(circle_center-currentposn)[::-1])
			distance2center = np.linalg.norm(circle_center-currentposn)
			distanceController = self.radius - distance2center
			if self.direction:
				tangentangle = angle_to_center + ( np.pi / 2 ) 
			else:
				tangentangle = angle_to_center + ( np.pi / 2 ) + np.pi
			# print('-------')
			# print(tangentangle*180/np.pi)
			# print((tangentangle+np.pi)*180/np.pi )
			if self.direction:
				newangle = tangentangle + distanceController/2.0
			else:
				newangle = tangentangle - distanceController/2.0

			xvelocity = np.cos(newangle)*self.velocity
			yvelocity = np.sin(newangle)*self.velocity

			zdelta = position_ned.position.down_m - start_height
			zvelocity = saturate(-0.1,0.1,(zdelta*-0.5))

			await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,zvelocity,0.0))
			if self.otherside_flag:
				if distance_between(circle_otherside, currentposn) < self.tolerance:
					print("otherside")
					self.otherside_flag = 0
			else:
				if distance_between(start_position, currentposn) < self.tolerance:
					self.otherside_flag = 1
					self.currentLoops += 1

			if self.currentLoops >= self.desiredLoops:
				print(f"{self.desiredLoops} completed!")
				# print(f"-- Go to {self.target[0]}m North, {self.target[1]}m East, {self.target[2]}m Down within local coordinate system")
				await drone.conn.offboard.set_position_ned(PositionNedYaw(*start_position, start_height, 0.0))

				break