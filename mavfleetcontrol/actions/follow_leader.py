from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, VelocityNedYaw,PositionNedYaw)
import numpy as np
import asyncio
# def cart2pol(x, y):
# 	rho = np.sqrt(x**2 + y**2)
# 	phi = np.arctan2(y, x)
# 	return(rho, phi)

# def pol2cart(rho, phi):
# 	x = rho * np.cos(phi)
# 	y = rho * np.sin(phi)
# 	return(x, y)

# def angular_diff(p1,p2):
# 	diff = p2 - p1
# 	if diff > 180:
# 		diff = diff -360
# 	else:
# 		if diff < -180:
# 			diff = diff +360
# 	# print(f"{diff}diff")
# 	return diff
# def saturate(lower,upper,value):
# 	if(value>upper):
# 		value = upper
# 	if(value<lower):
# 		value = lower
# 	return value
# def distance_between(p1,p2):
# 	squared_dist = np.sum((p1-p2)**2, axis=0)
# 	return np.sqrt(squared_dist)

class Follow:

	def __init__(self, leaderdrone,distance: float =1.0 ,angle: float = 0.0,tolerance: int =2 ):
		self.distance = distance
		self.angle = angle
		self.tolerance = tolerance
		self.leaderdrone = leaderdrone



	async def __call__(self,drone):

		await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])
		await drone.start_offboard()

	
		async for position_ned in self.leaderdrone.conn.telemetry.position_velocity_ned():
			position = np.array([position_ned.position.north_m,position_ned.position.east_m])#,position_ned.position.down_m])
			follow_point = np.array([position[0] - self.radius,start_position[1]])
			circle_otherside = np.array([start_position[0] + 2*self.radius,start_position[1]])
			await drone.conn.offboard.set_position_ned(PositionNedYaw(*follow_point, 0.0))