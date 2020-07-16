from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.land import land
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude,VelocityNedYaw, PositionNedYaw)
import numpy as np
import asyncio
def distance_between(p1,p2):
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return np.sqrt(squared_dist)
def saturate(lower,upper,value):
	if(value>upper):
		value = upper
	if(value<lower):
		value = lower
	return value
class PercisionLand:
	def __init__(self, velocity: float ,positions: np.array, tolerance: float = 1.0, landspeed = 0.5):
		self.target = velocity
		self.tolerance = tolerance
		self.positions = positions
		self.landspeed = landspeed
		self.slowHeight = -2
		# self.fleet = []
		# self.master = None

	async def __call__(self, drone):
		# print(drone.conn.telemetry.armed)

		await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])
		await drone.start_offboard()
		


		# await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(*self.target, 0.0))
		
		flag = True
		#implement a crude cross track controller
		print(f"-- Landing at {self.positions[0]}m North, {self.positions[1]}m East")
		
		async for position_ned in drone.conn.telemetry.position_velocity_ned():
			currentposn = np.array([position_ned.position.north_m,position_ned.position.east_m, position_ned.position.down_m])

			xerror =  self.positions[0] - currentposn[0] 
			yerror = self.positions[1] - currentposn[1]

			xvelocity = xerror *2
			yvelocity = yerror *2
			zvelocity = -0.5 *currentposn[2]
			if(currentposn[2]>self.slowHeight):
				zvelocity = -0.1 *currentposn[2]

			await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,zvelocity,0.0))
			await asyncio.sleep(0.01)

			if(currentposn[2]>-1):
				 # await drone.conn.action.land()
				 break