from mavfleetcontrol.craft import Craft
from mavsdk import System
import numpy as np
import asyncio

# from mavsdk.offboard import (OffboardError,Attitude, PositionNedYaw)

class Killing:

	# def __init__(self, point: np.array, tolerance: float = 1.0):
	# 	self.target = point
	# 	self.tolerance = tolerance


	async def __call__(self, drone):
		# print(drone.conn.telemetry.armed)

		await drone.kill()
		print("-- Drone Killed")