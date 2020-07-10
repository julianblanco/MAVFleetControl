from mavfleetcontrol.craft import Craft
from mavsdk import System
import numpy as np
# from mavsdk.offboard import (OffboardError,Attitude, PositionNedYaw)

class Disarm:


	async def __call__(self, drone):
		# print(drone.conn.telemetry.armed)

		await drone.disarm()
		print("-- Disarm")
