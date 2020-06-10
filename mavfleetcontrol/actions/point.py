from mavcontrol.craft import Craft
from mavsdk import System
from mavsdk import (OffboardError,Attitude, PositionNedYaw)
import numpy as np

def distance_between(p1,p2):
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return np.sqrt(squared_dist)

class FlyToPoint:

	def __init__(self, point: np.array, tolerance: float = 1.0):
		self.target = point
		self.tolerance = tolerance
		# self.fleet = []
		# self.master = None

	async def __call__(self, drone):


		print("-- Starting offboard")
		try:
			await drone.conn.offboard.start()
		except OffboardError as error:
			print(f"Starting offboard mode failed with error code: {error._result.result}")
			print("-- Disarming")
			await drone.conn.action.disarm()
			return

		print("-- Go 0m North, 0m East, -10m Down within local coordinate system")
		await drone.conn.offboard.set_position_ned(PositionNedYaw(*self.target, 0.0))

		async for position_ned in drone.conn.telemetry.position_velocity_ned():
			position = np.array([position_ned.position.north_m,position_ned.position.east_m,position_ned.position.down_m])
			if distance_between(self.target, position) < self.tolerance:
				print(f"-- Arrived @ {self.target[0]}m North, {self.target[1]}m East, {self.target[2]}m Down within local coordinate system")
				break

