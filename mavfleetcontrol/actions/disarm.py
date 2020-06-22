from mavfleetcontrol.craft import Craft
from mavsdk import System
# from mavsdk.offboard import (OffboardError,Attitude, PositionNedYaw)

class disarm:

	def __init__(self, point: np.array, tolerance: float = 1.0):
		self.target = point
		self.tolerance = tolerance
		# self.fleet = []
		# self.master = None

	async def __call__(self, drone):
		# print(drone.conn.telemetry.armed)

		await drone.disarm()
		print("-- Disarm")
