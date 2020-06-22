from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, PositionNedYaw)
import numpy as np
import asyncio

def angular_diff(p1,p2):
	diff = p2 - p1
	if diff > 180:
		diff = diff -360
	else:
		if diff < -180:
			diff = diff +360
	# print(f"{diff}diff")
	return diff
class Flip:

	def __init__(self, start_angle: float = 0, tolerance: float = 10.0):
		self.start_angle = start_angle
		self.tolerance = tolerance
		# self.fleet = []
		# self.master = None

	async def __call__(self, drone):

		await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])

		print("-- Starting offboard")
		try:
			await drone.conn.offboard.start()
		except OffboardError as error:
			print(f"Starting offboard mode failed with error code: {error._result.result}")
			print("-- Disarming")
			await drone.conn.action.disarm()
			return

		print("-- Starting Flip")
	
		#set aircraft to flip at 300 deg/s to the right (roll)

		await drone.conn.offboard.set_attitude_rate(AttitudeRate(300.0, 0.0, 0.0, 0.5))

		#when aircraft is upside down (break loop)
		normalized_desired_angle =-179
		async for angle in drone.conn.telemetry.attitude_euler():
			euler = np.array([angle.roll_deg,angle.pitch_deg,angle.yaw_deg])
			diff =  angular_diff(normalized_desired_angle, euler[0])
			# print(f"Diff{diff}")
			if abs(diff) < self.tolerance:
				print(f"-- Arrived @ {normalized_desired_angle}")
				print(euler[0])
				break
