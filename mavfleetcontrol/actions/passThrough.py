from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, PositionNedYaw, ActuatorControl)
import numpy as np
import asyncio


class ManualPass:

	# def __init__(self, start_angle: float = 0, tolerance: float = 10.0):
	# 	self.start_angle = start_angle
	# 	self.tolerance = tolerance
	# 	# self.fleet = []
	# 	# self.master = None

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

		print("-- Starting ManualPass")
	
		#set aircraft to flip at 300 deg/s to the right (roll)

		await drone.conn.offboard.set_actuator_control(ActuatorControl(({1.0,1.0,1.0,1.0})))

		# #when aircraft is upside down (break loop)
		# normalized_desired_angle =-179
		# async for angle in drone.conn.telemetry.attitude_euler():
