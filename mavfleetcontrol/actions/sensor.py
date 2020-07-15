from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import OffboardError, Attitude, AttitudeRate, PositionNedYaw
import numpy as np
import asyncio


class Sensor:
	# def __init__(self):
	async def __call__(self, drone):

		# schedule to get
		# acceleration_frd (AccelerationFrd) – Acceleration
		# angular_velocity_frd (AngularVelocityFrd) – Angular velocity
		# magnetic_field_frd (MagneticFieldFrd) – Magnetic field
		# temperature_degc (float) – Temperature

		await drone.register_sensor("imu", drone.conn.telemetry.imu())
		await drone.register_sensor("ned", drone.conn.telemetry.position_velocity_ned())
		while drone.imu is None or drone.ned is None:
			print('Not')
			await asyncio.sleep(0)
		# set aircraft to flip at 300 deg/s to the right (roll)

		while True:
			if drone.imu is not None:	
				print(drone.imu.acceleration_frd)
			else:
				print("i tried :(")
			await asyncio.sleep(0)
