#!/usr/bin/env python3

"""
Bla
"""
from typing import Callable, Awaitable, List
import threading
import functools

import asyncio

from mavsdk import System
from mavsdk import (OffboardError,Attitude, PositionNedYaw, PositionNed)
import numpy as np

class Craft(threading.Thread):

	def __init__ (self, connection_address: str, action: Callable[["Craft"], Awaitable[None]] = None):
		super().__init__()
		self.conn:System=None
		self.address: str = connection_address
		self.action: Callable[["Craft"], Awaitable[None]] = action
		self.loop = None

	def run(self):
		self.loop = asyncio.new_event_loop()
		try:
			self.loop.run_until_complete(self.connect())
			self.loop.run_forever()
		finally:
			self.loop.run_until_complete(self.loop.shutdown_asyncgens())
			self.loop.close()
	# def close_conn(self):

	def add_action(self, action):
		self.loop.call_soon_threadsafe(action,self)

	def close_conn(self):
		self.loop.stop()

	async def arm(self, coordinate:List[float] = None, attitude:List[float] = None):
		try:
			await self.conn.action.arm()
			# print("-- Setting initial setpoint")
			if coordinate is not None:
				await self.conn.offboard.set_position_ned(PositionNedYaw(*coordinate, 0.0))
			if attitude is not None:
				await self.conn.offboard.set_attitude(Attitude(*attitude, 0.0))

		except:# add exception later
			print('Exception Already Armed')

	async def connect(self):	
		self.conn = System()
		await self.conn.connect(system_address=self.address)

		async for state in self.conn.core.connection_state():
			if state.is_connected:
				break

	@property
	async def current_position(self) -> PositionNed:
		async for position_ned in self.conn.telemetry.position_velocity_ned():
			return np.array([position_ned.position.north_m,position_ned.position.east_m,position_ned.position.down_m])
	
	