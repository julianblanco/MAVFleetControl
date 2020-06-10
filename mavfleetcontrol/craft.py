

"""
Bla
"""
from typing import Callable, Awaitable, List
import threading
import functools
import queue
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
		# self.loop = None
		self.loop = asyncio.new_event_loop()
		self.tasking = queue.Queue()
		self.current_task = None
		self.current_task_lock = threading.Lock()

	def run(self):
		# self.loop = asyncio.new_event_loop()
		# print("run loo[")
		try:
			self.loop.run_until_complete(self.connect())
			while True:
				action = self.tasking.get()
				if isinstance(action,str) and action == 'exit':
					break
				# self.loop.run_until_complete(action(self))
				with self.current_task_lock:
					self.current_task = self.loop.create_task(action(self))
				try:
					self.loop.run_until_complete(self.current_task)
				except asyncio.CancelledError:
					print('Command Cancelled')

				with self.current_task_lock:
					self.current_task = None
				#Blocking call will continously try and get a task
				#if no tasks, aircraft will default to pix4 default maybe?


		finally:
			self.loop.run_until_complete(self.loop.shutdown_asyncgens())
			self.loop.close()
	# def close_conn(self):

	def add_action(self, action):
		self.tasking.put(action)
		# asyncio.run_coroutine_threadsafe(action(self),self.loop)
		# self.loop.call_soon_threadsafe(action,self)

	def override_action(self,action):

		with self.current_task_lock:
			self.tasking.queue.clear()
			#maybe add check to see if safe
			self.tasking.put(action)

			if self.current_task is not None:
				self.current_task.cancel()
		
	def close_conn(self):
		self.tasking.put('exit')
		# self.loop.stop()

	async def arm(self, coordinate:List[float] = None, attitude:List[float] = None):
		try:
			await self.conn.action.arm()
			print("-- Setting initial setpoint")
			if coordinate is not None:
				await self.conn.offboard.set_position_ned(PositionNedYaw(*coordinate, 0.0))
			if attitude is not None:
				await self.conn.offboard.set_attitude(Attitude(*attitude, 0.0))

		except Exception as bla:# add exception later
			print(bla)

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
	
	