from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.application import run_in_terminal
from prompt_toolkit.key_binding import KeyBindings
import os
from mavfleetcontrol.craft import Craft
from pymavlink import mavutil
import sys, select
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.circle import Circle
from mavfleetcontrol.actions.land import land
from mavfleetcontrol.actions.disarm import Disarm
from mavfleetcontrol.actions.arm import Arm
from mavfleetcontrol.actions.kill import Killing
import asyncio
import numpy as np
import serial
i = 2
terminalart = r"""
___________        _________________   _________
\__    ___/       /  _____/\_   ___ \ /   _____/
  |    |  ______ /   \  ___/    \  \/ \_____  \ 
  |    | /_____/ \    \_\  \     \____/        \
  |____|erminal   \______  /\______  /_______  /
                         \/        \/        \/ """

print(terminalart)
bindings = KeyBindings()

loop = True
remote = True
ser = serial.Serial('/dev/ttyUSB0',"57600")  # open serial port
#-------------------------------------------------------------------

async def heartbeat():
	"""send a message every second"""
	# push = ctx.socket(zmq.PUSH)
	# push.bind(url2)
	
	msg = '.'
	while True:
		ser.write(msg.encode()) 
		await asyncio.sleep(1)

def send_message(sendmsg):
	"""send a message every second"""
	# push = ctx.socket(zmq.PUSH)
	# push.bind(url2)
	
	msg = 'OA'+sendmsg
	ser.write(msg.encode()) 

def print_main_page():
	
	print('1: Connect To Aircraft')
	print('2: Actions')
	print('3: Status')
	print('4: Settings')
	print()

async def connect_to_aircraft():
	session = PromptSession()
	# text =  await prompt_async('TGCS> How many aircraft?:  ', key_bindings=bindings)
	with patch_stdout():
		result = await session.prompt_async('TGCS> How many aircraft?:  ')
	# text = await prompt_async('TGCS> Serial (1) or IP (2)', key_bindings=bindings)
	with patch_stdout():
		result = await session.prompt_async('TGCS> Serial (1) or IP (2)')
	if text == '1':    
		if sys.platform == "darwin":
			serialport = "/dev/tty.usbmodem1"
		else:
			serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
				"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"])

			if len(serial_list) == 0:
				print("Error: no serial connsection found")
				return

			if len(serial_list) > 1:
				print('Auto-detected serial ports are:')
				for port in serial_list:
					print(" {:}".format(port))
			print('Using port {:}'.format(serial_list[0]))
			serialport = serial_list[0].device
		drone = Craft('drone0','serial://' + serialport +':57600')
		

	if text =='2':
		drone = Craft('drone0','udp://:14540')	
	drone.start()
	return drone

async def actions(drone,session):
	while True:
		os.system('clear')
		print("1: Arm")
		print("2: Disarm")
		print("3: Take off")
		print("4: Land")
		print("5: Precision Land [1,1,0] NED")
		print("6: Go to [0,0,10] NED")
		print("7: Circle")
		print("8: Spin")
		print("9: Emergency Cut Off")
		print("0: Back to Main")
		print()
		if drone == 'serial':
			serial = 0
		else:
			serial = 0
		# session = PromptSession()
		with patch_stdout():
			text = await session.prompt_async('actions> ')
		# text = await prompt_async('actions> ', key_bindings=bindings)
		if text == '1':
			if serial:
				drone.add_action(Arm())
			else:
				send_message('1')
		if text == '2':
			if serial:
				drone.add_action(Disarm())
			else:
				send_message('2')
		if text == '3':
			if serial:
				drone.add_action(FlyToPoint(np.array([0,0,-1]),tolerance =1))
			else:
				send_message('3')

		if text == '4':
			if serial:
				dront.add_action(land())
			else:
				send_message('4')
		if text == '5':
			if serial:
				drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
			else:
				send_message('5')
		if text == '6':
			if serial:
				drone.add_action(FlyToPoint(np.array([0,0,-10]),tolerance =1))
			else:
				send_message('6')
		if text == '7':
			if serial:
				drone.add_action(Circle(velocity=5.0,radius=8.0,angle=0.0))
				# pass
			else:
				send_message('7')
		if text == '8':
			if serial:
				drone.override_action(spin())
			else:
				send_message('8')
		if text == '9':
			if serial:
				drone.override_action(Killing())
			else:
				send_message('9')
		if text == '0':
			break

def status(drone):
	print('s')
	drone.add_action(FlyToPoint(np.array([0,0,-5]),tolerance =1))
	drone.add_action(land)
	# drone.close_conn()#will run after FLYTOPOINT IS DONE)


def settings():
	print('set')




# @bindings.add('c-k')
# async def _(event):
# 	" Say 'hello' when `c-k` is pressed. "
# 	def print_hello():
# 		print('hello world')
# 	run_in_terminal(print_hello)



async def prompt():
	# @bindings.add('c-c')
	# async def _(event):
	# 	print("\n\n\n\n")
	# 	print("Good Bye!, may your props be intact")
	# 	event.app.exit()
	global loop
	loop = True
	print("Welcome!")
	session = PromptSession()

	drones = 'serial'
	while loop == True:
		print_main_page()
		with patch_stdout():
			text = await session.prompt_async('TGCS> ')#, key_bindings=bindings)
		# text = await prompt_async('TGCS> ', key_bindings=bindings)
		if text == '1':
			drones = connect_to_aircraft()
		if text == '2':
			await actions(drones,session)
		if text =='3':
			status(drones)
		if text == '4':
			settings()
		os.system('clear')


		# drones.override_action('exit')
#----------------------------------------------------------------------
if __name__ == "__main__":
	# Start the main function
	asyncio.ensure_future(heartbeat())
	asyncio.ensure_future(prompt())

	# Runs the event loop until the program is canceled with e.g. CTRL-C
	asyncio.get_event_loop().run_forever()
	# asyncio.get_event_loop().run_until_complete(asyncio.wait([receiver(), sender(0),]))
	