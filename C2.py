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
from mavfleetcontrol.actions.disarm import Disarming
from mavfleetcontrol.actions.arm import Arming
from mavfleetcontrol.actions.kill import Killing
import asyncio
import numpy as np
import zmq
from zmq.asyncio import Context, Poller

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
# directory  = "/home/jules/MAVFleetControl"
# scripts = os.listdir(directory)

# valid_scripts = []
# for script in scripts:
# 	if script.endswith('py'):
# 		valid_scripts.append(script)
# print('Availible Options:')
# for x in range(len(valid_scripts)):
# 	print(str(x)+'. ' + (valid_scripts[x])[:-3])

loop = True
remote = True
url = 'tcp://127.0.0.1:5555'
url2 = 'tcp://127.0.0.1:5556'
ctx = Context.instance()
push = ctx.socket(zmq.PUSH)
push.bind(url2)
#-------------------------------------------------------------------

async def receiver():
	"""receive messages with polling"""
	pull = ctx.socket(zmq.PULL)
	pull.connect(url)
	poller = Poller()
	poller.register(pull, zmq.POLLIN)
	while True:
		events = await poller.poll()
		if pull in dict(events):
			# print("recving", events)
			msg = await pull.recv_multipart()
			# print('recvd', msg)


async def heartbeat():
	"""send a message every second"""
	# push = ctx.socket(zmq.PUSH)
	# push.bind(url2)
	
	msg = '.'
	while True:
		# print("sending 0")
		# await push.send_multipart([msg.encode('ascii')])
		await push.send_string(msg)
		await asyncio.sleep(1)
def send_message(sendmsg):
	"""send a message every second"""
	# push = ctx.socket(zmq.PUSH)
	# push.bind(url2)
	
	msg = 'OA'+sendmsg
	# await push.send_multipart([msg.encode('ascii')])
	push.send_string(msg)

def print_main_page():
	os.system('clear')
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
		print("5: Precision Land [1,1,0]NED")
		print("6: Go to [0,0,10] NED")
		print("7: Circle")
		print("9: Emergency Cut Off")
		print("0: Back to Main")
		print()
		if drone == 'remote':
			remote = 0
		else:
			remote = 1
		# session = PromptSession()
		with patch_stdout():
			text = await session.prompt_async('actions> ')
		# text = await prompt_async('actions> ', key_bindings=bindings)
		if text == '1':
			if remote:
				drone.add_action(Arming())
			else:
				send_message('1')
		if text == '2':
			if remote:
				drone.add_action(Disarming())
			else:
				send_message('2')
		if text == '3':
			if remote:
				drone.add_action(FlyToPoint(np.array([0,0,-1]),tolerance =1))
			else:
				send_message('3')

		if text == '4':
			if remote:
				dront.add_action(land)
			else:
				send_message('4')
		if text == '5':
			if remote:
				drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
			else:
				send_message('5')
		if text == '6':
			if remote:
				drone.add_action(FlyToPoint(np.array([0,0,-10]),tolerance =1))
			else:
				send_message('6')
		if text == '9':
			if remote:
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

	drones = 'remote'
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


		# drones.override_action('exit')
#----------------------------------------------------------------------
if __name__ == "__main__":
	# Start the main function
	asyncio.ensure_future(receiver())
	asyncio.ensure_future(heartbeat())
	asyncio.ensure_future(prompt())

	# Runs the event loop until the program is canceled with e.g. CTRL-C
	asyncio.get_event_loop().run_forever()
	# asyncio.get_event_loop().run_until_complete(asyncio.wait([receiver(), sender(0),]))
	