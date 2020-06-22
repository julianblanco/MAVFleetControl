from prompt_toolkit import prompt
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
#-------------------------------------------------------------------
def print_main_page():
	print('1: Connect To Aircraft')
	print('2: Actions')
	print('3: Status')
	print('4: Settings')

def connect_to_aircraft():
	text = prompt('TGCS> How many aircraft?:  ', key_bindings=bindings)
	text = prompt('TGCS> Serial (1) or IP (2)', key_bindings=bindings)
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

def actions(drone):
	print("1: Arm")
	print("2: Disarm")
	print("3: Take off")
	print("4: Precision Land [1,1,0]NED")
	print("5: Go to [0,0,10] NED")
	print("6: Circle")
	text = prompt('actions> ', key_bindings=bindings)
	if text == '1':
		drone.add_action(Arming())
	if text == '2':
		drone.add_action(Disarming())
	if text == '3':
		drone.add_action(FlyToPoint(np.array([0,0,-1]),tolerance =1))
	if text == '4':
		drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
	if text == '5':
		drone.add_action(FlyToPoint(np.array([0,0,-10]),tolerance =1))
	if text == '9':
		drone.override_action(Killing())
def status(drone):
	print('s')
	drone.add_action(FlyToPoint(np.array([0,0,-5]),tolerance =1))
	drone.add_action(land)
	# drone.close_conn()#will run after FLYTOPOINT IS DONE)


def settings():
	print('set')




@bindings.add('c-k')
def _(event):
	" Say 'hello' when `c-k` is pressed. "
	def print_hello():
		print('hello world')
	run_in_terminal(print_hello)

@bindings.add('c-c')
def _(event):
	print("\n\n\n\n")
	print("Good Bye!, may your props be intact")
	event.app.exit()
	global loop
	loop = False


#----------------------------------------------------------------------

print("Welcome!")
drones = None
while loop == True:
	print_main_page()
	text = prompt('TGCS> ', key_bindings=bindings)
	if text == '1':
		drones = connect_to_aircraft()
	if text == '2':
		actions(drones)
	if text =='3':
		status(drones)
	if text == '4':
		settings()


drones.override_action('exit')