import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.land import land
from mavfleetcontrol.actions.follow_leader import Follow

if __name__ == "__main__":



	drone1 = Craft("drone1", "udp://:14540")
	drone1.start()
	with serial.Serial('/dev/ttyUSB0', baudrate=57600, timeout=1) as ser:
		while(1):
			line = ser.readline().decode('ascii', errors='replace')
			data = line.split(',')
			drone1.add_action(FlyToPoint(np.array([0,0,-2]),tolerance = 0.25))



	drone1.add_action(land)


	# drone.override_action(land)


	drone1.close_conn()#will run after FLYTOPOINT IS DONE)
	drone2.close_conn()#will run after FLYTOPOINT IS DONE)

	drone1.join()
	drone2.join()



