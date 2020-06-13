
import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.circle import Circle

from mavfleetcontrol.actions.land import land

if __name__ == "__main__":

	# loop = asyncio.get_event_loop()


	drone0 = Craft('drone0',"udp://:14540")
	drone1= Craft('drone1',"udp://:14541")
	# loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
	drone0.start()
	drone1.start()
	drone0.add_action(FlyToPoint(np.array([3,0,-5]),tolerance =1))
	drone1.add_action(FlyToPoint(np.array([0,0,-5]),tolerance =1))

	drone0.add_action(Circle(velocity=3.0,radius=5.0,angle=0.0,direction='cw'))
	drone1.add_action(Circle(velocity=3.0,radius=6.0,angle=0.0,direction='ccw'))
	drone0.add_action(land)
	drone1.add_action(land)
	# drone.override_action(land)
	drone0.close_conn()#will run after FLYTOPOINT IS DONE)
	drone0.join()
		# drone.override_action(land)
	drone1.close_conn()#will run after FLYTOPOINT IS DONE)
	drone1.join()




