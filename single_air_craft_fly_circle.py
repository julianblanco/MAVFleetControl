
import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.circle import Circle
from mavfleetcontrol.actions.percision_land import PercisionLand
from mavfleetcontrol.actions.land import land

if __name__ == "__main__":

	# loop = asyncio.get_event_loop()


	drone = Craft('drone1',"udp://:14540")
	# loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
	drone.start()
	drone.add_action(FlyToPoint(np.array([0,0,-5]),tolerance =1))
	drone.add_action(Circle(velocity=5.0,radius=8.0,angle=0.0))
	drone.add_action(FlyToPoint(np.array([0,0,-5]),tolerance =1))
	drone.add_action( PercisionLand( 1.0,   np.array([0, 0])   )  )
	drone.add_action(land)
	# drone.override_action(land)
	drone.close_conn()#will run after FLYTOPOINT IS DONE)
	drone.join()


