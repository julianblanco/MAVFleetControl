
import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import (FlyToPoint)

if __name__ == "__main__":

	# loop = asyncio.get_event_loop()


	drone = Craft("udp://:14540")
	# loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
	drone.start()
	drone.add_action(FlyToPoint(np.array([0,0,-20])))
	time.sleep(2)

	drone.override_action(FlyToPoint(np.array([10,10,-5])))
	drone.close_conn()#will run after FLYTOPOINT IS DONE)
	drone.join()


