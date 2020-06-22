import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.land import land
from mavfleetcontrol.actions.arm import Arming

if __name__ == "__main__":



	drone1 = Craft("drone1", "udp://:14540")

	drone1.start()
	drone1.add_action(Arming())
	# drone1.add_action(FlyToPoint(np.array([0,0,-2]),tolerance = 0.25))

	# drone1.add_action(land)


	# drone.override_action(land)


	drone1.close_conn()#will run after FLYTOPOINT IS DONE)

	drone1.join()


