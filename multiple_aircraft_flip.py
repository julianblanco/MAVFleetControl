
import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.flip import Flip

from mavfleetcontrol.actions.land import land

if __name__ == "__main__":

	# loop = asyncio.get_event_loop()


	drone0 = Craft('drone0',"udp://:14540")
	drone1 = Craft('drone1',"udp://:14541")
	drone2 = Craft('drone1',"udp://:14542")
	drone3 = Craft('drone1',"udp://:14543")
	drone4 = Craft('drone1',"udp://:14544")
	# loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
	drone0.start()
	drone1.start()
	drone2.start()
	drone3.start()
	drone4.start()

	drone0.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
	drone1.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
	drone2.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
	drone3.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
	drone4.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))		

	drone0.add_action(Flip(tolerance = 10))
	drone1.add_action(Flip(tolerance = 10))
	drone2.add_action(Flip(tolerance = 10))
	drone3.add_action(Flip(tolerance = 10))
	drone4.add_action(Flip(tolerance = 10))


	drone0.add_action(land)
	drone1.add_action(land)
	drone2.add_action(land)
	drone3.add_action(land)
	drone4.add_action(land)			
	# drone.override_action(land)
	drone0.close_conn()#will run after FLYTOPOINT IS DONE)
	drone1.close_conn()#will run after FLYTOPOINT IS DONE)
	drone2.close_conn()#will run after FLYTOPOINT IS DONE)
	drone3.close_conn()#will run after FLYTOPOINT IS DONE)
	drone4.close_conn()#will run after FLYTOPOINT IS DONE)

	drone0.join()
	drone1.join()
	drone2.join()
	drone3.join()
	drone4.join()

