import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.percision_land import PercisionLand
from mavfleetcontrol.actions.land import land

if __name__ == "__main__":

    # loop = asyncio.get_event_loop()

    drone = Craft("drone1", "udp://:14540")
    # loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
    drone.start()
    drone.add_action(FlyToPoint(np.array([1, 1, -20]), tolerance=0.5))
    drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
    drone.add_action(land)
    # drone.override_action(land)
    drone.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone.join()
