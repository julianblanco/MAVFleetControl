# Test the implementation of a minimum snap trajectory generator
#
# By: Patrick Ledzian
# Date: 17Jul20
#


import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.min_snap import MinSnap
from mavfleetcontrol.actions.percision_land import PercisionLand
from mavfleetcontrol.actions.land import land

if __name__ == "__main__":
    drone = Craft('drone1',"udp://:14540")
    drone.start()
    drone.add_action(FlyToPoint(np.array([0, 0, -5]), tolerance=1))
    drone.add_action(MinSnap())
    drone.add_action(PercisionLand(1.0, np.array([0, 0])))
    drone.add_action(land)
    drone.close_conn()
    drone.join()
