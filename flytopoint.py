
import asyncio

import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import (FlyToPoint)

if __name__ == "__main__":
	drone = Craft("udp://:14540")
	drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])
	drone.start()
	drone.add_action(FlyToPoint(np.array([0,0,-10])))
	drone.join()

