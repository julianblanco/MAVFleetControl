
import asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.percision_land import PercisionLand
from mavfleetcontrol.actions.arm import Arm
from mavfleetcontrol.actions.disarm import Disarm
from mavfleetcontrol.actions.land import land
from mavfleetcontrol.actions.circle import Circle

import zmq
from zmq.asyncio import Context, Poller
"""Reciever to be run on the companion computer of the drone
using zmq with asyncio with pub/sub and dealer/router"""

url = 'tcp://127.0.0.1:5555'
url2 = 'tcp://127.0.0.1:5556'
ctx = Context.instance()

async def heartbeat(drone):
    while True:
        await asyncio.sleep(1)
    # drone.override_action(land)

async def receiver(drone):
    """receive messages with polling"""
    pull = ctx.socket(zmq.PULL)
    pull.connect(url2)
    poller = Poller()
    poller.register(pull, zmq.POLLIN)
    while True:

            try:
                events = await poller.poll()
                if pull in dict(events):
                    # print("recving", events)
                    msg = await pull.recv_multipart()
                    # print((msg[0]))
                    # print(msg.type)
                    if(msg[0]==b'.'):
                        # heartbeat(drone)
                        pass
                    if(((msg[0])[:2])==(b'OA')):
                        # print((msg[0])[2])
                        if(((msg[0])[2])==49):
                            drone.add_action(Arm())
                        if(((msg[0])[2])==50):
                            drone.add_action(Disarm())
                        if(((msg[0])[2])==51):
                            drone.add_action(FlyToPoint(np.array([0, 0, -1]), tolerance=2.5))
                        if(((msg[0])[2])==52):
                            drone.add_action(land())
                        if(((msg[0])[2])==53):
                            drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
                        if(((msg[0])[2])==54):
                            drone.add_action(FlyToPoint(np.array([0,0,-10]),tolerance =1))
                        if(((msg[0])[2])==55):
                            drone.add_action(Circle(velocity=20.0,radius=8.0,angle=0.0))
                        if(((msg[0])[2])==57):
                            drone.override_action(Spin())
                        if(((msg[0])[2])==57):
                            drone.override_action(Killing())
            except Exception as E:
                print(E)





#----------------------------------------------------------------------
if __name__ == "__main__":
    
    # drone = Craft("drone1","serial:///dev/serial0:1000000")
    drone = Craft("drone1", "udp://:14540")
    drone.start()
    asyncio.ensure_future(receiver(drone))
    asyncio.get_event_loop().run_forever()
