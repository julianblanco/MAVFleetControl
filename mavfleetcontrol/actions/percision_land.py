from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude,VelocityNedYaw, PositionNedYaw)
import numpy as np
import asyncio
def distance_between(p1,p2):
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return np.sqrt(squared_dist)

class PercisionLand:
    def __init__(self, velocity: float ,positions: np.array, tolerance: float = 1.0, landspeed = 0.5):
        self.target = velocity
        self.tolerance = tolerance
        self.positions = positions
        self.landspeed = landspeed
        # self.fleet = []
        # self.master = None

    async def __call__(self, drone):
        # print(drone.conn.telemetry.armed)

        await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])
        await drone.start_offboard()
        

        await drone.register_sensor("imu", drone.conn.telemetry.imu())
        await drone.register_sensor("ned", drone.conn.telemetry.position_velocity_ned())
        while drone.imu is None or drone.ned is None:
            await asyncio.sleep(0)

        # await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(*self.target, 0.0))
        
        flag = True
        #implement a crude cross track controller
        print(f"-- Landing at {self.positions[0]}m North, {self.positions[1]}m East, {self.positions[2]}m Down within local coordinate system")
        
        while flag:
            currentposn = np.array([drone.ned.position.north_m,drone.ned.position.east_m,drone.ned.position.down_m])

            xerror =  self.positions[0] - currentposn[0] 
            yerror = self.positions[1] - currentposn[1]

            xvelocity = xerror *2
            yvelocity = yerror *2
            zvelocity = -0.5 *currentposn[2]
            await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,zvelocity,0.0))
            await asyncio.sleep(0.01)

            if currentposn[2]> -0.2:
            	flag =False

        