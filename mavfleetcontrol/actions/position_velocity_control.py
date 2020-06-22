from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude,VelocityNedYaw, PositionNedYaw)
import numpy as np
import asyncio

def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return ang1-ang2
def distance_between(p1,p2):
    squared_dist = np.sum((p1-p2)**2, axis=0)
    return np.sqrt(squared_dist)
class PositionVelocityControl:

    def __init__(self, velocity: float ,positions: np.array, tolerance: float = 1.0):
        self.target = velocity
        self.tolerance = tolerance
        self.positions = positions
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

        #implement a crude cross track controller
        while True:
            currentposn = np.array([drone.ned.position.north_m,drone.ned.position.east_m])
            trackangle = np.arctan2(*(self.positions[1]-self.positions[0])[::-1])
            angle_to_target = np.arctan2(*(self.positions[1]-currentposn)[::-1])
            delta_heading = trackangle - angle_to_target
            distance2WP = np.linalg.norm(self.positions[1]-currentposn)
            crosstrackerror = distance2WP * np.sin(delta_heading)
            # crosstrackcorrection = crosstrackerror * 2.0
            # projectedremainingtrack = (2.0/3.0)*distance2WP*cos(delta_heading)
            newheading =   angle_to_target - crosstrackerror * (3.14159/180)  
            xvelocity = np.cos(newheading)*self.target
            yvelocity = np.sin(newheading)*self.target
            # print(f'trackangle: {trackangle}')
            # print(f'angle_to_target: {angle_to_target}')
            # print(f'Current POSN: {currentposn}')
            # print(f'Xvelocity: {xvelocity}')
            # print(f'Yvelocity: {yvelocity}')
            await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,0.0,0.0))
            await asyncio.sleep(1)
        
