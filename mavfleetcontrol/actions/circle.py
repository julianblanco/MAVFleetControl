from mavfleetcontrol.craft import Craft
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, VelocityNedYaw,PositionNedYaw)
import numpy as np
import asyncio
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def angular_diff(p1,p2):
    diff = p2 - p1
    if diff > 180:
        diff = diff -360
    else:
        if diff < -180:
            diff = diff +360
    # print(f"{diff}diff")
    return diff
class Circle:

    def __init__(self, velocity: float = 1.0, radius: float =1.0 ,angle: float = 0.0,direction:str = 'cw'):
        self.velocity = velocity
        self.radius = radius
        self.angle = angle# self.fleet = []
        if direction == 'ccw':
            self.direction = True
        else:
            self.direction = False
        # self.master = None

    async def __call__(self, drone):

        await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])

        await drone.start_offboard()
        await drone.register_sensor("imu", drone.conn.telemetry.imu())
        await drone.register_sensor("ned", drone.conn.telemetry.position_velocity_ned())
        while drone.imu is None or drone.ned is None:
            await asyncio.sleep(0)
        print("-- Starting Circle")
    
        #set aircraft to flip at 300 deg/s to the right (roll)
        start_position = np.array([drone.ned.position.north_m,drone.ned.position.east_m])
        circle_center = np.array([drone.ned.position.north_m + self.radius,drone.ned.position.east_m])


        while True:
            currentposn = np.array([drone.ned.position.north_m,drone.ned.position.east_m])
            angle_to_center = np.arctan2(*(circle_center-currentposn)[::-1])
            distance2center = np.linalg.norm(circle_center-currentposn)
            distanceController = self.radius - distance2center
            if self.direction:
                tangentangle = angle_to_center + ( np.pi / 2 ) 
            else:
                tangentangle = angle_to_center + ( np.pi / 2 ) + np.pi
            print('-------')
            print(tangentangle*180/np.pi)
            print((tangentangle+np.pi)*180/np.pi )
            if self.direction:
                newangle = tangentangle + distanceController/2.0
            else:
                newangle = tangentangle - distanceController/2.0

            xvelocity = np.cos(newangle)*self.velocity
            yvelocity = np.sin(newangle)*self.velocity
            await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,0.0,0.0))