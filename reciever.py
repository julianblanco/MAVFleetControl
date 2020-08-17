`import asyncio
import serial_asyncio
import time
import numpy as np

from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.point import FlyToPoint
from mavfleetcontrol.actions.percision_land import PercisionLand
from mavfleetcontrol.actions.arm import Arm
from mavfleetcontrol.actions.disarm import Disarm
from mavfleetcontrol.actions.land import land
from mavfleetcontrol.actions.circle import Circle




class Output(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        transport.serial.rts = False
        transport.write(b'hello world\n')


    def data_received(self, msg):
        # print('data received', repr(msg))
        try:
            if(msg==b'.'):
            # heartbeat(drone)
                async for pos_vel in drone.conn.telemetry.position_velocity_ned():
                    s[0] = pos_vel.position.north_m
                    s[1] = pos_vel.position.east_m
                    s[2] = -pos_vel.position.down_m
                    s[3] = pos_vel.velocity.north_m_s
                    s[4] = pos_vel.velocity.east_m_s
                    s[5] = -pos_vel.velocity.down_m_s     
                
            if(((msg)[:2])==(b'OA')):
            # print((msg)[2])
                if(((msg)[2])==49):#1
                    drone.add_action(Arm())
                    print('arm')
                if(((msg)[2])==50):#2
                    drone.add_action(Disarm())
                    print('disarm')
                if(((msg)[2])==51):#3
                    drone.add_action(FlyToPoint(np.array([0, 0, -1]), tolerance=2.5))
                    print('ftp0,0,-1')
                if(((msg)[2])==52):#4
                    drone.add_action(land())
                    print('land')
                if(((msg)[2])==53):#5
                    drone.add_action( PercisionLand( 1.0,   np.array([1, 1])   )  )
                    print('percision_land')
                if(((msg)[2])==54):#6
                    drone.add_action(FlyToPoint(np.array([0,0,-10]),tolerance =1))
                    print('FlyToPoint0,0,-10')
                if(((msg)[2])==55):#7
                    drone.add_action(Circle(velocity=2.0,radius=8.0,angle=0.0))
                    print('circle')
                if(((msg)[2])==56):#8
                    drone.add_action(Spin())
                    print('Spin')
                if(((msg)[2])==57):#9
                    drone.override_action(Killing())
                    print('Killing!')
        except Exception as E:
            print(E)
    def connection_lost(self, exc):
        print('port closed')
        asyncio.get_event_loop().stop()

# drone = Craft("drone1","serial:///dev/serial0:1000000")
drone = Craft('drone0',"udp://:14540")
drone.start()
loop = asyncio.get_event_loop()
# coro = serial_asyncio.create_serial_connection(loop, Output, '/dev/ttyUSB0', baudrate=57600)
coro = serial_asyncio.create_serial_connection(loop, Output, '/dev/serial0', baudrate=1000000)
loop.run_until_complete(coro)
loop.run_forever()
loop.close()