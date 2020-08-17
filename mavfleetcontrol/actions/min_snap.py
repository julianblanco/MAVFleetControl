# Implements a minimum snap trajectory generator coupled with a hover envelope position controller to generate combined
# thrust and body rate setpoints
#
# By: Patrick Ledzian
# Date: 17Jul20
#

from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude, AttitudeRate, VelocityNedYaw,PositionNedYaw)
import numpy as np
import asyncio
import time
from mavfleetcontrol.actions.quadcopter import Quadcopter


class MinSnap(Quadcopter):

    def __init__(self):
        super(MinSnap, self).__init__()

    async def __call__(self, drone):
        await drone.arm(coordinate=[0.0, 0.0, 0.0], attitude=[0.0, 0.0, 0.0])
        await drone.start_offboard()
        print("-- Starting Minimum Snap Test")

        # initialize the vehicle state
        loop = 1
        init_arr = np.zeros(6)
        async for pos_vel in drone.conn.telemetry.position_velocity_ned():
            init_arr[0] = pos_vel.position.north_m
            init_arr[1] = -pos_vel.position.east_m
            init_arr[2] = -pos_vel.position.down_m
            break
        async for quat in drone.conn.telemetry.attitude_quaternion():
                # loop = 2
            init_arr[3] = quat.x
            init_arr[4] = quat.y
            init_arr[5] = quat.z
            break

        self._state = self.set_initial_state(init_arr)
        self._desired_state = self.set_desired_state([0.0, 0.0, 5.0])
        self._goal = np.array(self._desired_state)

        # main loop; queries sensors, runs min snap trajectory generation and position controller
        s = np.zeros(13)
        start_time = time.time()
        cnt = 0
        while(1):
            # print("1")
            async for pos_vel in drone.conn.telemetry.position_velocity_ned():

                s[0] = pos_vel.position.north_m
                s[1] = -pos_vel.position.east_m
                s[2] = -pos_vel.position.down_m
                s[3] = pos_vel.velocity.north_m_s
                s[4] = -pos_vel.velocity.east_m_s
                s[5] = -pos_vel.velocity.down_m_s           # TODO: should this be negative? velocity encodes direction...
                break
            asyncio.sleep(0.01)
            # # # print("loop 2")
            async for quat in drone.conn.telemetry.attitude_quaternion():
                s[6] = quat.w
                s[7] = quat.x
                s[8] = quat.y
                s[9] = quat.z
                break
            asyncio.sleep(0.01)
            # # print("loop 3")
            # async for rate in drone.conn.telemetry.attitude_angular_velocity_body():
            #     # loop = 1
            #     s[10] = rate.roll_rad_s
            #     s[11] = rate.pitch_rad_s
            #     s[12] = rate.yaw_rad_s
            #         # print("loop 3")
            #     break
            asyncio.sleep(0.01)
            # print(s)
            self._state = s
            assert str(np.shape(self._state)) == "(13,)", "Incorrect state vector size in simulation_step: {}".format(np.shape(self._state))
            assert str(np.shape(self._desired_state)) == "(11,)", "Incorrect desired state vector size in simulation_step: {}".format(np.shape(self._desired_state))
            assert str(np.shape(self._goal)) == "(11,)", "Incorrect goal vector size in simulation_step: {}".format(np.shape(self._goal))

            t = time.time() - start_time
            new_des_state = self.minimun_snap_trajectory(self._state[0:3], t)												# minimum snap trajecotry with default path
            # new_des_state = self.simple_line_trajectory(self._state[0:3], self._goal[0:3], 10, t)				# straight line
            self._desired_state[0:9] = new_des_state
            thrust, moment = self.pid_controller()
            # print("state: ", self._state)
            # print("desired state: ", self._desired_state)

            # print("pre-clip thrust: ", thrust)
            # print("pre-clip moment: ", moment)

            thrust = thrust / 60.0                      # normalize the thrust value; what is the max? tried: 24
            moment = moment * (180/np.pi)
            print("clipped thrust: ", thrust)
            print("desired att: ", moment)
            asyncio.sleep(0.01)
            await drone.conn.offboard.set_attitude(Attitude(moment[0], moment[1], moment[2], thrust))

            # await drone.conn.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.5))
            asyncio.sleep(0.01)
            # cnt +=1
            # if cnt > 6:
            #     break
