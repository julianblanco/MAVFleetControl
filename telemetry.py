from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.accel import Accel

drone = Craft('drone1',"udp://:14540")
drone.start()
drone.add_action(Sensor())
print(drone.imu.acceleration_frd)
drone.close_conn()#will run after FLYTOPOINT IS DONE)
drone.join()