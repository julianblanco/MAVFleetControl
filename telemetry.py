from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.sensor import Sensor
# drone = Craft('drone1',"serial:///dev/ttyUSB0:57600")
drone = Craft("drone1", "udp://:14540")

drone.start()
drone.add_action(Sensor())
drone.close_conn()#will run after FLYTOPOINT IS DONE)
drone.join()
