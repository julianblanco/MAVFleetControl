
if __name__ == "__main__":

	# loop = asyncio.get_event_loop()


	dronenames=[]
	droneports=[]
	dronehandles=[]

	for x in range(0,5):
		dronenames.append('drone'+str(x))
		droneports.append('udp://:1454'+str(x))

	for x in range(len(dronenames)):
		drone = Craft(dronenames[x],droneports[x])
		dronehandles.append(drone)

	for drone in dronehandles:
		drone.start()
		drone.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
		drone.add_action(Flip(tolerance = 1))
		drone.add_action(FlyToPoint(np.array([0,0,-15]),tolerance =1))
		drone.add_action(Flip(tolerance = 1))
		drone.add_action(land)
		# drone.override_action(land)

	for drone in dronehandles:
		drone.close_conn()#will run after FLYTOPOINT IS DONE)
		drone.join()

