import asyncio
async def land(drone):
	await drone.conn.action.land()
	await asyncio.sleep(10)
