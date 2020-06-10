import asyncio
from mavcontrol.craft import Craft

drone1 = Craft("udp://:14540")
print(drone1.conn)
