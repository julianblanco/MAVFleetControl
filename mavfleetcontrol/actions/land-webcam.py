from mavfleetcontrol.craft import Craft
from mavfleetcontrol.actions.land import land
from mavsdk import System
from mavsdk.offboard import (OffboardError,Attitude,VelocityNedYaw, PositionNedYaw)
import numpy as np
import asyncio
import cv2
import numpy as np
cam = cv2.VideoCapture(0)
helipad_src_1 = cv2.imread('h.png',0)
helipad_src_2 = cv2.imread('middle-h.png',0)
sift = cv2.xfeatures2d.SIFT_create()


def surf(image1,image2):
    sift = cv2.xfeatures2d.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(image1,None)
    kp2, des2 = sift.detectAndCompute(image2,None)
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.6*n.distance:
            good.append(m)
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        middle = cv2.moments(dst)
        cX = int(middle["m10"] / middle["m00"])
        cY = int(middle["m01"] / middle["m00"])

        
        # img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        matchesMask = None
        cX = -1
        cY = -1
    return cX , cY
def distance_between(p1,p2):
	squared_dist = np.sum((p1-p2)**2, axis=0)
	return np.sqrt(squared_dist)
def saturate(lower,upper,value):
	if(value>upper):
		value = upper
	if(value<lower):
		value = lower
	return value
class PercisionLand:
	def __init__(self, velocity: float ,positions: np.array, tolerance: float = 1.0, landspeed = 0.5):
		self.target = velocity
		self.tolerance = tolerance
		self.positions = positions
		self.landspeed = landspeed
		self.slowHeight = -2
		# self.fleet = []
		# self.master = None

	async def __call__(self, drone):
		# print(drone.conn.telemetry.armed)

		await drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0])
		await drone.start_offboard()
		


		# await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(*self.target, 0.0))
		
		flag = True
		#implement a crude cross track controller
		print(f"-- Landing at {self.positions[0]}m North, {self.positions[1]}m East")
		
		async for position_ned in drone.conn.telemetry.position_velocity_ned():
			ret, image = cam.read()
			small_to_large_image_size_ratio = 0.5
	        # small_img= cv2.resize(img1,(0,0),fx=small_to_large_image_size_ratio, fy=small_to_large_image_size_ratio, interpolation=cv2.INTER_NEAREST)
	        small_frame= cv2.resize(frame,(0,0),fx=small_to_large_image_size_ratio, fy=small_to_large_image_size_ratio, interpolation=cv2.INTER_NEAREST)
	        x,y = surf(img1,small_frame)




			currentposn = np.array([position_ned.position.north_m,position_ned.position.east_m, position_ned.position.down_m])

			xerror =  self.positions[0] - currentposn[0] 
			yerror = self.positions[1] - currentposn[1]

			xvelocity = xerror *2
			yvelocity = yerror *2
			zvelocity = -0.5 *currentposn[2]
			if(currentposn[2]>self.slowHeight):
				zvelocity = -0.1 *currentposn[2]

			await drone.conn.offboard.set_velocity_ned(VelocityNedYaw(xvelocity,yvelocity,zvelocity,0.0))
			await asyncio.sleep(0.01)

			if(currentposn[2]>-1):
				 # await drone.conn.action.land()
				 break




			cam.release()