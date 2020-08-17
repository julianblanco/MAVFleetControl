# Class description of a Quadcopter object
#
# By: Patrick Ledzian
# Date: 14 Apr 2020

"""
Class descriptor for a quadcopter object. Includes trajectory, controller, and dynamical equation modules. Calculates
the state update in a single method which is integrated in main.py
"""

# External Libraries
import numpy as np


# noinspection SpellCheckingInspection
class Quadcopter:
	"""
	Class descriptor for a quadcopter object
	"""
	def __init__(self, initial_state=np.zeros(6), desired_state=np.zeros(3)):
		"""
		Class constructor, define the properties inherent to a quadcopter
		:param initial_state: so the initial position and orientation (quaternion) can be set
		:param desired_state: desired position, assumes desired orientation is level
		"""
		# TODO: could make a physical properties dictionary and a quadcopter properties dictionary... maybe idk yet
		self._quad_properties = {
			"mass": 1.5,  																		# mass in kg
			"gravity": 9.81,  																		# gravitational force
			# "inertia": np.array([[0.000000975, 0.0, 0.0], [0.0, 0.000273104, 0.0], [0.0, 0.0, 0.000274004]]), 	# inertial tensor m^2 kg
			"inertia": np.array([[0.029125, 0.0, 0.0], [0.0, 0.029125, 0.0], [0.0, 0.0, 0.055225]]), 	# inertial tensor m^2 kg
			"invI": np.zeros((3, 3)),
			"length": 0.13,  																		# quadcopter arm length in meters
			"max_force": 0.0,  																		# max force allowed in Newtons
			"min_force": 0.0,
			"limitsA": np.zeros((4, 3)),
			"limitsB": np.zeros((3, 4))
		}
		self._quad_properties.update({
			"invI": np.linalg.inv(self._quad_properties["inertia"]),
			"max_force": (2.5 * self._quad_properties["mass"] * self._quad_properties["gravity"]),
			"min_force": (-0.95 * self._quad_properties["mass"] * self._quad_properties["gravity"]),
			"limitsA": np.array([
				[0.25, 0.0, -0.5 / self._quad_properties["length"]],
				[0.25, 0.5 / self._quad_properties["length"], 0.0],
				[0.25, 0.0, 0.5 / self._quad_properties["length"]],
				[0.25, -0.5 / self._quad_properties["length"], 0.0]
			]),
			"limitsB": np.array([
				[1, 1, 1, 1],
				[0.0, self._quad_properties["length"], 0.0, -self._quad_properties["length"]],
				[-self._quad_properties["length"], 0.0, self._quad_properties["length"], 0.0]
			])
		})
		self._state = Quadcopter.set_initial_state(initial_state)
		self._desired_state = Quadcopter.set_desired_state(desired_state)
		self._goal = np.array(self._desired_state) 							# necessary since by default python assigns by reference

		# minimum snap trajectory generation variables
		self.path = []
		self.total_time = 0.0
		self.ts = []
		self.coef = []
		self.minSnapSet = False
	#
	# Helper methods
	#
	@staticmethod
	def set_initial_state(s):
		# x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r
		return np.array([s[0], s[1], s[2], 0.0, 0.0, 0.0, 1.0, s[3], s[4], s[5], 0.0, 0.0, 0.0])

	@staticmethod
	def set_desired_state(s):
		# x, y, z, xd, yd, zd, xdd, ydd, zdd, yaw, yawd
		return np.array([s[0], s[1], s[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

	# TODO: encapsulte in quaternion class
	@staticmethod
	def quat2rot(quat):
		"""
		A quaternion describes some rotation in 3-D space to get to a specific position, this function converts that
		to a euler angle parameterized rotation matrix
		:param quat: 1x4 quaternion
		:return: 3x3 rotation matrix bRw
		"""
		# assert np.linalg.norm(quat) > 2e-52, "Quaternion norm too small, error"
		# if np.linalg.norm(quat) < 2e-52:
		# 	stop_sim = 1
		# 	return

		# THE NORM IS PRESERVED HERE, THE ISSUE MAY BE WITH THE SOLVER...
		q_norm = quat / np.linalg.norm(quat) 										# Frobenius / 2-norm is defaulT
		assert np.shape(q_norm) != "(4,)", "Quaternion vector dimension error"
		q_hat = np.array([
			[0, -q_norm[3], q_norm[2]],
			[q_norm[3], 0, -q_norm[1]],
			[-q_norm[2], q_norm[1], 0]
			])
		R = np.identity(3) + 2*np.matmul(q_hat, q_hat) + 2*q_norm[0]*q_hat

		return R

	@staticmethod
	def quat2euler(quat):
		"""
		Converts a quaternion to euler angles of roll, pitch, and yaw
		:param quat: 1x4 quaternion
		:return: 1x3 euler angle orientation
		"""
		assert str(np.shape(quat)) == "(4,)", "Not a valid quaterion in quat2euler"
		R = Quadcopter.quat2rot(quat)
		phi = np.arcsin(R[1, 2])
		# TODO: assert for imaginary components that can appear in arctan2, compare real to magnitue w/ error bound
		psi = np.arctan2((-R[1, 0]/np.cos(phi)), (R[1, 1]/np.cos(phi)))
		theta = np.arctan2((-R[0, 2]/np.cos(phi)), (R[2, 2]/np.cos(phi)))

		return np.array([phi, theta, psi]) 		# roll, pitch, yaw

	#
	# Behavioral methods
	#
	def simple_line_trajectory(self, cur_pos, stop_pos, finish_time, cur_time):
		"""
		Creates a trajectory by updating the desired state iteratively in the simulation. Doesn't take into account
		the vehicle dynamics when generating
		:param cur_pos: 1x3 current vehicle position
		:param stop_pos: 1x3 desired vehicle position
		:param finish_time: time allowed to finish the movement
		:param cur_time: current time during the trajectory maneuver
		:return:
		"""
		assert cur_time >= 0, "Current time is less than zero in trajectory"

		# create desired pos, vel, acc at a given point in time for a linear trajectory
		vel_max = (stop_pos - cur_pos) * 2 / finish_time
		if (cur_time >= 0.0) and (cur_time < finish_time/2):
			vel = vel_max * cur_time / (finish_time/2)  # think about having this decay as a broad explonential curve
			pos = cur_pos + (0.5) * cur_time * vel
			acc = np.zeros(3)
		elif (cur_time >= 0.0) and (cur_time > finish_time/2) and (cur_time < finish_time): 						# I think this keeps the velocity up
			vel = vel_max * (finish_time-cur_time) / (finish_time/2)  # think about having this decay as a broad explonential curve
			pos = cur_pos - (0.5) * (finish_time-cur_time) * vel
			acc = np.zeros(3)
		else:
			pos = cur_pos
			vel = np.zeros(3)
			acc = np.zeros(3)

		new_des_state = np.concatenate((pos, vel, acc))

		return new_des_state

	def hover_trajectory(self, cur_pos, cur_time):
		"""
		Creates a hover, land, hover, land trajectory by using step inputs. This does not work if the steps are large (> 1.5-ish)
		:param cur_pos: 1x3 desired vehicle position
		:param cur_time: current time during the trajectroy maneuver
		"""

		# setting a base position globally then maneuvering about that isn't friendly to odeit...
		# also we don't have fixed size solver steps so maneuvering around cur_pos[2] doesn't work well either
		if cur_time < 0.8:
			pos = np.array([cur_pos[0], cur_pos[1], 0.0])
		elif 1.0 <= cur_time < 4.0:
			pos = np.array([cur_pos[0], cur_pos[1], 0.5])
		elif 4.0 <= cur_time < 6.0:
			pos = np.array([cur_pos[0], cur_pos[1], 0.0])
		elif 6.0 <= cur_time < 8.0:
			pos = np.array([cur_pos[0], cur_pos[1], 1.5])
		else:
			return np.array([cur_pos[0], cur_pos[1], 0.0])

		vel = np.array([0.0, 0.0, 0.0])
		acc = np.array([0.0, 0.0, 0.0])
		new_des_state = np.concatenate((pos, vel, acc))

		return new_des_state

	def circle_trajectory(self, cur_time):
		pass

	def minimun_snap_trajectory(self, cur_pos, cur_time):
		if not self.minSnapSet:
			# global path
			# global total_time
			# global ts
			# global coef
			self.path = np.array([[cur_pos[0], cur_pos[0],       cur_pos[0] + 0.01, cur_pos[0] + 1.0, cur_pos[0],       cur_pos[0]],
							 [cur_pos[1], cur_pos[1],       cur_pos[1],       cur_pos[1] + 1.0, cur_pos[1] + 1.0, cur_pos[1]],
							 [cur_pos[2], cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0]
							 ])
			self.path = self.path[:, 0:3] 			# TODO: remove after troubleshooting
			# TODO: do here, "plot_path()" function for ideal path
			(self.ts, self.total_time) = self.generate_ts(self.path)						# create time expectation for each waypoint
			# print(ts)
			# print(total_time)
			# exit()
			# Coef = self.trajectory_optimization(path) 		# solve for optimal polynomial coefficients
			self.coef = self.trajectory_optimization(np.transpose(self.path)) 		# transpose the input path...
			print("RUNNING TRAJ GEN, SHOULD HAPPEN ONCE")
			print("Starting position: ", cur_pos)
			print("First wpt: ", self.path[:, 0])
			print("Second wpt: ", self.path[:, 1])
			print("last wpt: ", self.path[:, -1])
			print("Time series: ", self.ts)
			print("Total time: ", self.total_time)
			self.minSnapSet = True
			# TODO: do I need a return statement here? my code doesn't have an initialization loop....
			# return np.array([cur_pos[0], cur_pos[1], cur_pos[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

		# cur_time = 0.0						# maybe useful debugging... DO NOT INCLUDE THIS TERM
		if cur_time >= self.total_time:
			pos = self.path[:, -1] 					# TODO: tied this to the last achieved position, if the simulation length isn't long enough it'll jump, maybe have adaptive length/duration simulation
			vel = [0.0, 0.0, 0.0]
			acc = [0.0, 0.0, 0.0]
			# TODO: main reason for the jump right now is that there are no x-y values coming out of the optimizer
		else:

			k = (self.ts <= cur_time).nonzero()
			k = k[-1][-1]						# TODO: maybe because of a sign error the path never iterates wpts
			print(k)
			# THESE CALCS ARE OKAY
			pos_temp = np.array([cur_time**7, cur_time**6, cur_time**5, cur_time**4, cur_time**3, cur_time**2, cur_time, 1])
			pos = np.matmul(pos_temp, self.coef[8*k:8*(k+1), :])
			# print("pos coef: ", coef[8*k:8*(k+1), :])
			vel_temp = np.array([7*cur_time**6, 6*cur_time**5, 5*cur_time**4, 4*cur_time**3, 3*cur_time**2, 2*cur_time, 1, 0])
			vel = np.matmul(vel_temp, self.coef[8*k:8*(k+1), :])
			# print("vel coef: ", coef[8*k:8*(k+1), :])
			acc_temp = np.array([42*cur_time**5, 30*cur_time**4, 20*cur_time**3, 12*cur_time**2, 6*cur_time, 2, 0, 0])
			acc = np.matmul(acc_temp, self.coef[8*k:8*(k+1), :])
			# print(pos)
			# print(vel)
			# print(acc)
			# print(cur_pos)
			# print(path)
			# exit()
			# input()

		new_des_state = np.concatenate((pos, vel, acc))

		return new_des_state

	def generate_ts(self, path):
		speed = 0.5
		path_len = np.sum(np.sqrt(np.sum(np.power(path[:, 1:] - path[:, 0:-1], 2), 0)))
		total_time = path_len/speed
		path_seg_len = np.sqrt(np.sqrt(np.sum(np.power(path[:, 1:] - path[:, 0:-1], 2), 0)))
		ts = np.cumsum(path_seg_len)
		ts = ts/ts[-1]
		ts = np.insert(ts, 0, 0)
		ts = ts*total_time

		return ts, total_time

	def trajectory_optimization(self, path):
		#
		# path: 		m+1 x 3 array of desired waypoints
		#
		# X: 			8m x 3 array of optimal coeficients
		#					x_k(t)=X(8*k:8*(k+1))'*[t^3;t^2,t,1]
		# 					where d \in [1,2,3] to indicate dimension xyz

		# TODO: fix the coefficient generation to include x and y components
		path0 = path
		shape = path0.shape
		m = shape[0]
		n = shape[1]
		m = m-1 			# mathematical convenience
		eps = 2e-52 			# TODO: what is the purpose of this?, eps = 2e-52
								# WHEN THIS IS NOT 2E-52 THE Z COMPONENT GOES HAYWIRE, not sure why probably numerical issues
		# In a 7th order (minimum snap) trajectory optimization there are 8 parameters for each subpath
		X = np.zeros((8*m, n))
		A = np.zeros((8*m, 8*m, n))
		Y = np.zeros((8*m, n))

		# TODO: [95% certain this is fixed] verify that n+1 is needed and not n (NEED 0 INDEX IN THESE) (Python doesn't count last index, MATLAB does)
		for i in range(0, n):
			A[:, :, i] = np.eye(8*m) * eps

			# Constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a waypoint
			idx = 0 					# constraint counter
			# TODO: [95% certain this is fixed] verify that m is needed and not m-1
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = np.array([self.ts[k+1]**7, self.ts[k+1]**6, self.ts[k+1]**5, self.ts[k+1]**4, self.ts[k+1]**3, self.ts[k+1]**2, self.ts[k+1], 1.0]) 			# 1:8, 9:16, 17:24; want 0:7, 8:15, 16:23
				Y[idx, i] = path0[k+1, i]
				idx += 1
				A[idx, 8*(k+1):8*(k+2), i] = np.array([self.ts[k+1]**7, self.ts[k+1]**6, self.ts[k+1]**5, self.ts[k+1]**4, self.ts[k+1]**3, self.ts[k+1]**2, self.ts[k+1], 1]) 		# 9:16, 17:24; want 8:15, 16:23
				Y[idx, i] = path0[k+1, i]
				idx += 1

			# Constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = np.array([7*self.ts[k+1]**6, 6*self.ts[k+1]**5, 5*self.ts[k+1]**4, 4*self.ts[k+1]**3, 3*self.ts[k+1]**2, 2*self.ts[k+1], 1, 0])
				A[idx, 8*(k+1):8*(k+2), i] = np.array([-7*self.ts[k+1]**6, -6*self.ts[k+1]**5, -5*self.ts[k+1]**4, -4*self.ts[k+1]**3, -3*self.ts[k+1]**2, -2*self.ts[k+1], -1, 0]) 		# 9:16, 17:24; want 8:15, 16:23
				Y[idx, i] = 0
				idx += 1

			# Constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = [42*self.ts[k+1]**5, 30*self.ts[k+1]**4, 20*self.ts[k+1]**3, 12*self.ts[k+1]**2, 6*self.ts[k+1], 2, 0, 0]
				A[idx, 8*(k+1):8*(k+2), i] = [-42*self.ts[k+1]**5, -30*self.ts[k+1]**4, -20*self.ts[k+1]**3, -12*self.ts[k+1]**2, -6*self.ts[k+1], -2, 0, 0]
				Y[idx, i] = 0
				idx += 1

			# Constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = [210*self.ts[k+1]**4, 120*self.ts[k+1]**3, 60*self.ts[k+1]**2, 24*self.ts[k+1], 6, 0, 0, 0]
				A[idx, 8*(k+1):8*(k+2), i] = [-210*self.ts[k+1]**4, -120*self.ts[k+1]**3, -60*self.ts[k+1]**2, -24*self.ts[k+1], -6, 0, 0, 0]
				Y[idx, i] = 0
				idx += 1

			# Constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = [840*self.ts[k+1]**3, 360*self.ts[k+1]**2, 120*self.ts[k+1], 24, 0, 0, 0, 0]
				A[idx, 8*(k+1):8*(k+2), i] = [-840*self.ts[k+1]**3, -360*self.ts[k+1]**2, -120*self.ts[k+1], -24, 0, 0, 0, 0]
				Y[idx, i] = 0
				idx += 1

			# Constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = [2520*self.ts[k+1]**2, 720*self.ts[k+1], 120, 0, 0, 0, 0, 0]
				A[idx, 8*(k+1):8*(k+2), i] = [-2520*self.ts[k+1]**2, -720*self.ts[k+1], -120, 0, 0, 0, 0, 0]
				Y[idx, i] = 0
				idx += 1

			# Constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
			for k in range(0, m-1):
				A[idx, 8*k:8*(k+1), i] = [5040*self.ts[k+1], 720, 0, 0, 0, 0, 0, 0]
				A[idx, 8*(k+1):8*(k+2), i] = [-5040*self.ts[k+1], -720, 0, 0, 0, 0, 0, 0]
				Y[idx, i] = 0
				idx += 1

			# so far there are 8(m-1) constraints
			# there are 8 left:
			#    x_1(t_0) = p_0
			#    x^(1)_0(t_0) = 0
			#    x^(2)_0(t_0) = 0
			#    x^(3)_0(t_0) = 0
			#    x_T(t_T) = p_T
			#    x^(1)_T(t_T) = 0
			#    x^(2)_T(t_T) = 0
			#    x^(3)_T(t_T) = 0

			k = 0
			A[idx, 8*k:8*(k+1), i] = [self.ts[k]**7, self.ts[k]**6, self.ts[k]**5, self.ts[k]**4, self.ts[k]**3, self.ts[k]**2, self.ts[k], 1]
			Y[idx, i] = path0[k, i]
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [7*self.ts[k]**6, 6*self.ts[k]**5, 5*self.ts[k]**4, 4*self.ts[k]**3, 3*self.ts[k]**2, 2*self.ts[k], 1, 0]
			Y[idx, i] = 0
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [42*self.ts[k]**5, 30*self.ts[k]**4, 20*self.ts[k]**3, 12*self.ts[k]**2, 6*self.ts[k], 2, 0, 0]
			Y[idx, i] = 0
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [210*self.ts[k]**4, 120*self.ts[k]**3, 60*self.ts[k]**2, 24*self.ts[k], 6, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

			# TODO: [95% certain this is correct] double check that this should be "m-1" and not m
			k = m-1
			A[idx, 8*k:8*(k+1), i] = [self.ts[k+1]**7, self.ts[k+1]**6, self.ts[k+1]**5, self.ts[k+1]**4, self.ts[k+1]**3, self.ts[k+1]**2, self.ts[k+1], 1]
			Y[idx, i] = path0[k+1, i]
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [7*self.ts[k+1]**6, 6*self.ts[k+1]**5, 5*self.ts[k+1]**4, 4*self.ts[k+1]**3, 3*self.ts[k+1]**2, 2*self.ts[k+1], 1, 0]
			Y[idx, i] = 0
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [42*self.ts[k+1]**5, 30*self.ts[k+1]**4, 20*self.ts[k+1]**3, 12*self.ts[k+1]**2, 6*self.ts[k+1], 2, 0, 0]
			Y[idx, i] = 0
			idx += 1
			A[idx, 8*k:8*(k+1), i] = [210*self.ts[k+1]**4, 120*self.ts[k+1]**3, 60*self.ts[k+1]**2, 24*self.ts[k+1], 6, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

			X[:, i] = np.linalg.solve(A[:, :, i], Y[:, i])

		return X


	def pid_controller(self):
		"""
		Nested PID controller, the inner-loop is the attitute controller and the outer-loop is the position
		:return: 1x1 desired sum of prop thrusts in body frame, 3x1 desired angular velocity in body frame
		"""
		# define the basic nested PID controller for the quadcopter
		Kp_pos = np.array([2, 2, 2])
		Kd_pos = np.array([0, 0, 0])

		Kp_ang = np.ones(3)*3000
		Kd_ang = np.ones(3)*300
		print("state")
		print(self._state)
		print("desired state")
		print(self._desired_state)
		# Linear acceleration
		acc_des = self._desired_state[6:9] + Kd_pos*(self._desired_state[3:6] - self._state[3:6]) + Kp_pos*(self._desired_state[0:3] - self._state[0:3]) 	# 3x1
		print("acc_des")
		print(acc_des)
		# Build desired roll, pitch, and yaw
		des_yaw = self._desired_state[9]
		phi_des = (1/self._quad_properties["gravity"]) * (acc_des[0]*np.sin(des_yaw) - acc_des[1]*np.cos(des_yaw))
		theta_des = (1/self._quad_properties["gravity"]) * (acc_des[0]*np.cos(des_yaw) + acc_des[1]*np.sin(des_yaw))
		# theta_des = -theta_des
		# phi_des = -phi_des
		psi_des = des_yaw
		print("Desired ypt")
		print(des_yaw)
		print(phi_des)
		print(theta_des)
		euler_des = np.array([phi_des, theta_des, psi_des])
		# pqr_des = np.array([0, 0, self._desired_state[10]])

		# quat = self._state[6:10] 					# 4x1
		# euler = Quadcopter.quat2euler(quat) 		# 3x1
		# print("des yaw: ", des_yaw)
		# print("euler: ", euler)

		thrust = self._quad_properties["mass"] * (self._quad_properties["gravity"] + acc_des[2]) 		# 1x1
		# moment = np.matmul(self._quad_properties["inertia"], (Kd_ang*(pqr_des - self._state[10:13]) + Kp_ang*(euler_des - euler))) 		# 3x1

		# forces = np.array([thrust, moment[0], moment[1]]) 					# excludes the yaw force from physical limitations
		# prop_thrust = np.matmul(self._quad_properties["limitsA"], forces) 																		# 4x1
		# prop_thrust_limited = np.maximum(np.minimum(prop_thrust, self._quad_properties["max_force"]), self._quad_properties["min_force"]) 		# 4x1
		# new_thrust = np.matmul(self._quad_properties["limitsB"][0, :], prop_thrust_limited) 													# 1x1
		# new_moment = np.append(np.matmul(self._quad_properties["limitsB"][1:3], prop_thrust_limited), moment[2]) 						# 3x1

		return thrust, euler_des

	def equations_of_motion(self, controller_thrust, angular_force):
		"""
		Calculates the rate of change of the vehicle outputting the derivative of the state
		:param controller_thrust: 1x1 desired sum of prop thrusts in body frame
		:param angular_force: 3x1 desired angular velocity in body frame
		:return: 13x1 statedot, rate of change of current state due to controller inputs
		"""
		# define the quadcopter dynamics time step update
		assert str(np.shape(angular_force)) == "(3,)", "Incorrect moment matrix dimensions"
		forces = np.array([controller_thrust, angular_force[0], angular_force[1]]) 					# excludes the yaw force from physical limitations
		prop_thrust = np.matmul(self._quad_properties["limitsA"], forces) 																		# 4x1
		prop_thrust_limited = np.maximum(np.minimum(prop_thrust, self._quad_properties["max_force"]), self._quad_properties["min_force"]) 		# 4x1
		new_thrust = np.matmul(self._quad_properties["limitsB"][0, :], prop_thrust_limited) 													# 1x1
		new_ang_force = np.append(np.matmul(self._quad_properties["limitsB"][1:3], prop_thrust_limited), angular_force[2]) 						# 3x1

		# assign variables from the state array
		state = self._state
		x = state[0]
		y = state[1]
		z = state[2]
		xd = state[3]
		yd = state[4]
		zd = state[5]
		qw = state[6]
		qx = state[7]
		qy = state[8]
		qz = state[9]
		p = state[10]
		q = state[11]
		r = state[12]

		# Orientation
		quat = np.array([qw, qx, qy, qz])
		quat[0] = 1.0
		bRw = Quadcopter.quat2rot(quat) 											# 3x3
		wRb = np.transpose(bRw) 													# 3x3, transforms body frame to world frame

		# Linear Acceleration
		z_thrust = np.array([0, 0, new_thrust]) 									# 3x1
		downward_force = np.array([0, 0, (self._quad_properties["mass"]*self._quad_properties["gravity"])]) 	# 3x1
		linear_accel = 1 / self._quad_properties["mass"] * (np.matmul(wRb, z_thrust) - downward_force) 			# 3x1, in world frame

		# Constraint quaternion (must have norm of 1)
		quat_err = 1 - np.sum(np.square(quat)) 										# 1x1
		# TODO: implement skew symmetric matrix check
		# TODO: encapsulte in quaternion class
		# symmetric check: (arr.transpose() == arr).all()
		# skew symmetric check: (arr.transpose() == -arr).all()

		q_special_form = np.array([
			[0, -p, -q, -r],
			[p, 0, -r, q],
			[q, r, 0, -p],
			[r, -q, p, 0]
		])
		qdot = -0.5*np.matmul(q_special_form, quat) + 2*quat_err*quat

		# Angular Acceleration
		# TODO: validate the size of these numbers, they seem large
		# TODO: implement check for diagonal omegadot, off-diagonal values should be 0
		omega = np.array([p, q, r]) 												# 3x1
		omegadot = self._quad_properties["invI"] * (new_ang_force - np.cross(omega, (np.matmul(self._quad_properties["inertia"], omega)))) 		# 3x3

		# Create state derivative
		statedot = np.array([
			xd,
			yd,
			zd,
			linear_accel[0],
			linear_accel[1],
			linear_accel[2],
			qdot[0],
			qdot[1],
			qdot[2],
			qdot[3],
			omegadot[0, 0],
			omegadot[1, 1],
			omegadot[2, 2]
		])
		assert str(np.shape(statedot)) == "(13,)", "statedot is of the wrong dimension"

		return statedot 					# 13x1

	def simulation_step(self, s=0, t=0, total_sim_time=0):
		"""
		Combines a trajectory with a controller with a definition of dynamics to return a rate of change of the current
		state for integration in main.py to get the new state s.
		:param s: 13x1 current state
		:param t: current time step for the odeint() solver
		:param total_sim_time: total time allowed to finish the simple line trajectory
		:return: 13x1 statedot, rate of change of current state due to controller inputs
		"""
		self._state = s
		assert str(np.shape(self._state)) == "(13,)", "Incorrect state vector size in simulation_step: {}".format(np.shape(self._state))
		assert str(np.shape(self._desired_state)) == "(11,)", "Incorrect desired state vector size in simulation_step: {}".format(np.shape(self._desired_state))
		assert str(np.shape(self._goal)) == "(11,)", "Incorrect goal vector size in simulation_step: {}".format(np.shape(self._goal))

		# This is where the lego blocks go together. Change trajectories, controllers, and dynamics here.
		new_des_state = self.simple_line_trajectory(self._state[0:3], self._goal[0:3], total_sim_time, t)				# straight line
		# new_des_state = self.hover_trajectory(self._state[0:3], t) 													# hover
		# new_des_state = self.minimun_snap_trajectory(self._state[0:3], t)												# minimum snap trajecotry with default path
		self._desired_state[0:9] = new_des_state
		# if t > 3.95:
		# 	print(new_des_state)
		thrust, moment = self.pid_controller()
		statedot = self.equations_of_motion(thrust, moment)

		return statedot