#!/usr/bin/env python
import matplotlib.pyplot as plt
import kalman_filter as kf
import numpy as np
import tool

class FusionEKF(object):
	def __init__(self):
		self._ekf = kf.KalmanFilter()
		self._is_initialized = False
		self._prev_timestmp = 0
		self._R_laser = np.array([[0.0225, 0], [0, 0.0225]])
		self._R_radar = np.array([[0.09, 0, 0], [0, 0.0009, 0], [0, 0, 0.09]])
		self._H_laser = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
		self._H_radar = None  # jacobian of radar measurement model
		
		self.noise_ax = 9
		self.noise_ay = 9
		
		self._ekf._P = np.array([[1, 0, 0, 0], 
		                        [0, 1, 0, 0], 
		                        [0, 0, 1000, 0], 
		                        [0, 0, 0, 1000]])
		                        
		self._ekf._F = np.array([[1, 0, 1, 0],
								 [0, 1, 0, 1],
								 [0, 0, 1, 0],
								 [0, 0, 0, 1]])
													 
	def process_est(self, meas):
		if not self._is_initialized:
			if meas[0] == "R":
				rho = meas[1]
				phi = meas[2]
				rho_dot = meas[3]
				
				self._ekf._x = np.array([[rho*np.cos(phi)], [rho*np.sin(phi)], [0], [0]])
				self._prev_timestmp = meas[4]
			else:
				self._ekf._x = np.array([[meas[1]], [meas[2]], [0], [0]])
				self._prev_timestmp = meas[3]
				
			self._is_initialized = True
			
			return
			
		# While already initialized
		if meas[0] == "R":
			dt = (meas[4] - self._prev_timestmp) / 1000000.0
			self._prev_timestmp = meas[4]
			z = np.array([[meas[1]], [meas[2]], [meas[3]]])
		else:
			dt = (meas[3] - self._prev_timestmp) / 1000000.0
			self._prev_timestmp = meas[3]
			z = np.array([[meas[1]], [meas[2]]])
			
			
		# modify the state-transition matrix with current dt value
		self._ekf._F[0, 2] = dt
		self._ekf._F[1, 3] = dt
		
		# set the process covariance matrix self._ekf._Q
		self._ekf._Q = np.array([[(dt**4)/(4*self.noise_ax), 0, (dt**3)/(2*self.noise_ax), 0],
														 [0, (dt**4)/(4*self.noise_ay), 0, (dt**3)/(2*self.noise_ay)],
														 [(dt**3)/(2*self.noise_ax), 0, (dt**2)*self.noise_ax, 0],
														 [0, (dt**3)/(2*self.noise_ay), 0, (dt**2)*self.noise_ay]])
				 
		# predict
		self._ekf.predict()
		
		# update (case1: updateEKF for RADAR or case2: update for LASER)
		if meas[0] == "R":
			H_jac = tool.ComputeJac(self._ekf._x)
			self._ekf._H = H_jac
			self._ekf._R = self._R_radar
			self._ekf.updateEKF(z)
		else:
			self._ekf._H = self._H_laser
			self._ekf._R = self._R_laser
			self._ekf.update(z)
