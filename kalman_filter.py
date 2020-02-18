#!/usr/bin/env python
import numpy as np
import tool

class KalmanFilter(object):
	def __init__(self):
		self._x = None
		self._P = None
		self._F = None
		self._Q = None
		self._H = None
		self._R = None

	# Prediction method (same for radar and laser data)
	def predict(self):
		self._x = np.dot(self._F, self._x)
		self._P = np.dot(np.dot(self._F, self._P), self._F.T) + self._Q
		print("prediction step: x: %2.4f, y: %2.4f, dx: %2.4f, dy: %2.4f"%(self._x[0], self._x[1], self._x[2], self._x[3]))

	# Update method for linear laser measurement model
	def update(self, z):
		z_pred = np.dot(self._H, self._x)

		y = z - z_pred
		Ht = self._H.T
		PHt = np.dot(self._P, Ht)
		S = np.dot(self._H, PHt) + self._R
		Si = np.linalg.inv(S)
		K = np.dot(PHt, Si)

		self._x += np.dot(K, y)
		I = np.eye(self._x.shape[0])
		self._P = np.dot(I - np.dot(K, self._H), self._P)
		print("update step: x: %2.4f, y: %2.4f, dx: %2.4f, dy: %2.4f"%(self._x[0], self._x[1], self._x[2], self._x[3]))

	# Update method for non-linear radar measurement model
	def updateEKF(self, z):
		z_pred = tool.Cart2Polar(self._x)
		y = z - z_pred

		if y[1, 0] > np.pi:
			y[1, 0] -= 2*np.pi

		if y[1, 0] < -np.pi:
			y[1, 0] += 2*np.pi

		# Below is copied from update method
		Ht = self._H.T
		PHt = np.dot(self._P, Ht)
		S = np.dot(self._H, PHt) + self._R
		Si = np.linalg.inv(S)
		K = np.dot(PHt, Si)

		self._x += np.dot(K, y)
		I = np.eye(self._x.shape[0])
		self._P = np.dot(I - np.dot(K, self._H), self._P)
		print("update step: x: %2.4f, y: %2.4f, dx: %2.4f, dy: %2.4f"%(self._x[0], self._x[1], self._x[2], self._x[3]))
