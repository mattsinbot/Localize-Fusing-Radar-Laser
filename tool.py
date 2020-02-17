#!/usr/bin/env python
import numpy as np

def Cart2Polar(x):
	px = x[0]
	py = x[1]
	vx = x[2]
	vy = x[3]
	
	rho = np.sqrt(px**2 + py**2)
	phi = np.arctan2(py, px)
	
	# handle division by zero
	if rho < 1e-4:
		rho = 1e-4
	
	rho_dot = (px*vx + py*vy)/rho
	
	z_pred = np.zeros((3, 1))
	z_pred[0, 0] = rho
	z_pred[1, 0] = phi
	z_pred[2, 0] = rho_dot
	
	return z_pred
	
def ComputeJac(x):
	px = x[0, 0]
	py = x[1, 0]
	vx = x[2, 0]
	vy = x[3, 0]
	
	c1 = px**2 + py**2
	c2 = np.sqrt(c1)
	c3 = c1*c2
	
	if c1 < 1e-4:
		Hj = np.zeros((3, 4))
		return Hj
		
	Hj = np.array([[px/c2, py/c2, 0, 0], 
							   [-py/c1, px/c1, 0, 0], 
							   [py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2]])
							   
	return Hj

