#!/usr/bin/env python
import matplotlib.pyplot as plt
import fusion_ekf as fus_ekf
import numpy as np

"""
1. This function reads the measurement data file and estimates the 
position of the tracked object.
2. To estimate the position, it uses linear Kalman Filter or EKF 
depending on measurement data is from laser or radar.  
"""

def fused_estimate():
	fusionEKF = fus_ekf.FusionEKF()
	count = 0

	plt.figure(1)
	plt.xlim((-30,25))
	plt.ylim((-15,25))
	plt.legend()
	with open("data/obj_pose-laser-radar-synthetic-input.txt") as readf:
		line = readf.readline()
		while line:
			data_list = line.split("\t")

			processed_meas = []
			processed_meas.append(data_list[0])
			for i in range(1, len(data_list)):
				processed_meas.append(float(data_list[i]))

			fusionEKF.process_est(processed_meas)
			x_est, y_est = fusionEKF._ekf._x[0, 0], fusionEKF._ekf._x[1, 0]

			# Start plotting
			if count == 2:
				plt.legend(loc="lower right")
			if count == 0:
				plt.plot(x_est, y_est, "g.", markersize=2, label="estimate")
			else:
				plt.plot(x_est, y_est, "g.", markersize=2)
			if processed_meas[0] == "R":
				plt.plot(processed_meas[1]*np.cos(processed_meas[2]), processed_meas[1]*np.sin(processed_meas[2]), "b.", markersize=2, label="radar meas")
			else:
				plt.plot(processed_meas[1], processed_meas[2],"r.", markersize=2, label="laser meas")
			plt.pause(0.10)
			# End plotting

			print("---------------------")
			count += 1
			line = readf.readline()

if __name__=="__main__":
	fused_estimate()
