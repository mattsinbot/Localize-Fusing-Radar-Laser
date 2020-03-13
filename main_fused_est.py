#!/usr/bin/env python
import matplotlib.pyplot as plt
import fusion_ekf as fus_ekf
import numpy as np


def fused_estimate():
	fusionEKF = fus_ekf.FusionEKF()
	count = 0
	x_data, y_data = list(), list()
	x_gth, y_gth = list(), list()
	x_rad, y_rad = list(), list()
	x_las, y_las = list(), list()

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
			count += 1
			print("---------------------")
			x_data.append(x_est)
			y_data.append(y_est)

			# Store data for plotting
			if processed_meas[0] == "R":
				x_gth.append(processed_meas[5])
				y_gth.append(processed_meas[6])
				x_rad.append(processed_meas[1]*np.cos(processed_meas[2]))
				y_rad.append(processed_meas[1]*np.sin(processed_meas[2]))
			else:
				x_gth.append(processed_meas[4])
				y_gth.append(processed_meas[5])
				x_las.append(processed_meas[1])
				y_las.append(processed_meas[2])

			line = readf.readline()

	plt.figure(1)
	plt.plot(x_data, y_data, "r--", label="estimate")
	plt.plot(x_gth, y_gth, "g--", label="grnd truth")
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")
	plt.title("LOCALIZATION fusing RADAR and LASER data")
	plt.legend(loc="best")

	plt.figure(2)
	est_data2 = plt.plot(x_data, y_data, color="green", marker="D", markersize=2, label="estimate")
	laser_meas = plt.plot(x_las, y_las, color="red", marker="D", markersize=3, label="laser meas", linestyle="None")
	radar_meas = plt.plot(x_rad, y_rad, color="blue", marker="D", markersize=3, label="radar meas", linestyle="None")
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")
	plt.legend(loc="best")

	plt.show()

if __name__=="__main__":
	fused_estimate()
