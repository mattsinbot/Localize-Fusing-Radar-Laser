#!/usr/bin/env python
import matplotlib.pyplot as plt
import fusion_ekf as fus_ekf


def fused_estimate():
	fusionEKF = fus_ekf.FusionEKF()
	count = 0
	x_data, y_data = list(), list()
	
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
			print("count: %d, x_est: %2.4f, y_est: %2.4f"%(count, x_est, y_est))
			x_data.append(x_est)
			y_data.append(y_est)
			
			line = readf.readline()
			
	plt.plot(x_data, y_data)
	plt.show()
			
if __name__=="__main__":
	fused_estimate()

