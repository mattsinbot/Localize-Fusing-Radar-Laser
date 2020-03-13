#!/usr/bin/env python
import matplotlib.pyplot as plt

x_data, y_data = list(), list()
with open("data/obj_pose-laser-radar-synthetic-input.txt") as readf:
	line = readf.readline()
	while line:
		data_list = line.split("\t")
		if data_list[0] == "L":
			x_data.append(float(data_list[4]))
			y_data.append(float(data_list[5]))
		else:
			x_data.append(float(data_list[5]))
			y_data.append(float(data_list[6]))
		line = readf.readline()

# plot
for i in range(len(x_data)):
	plt.clf()
	plt.xlim((-30,25))
	plt.ylim((-30,25))
	plt.grid(True)
	plt.plot(x_data[i], y_data[i], "ro")
	plt.pause(0.01)
	
plt.plot(x_data, y_data)
plt.show()

