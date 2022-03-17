#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32MultiArray
from read_sensor.msg import tactile_sensor_data
import matplotlib.pyplot as plt
#import matplotlib
from matplotlib import animation
import numpy as np
import cv2

RED = "\033[31m"
BOLDRED = "\033[1m\033[31m"
RESET = "\033[0m"


what_to_plot = 0
params = []
deltav = [0]*25
centroids = [0]*6

xi = np.linspace(-7.1,7.1,5)	#[-7.1, -3.55, 0, 3.55, 7.1]
yi = xi[::-1]
X,Y = np.meshgrid(xi,yi)

fig = plt.figure(figsize=[7.6, 6.6])

ax = plt.axes(xlabel='x [mm]', xlim=(-12.5, 12.5),\
	      ylabel='y [mm]', ylim=(-12.5, 12.5),\
	      aspect='equal')
# ax.invert_xaxis(); # to see the sensor upside down
# ax.invert_yaxis(); # to see the sensor upside down
cen, = ax.plot([], [], '.', c='lime')
line, = ax.plot([], [], lw=1, c='r')
sca = plt.scatter(np.reshape(X,25), np.reshape(Y,25), s=1, c='b')


def callback_params(data):
	global params
	params = data.data
	
def callback_deltav(data):
	global deltav
	deltav = data.tactile_sensor_data

def callback_centroids(data):
	global centroids
	centroids = data.data


	
def animate(i):
	
	if what_to_plot == 0 or (params[0]==0.0 and params[1]==0.0):
		line.set_data([],[])
		cen.set_data([],[])
	else:
		if what_to_plot == 1:
			x = np.linspace(-12.5,12.5,5)
			y = x*params[0]+params[1]
			line.set_data(x,y)
		if centroids[0]==0.0:
			cen.set_data(xi,centroids[1:6])
			if what_to_plot == 2:
				x = np.linspace(-12.5,12.5,50)
				y = x**2*params[0]+x*params[1]+params[2]
				line.set_data(x,y)
		elif centroids[0]==1.0:
			cen.set_data(centroids[1:6],yi)
			if what_to_plot == 2:
				y = np.linspace(-12.5,12.5,50)
				x = y**2*params[0]+y*params[1]+params[2]
				line.set_data(x,y)
		
	sca.set_sizes([500*n if n>0.01 else 1 for n in deltav])


	
def plotter(invert=0):

	if (invert):
		ax.invert_xaxis(); # to see the sensor upside down
		ax.invert_yaxis(); # to see the sensor upside down
	
	rospy.init_node('plotter', anonymous=True)
	topic_list = rospy.get_published_topics()
	sensor_list = []
	for topic in topic_list:
		if "TactileData" in topic[0] and not "raw" in topic[0]:
			sensor_list.append(topic[0])
	if len(sensor_list) == 0:
		print(BOLDRED+"Error:"+RESET+" no tactile sensor is publishing. Please, make sure read_sensor_node is running.")
		exit(-1)
	elif len(sensor_list) == 1:
		topic_name = sensor_list[0]
	else:
		print("Available Sensors:")
		for i,sensor in enumerate(sensor_list):
			print("["+ str(i) + "] - " + sensor)
		choice = -1
		while (choice>len(sensor_list)-1 or choice<0):
			choice = input("Select the Sensor to use: ")
			if (choice>len(sensor_list)-1 or choice<0):
				print(RED+"A wrong number has been inserted."+RESET)
		topic_name = sensor_list[choice]
	sensor_name = topic_name[-4:]
	fig.canvas.set_window_title('Tactile Map '+sensor_name)
	
	rospy.Subscriber(topic_name, tactile_sensor_data, callback_deltav)
	
	if any("params_"+sensor_name in i[0] for i in topic_list):
		rospy.Subscriber("centroids_"+sensor_name, Float32MultiArray, callback_centroids)
		print("\nWhat do you want to plot?")
		print("[0] - Tactile map")
		print("[1] - Tactile map and wire estimation (FIRST order)")
		print("[2] - Tactile map and wire estimation (SECOND order)")
		global what_to_plot
		what_to_plot = 10
		while (what_to_plot>2 or what_to_plot<0):
			what_to_plot = input("Insert the corresponding number: ")
			if what_to_plot>2 or what_to_plot<0:
				print(RED+"A wrong number has been inserted."+RESET)
		if what_to_plot == 1:
			rospy.Subscriber("first_order_params_"+sensor_name, Float32MultiArray, callback_params)
		elif what_to_plot == 2:
			rospy.Subscriber("second_order_params_"+sensor_name, Float32MultiArray, callback_params)
	
	ani = animation.FuncAnimation(fig, animate, interval=10)
	plt.show()
		
		
if __name__ == '__main__':
	try:
		if (len(sys.argv) > 1):
			plotter(1)
		else:
			plotter()
	except rospy.ROSInterruptException:
		pass
		
		

