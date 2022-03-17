#!/usr/bin/env python

import Tkinter as tk
import rospy
import actionlib
import read_sensor.msg

RED = "\033[31m"
BOLDRED = "\033[1m\033[31m"
RESET = "\033[0m"
GREEN = "\033[32m"
BOLDGREEN = "\033[1m\033[32m"

def pressed(server,num_samp):
	client = actionlib.SimpleActionClient(server, read_sensor.msg.ComputeBiasAction)
	wait_serv = client.wait_for_server(timeout = rospy.Duration(1.0))
	if not wait_serv:
		print(BOLDRED+"Error: "+RESET+"Action server for "+server[1:5]+" not found.")
	else:
		goal = read_sensor.msg.ComputeBiasGoal(num_samples=num_samp)
		if(client.send_goal_and_wait(goal)==3):
			print(GREEN+server[1:5]+":"+RESET+" reset bias completed.")


def client_gui():
	rospy.init_node('client_gui', anonymous=True)
	topic_list = rospy.get_published_topics()
	server_list = []
	for topic in topic_list:
		if "action_compute_bias/result" in topic[0]:
			server_list.append(topic[0][:-7])
	if len(server_list) == 0:
		print(BOLDRED+"Error:"+RESET+" no action server for bias found. Please, make sure read_sensor_as_node is running.")
		exit(-1)
	window = tk.Tk()
	window.title("Reset Bias")
	window.minsize(10,10)
	
	num_samp_lbl = tk.Label(window, text="# of samples:", font=('helvetica', 20))
	num_samp_lbl.grid(column=0, row=0, padx=5, pady=10)

	num_samp_form = tk.Entry(window, width=5, font=('helvetica', 20))
	num_samp_form.insert(0,"50")
	num_samp_form.grid(column=1, row=0, padx=5, pady=10)

	button=[]
	for i,server in enumerate(server_list):
		button.append(tk.Button(window, 
		                   text="Reset "+server[1:5],
		                   activeforeground = "red",
		                   bd = 3,
		                   fg="black",
		                   font=('helvetica', 20, 'bold'),
		                   command=lambda s=server: pressed(s,int(num_samp_form.get()))))
		button[i].grid(columnspan=2, row=i+1, padx=10, pady=10)

	window.mainloop()
		
		
if __name__ == '__main__':
	try:
		client_gui()
	except rospy.ROSInterruptException:
		pass
		