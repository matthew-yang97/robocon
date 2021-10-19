#!/usr/bin/env python

import rospy
import serial
	
from std_msgs.msg import Float32

#Variableys--------------------------------------------------------------------
monitor = serial.Serial(port = "/dev/ttyUSB0", \
                      baudrate = 115200, \
                      parity = serial.PARITY_NONE, \
                      stopbits = serial.STOPBITS_ONE, \
                      bytesize = serial.EIGHTBITS, \
                      timeout = 1)
X_Cord = 0.0
Y_Cord = 0.0
Degree = 0.0
    
#Function to init the monitor-------------------------------------------------
def monitor_init():
	#Global Variable
	global monitor

	init_finish = False
	while not init_finish:
		if monitor.writable():
			monitor.write('CLS(0);\r\n')
			init_finish = True		

#main functions--------------------------------------
def write_sth_to_monitor(event):

	global X_Cord , Y_Cord , Degree

	if monitor.writable():

		monitor.write("DS64(2,2,'Coordinate Feedback by KELLO',2);")
				
		monitor.write("DS64(2,75,'X_Cord: %.5s   ',2);"%X_Cord)
		
		monitor.write("DS64(400,75,'Y_Cord: %.5s   ',2);"%Y_Cord)
		
		monitor.write("DS64(2,140,'Degree: %.2s     ',2);\r\n"%Degree)

#subscriber-----------------------------------------------
def sub_Xaxis_callback(msg):
	global X_Cord
	X_Cord = msg.data

def sub_Yaxis_callback(msg):
	global Y_Cord
	Y_Cord = msg.data

def sub_Degree_callback(msg):
	global Degree
	Degree = msg.data
	
#init function-----------------------------------------------------------
def init_for_ros():
	
	rospy.init_node('Cordinate_Feedback')

	sub_Xaxis = rospy.Subscriber('Xaxis', Float32 , sub_Xaxis_callback)
	sub_Yaxis = rospy.Subscriber('Yaxis', Float32 , sub_Yaxis_callback)
	sub_Degree = rospy.Subscriber('Degree', Float32 , sub_Degree_callback)
	
#Main function----------------------------------------------------------------
if __name__ == '__main__':
	
	#Init ROS
	init_for_ros()
	
	#Init Monitor
	monitor_init()
	
	#Timer Interrupt for differnet freq. loop
	rospy.Timer(rospy.Duration(.3), write_sth_to_monitor)
	
	rospy.spin()
