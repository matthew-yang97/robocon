#!/usr/bin/env python
#Author: Ah Jo

import rospy
import serial

from auto_bot.msg import chassic_speed
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from std_msgs.msg import Float32

#Variables--------------------------------------------------------------------
monitor = serial.Serial(port = "/dev/ttyUSB0", \
                      baudrate = 115200, \
                      parity = serial.PARITY_NONE, \
                      stopbits = serial.STOPBITS_ONE, \
                      bytesize = serial.EIGHTBITS, \
                      timeout = 1)
X_cor = 0
Y_cor = 0
motor0 = 0
motor1 = 0
motor2 = 0
motor3 = 0

#Function to update coordinates value-------------------------------------------------
def X_handler(income_x):
	global X_cor
	X_cor = income_x.data

def Y_handler(income_y):
	global Y_cor
	Y_cor = income_y.data

def chassic_speed_handler(income_speed):
	global motor0, motor1, motor2, motor3

	motor0 = income_speed.motor0_power
	motor1 = income_speed.motor1_power
	motor2 = income_speed.motor2_power
	motor3 = income_speed.motor3_power

#Function to init the monitor-------------------------------------------------
def monitor_init():
	#Global Variable
	global monitor

	init_finish = False
	while not init_finish:
		if monitor.writable():
			monitor.write('CLS(0);\r\n')
			init_finish = True

#Function to update monitor---------------------------------------------------
def monitor_update(event):
	#Global Variable
	global monitor, X_cor, Y_cor, motor0, motor1, motor2, motor3

	if monitor.writable():

		monitor.write("DS64(2,2,'X, Y: %.2f   %.2f      ',2);"%(X_cor,Y_cor))
		monitor.write("DS64(2,70,'Motor_0: %.2f',2);"%motor0)
		monitor.write("DS64(2,138,'Motor_1: %.2f',2);"%motor1)
		monitor.write("DS64(2,206,'Motor_2: %.2f',2);"%motor2)
		monitor.write("DS64(2,276,'Motor_3: %.2f',2);"%motor3)
		monitor.write("DS64(2,408,'Working',2);\r\n")

#Main function----------------------------------------------------------------
if __name__ == '__main__':
	
	#Init Monitor
	monitor_init()
	
	#Init ROS Node
	rospy.init_node('Uart_Monitor_Node')

	#Init ROS Subscriber
	rospy.Subscriber('Xaxis', Float32, X_handler)
	rospy.Subscriber('Yaxis', Float32, Y_handler)
	rospy.Subscriber('Auto_Bot_Chassic_Speed', chassic_speed, chassic_speed_handler)
    
	rospy.Timer(rospy.Duration(.4), monitor_update)
    
	#ROS Loop
	rospy.spin()
