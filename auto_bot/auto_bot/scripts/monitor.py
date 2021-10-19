#!/usr/bin/env python

import rospy
import serial
	
import threading

from time import sleep
from threading import Timer
from std_msgs.msg import Float32

from sensor_msgs.msg import Joy
from auto_bot.msg import ps4 #subject to change "package name" to match your project
from std_msgs.msg import Header 

#PS4 message type-----------------------------------------------------

ps4_out_message = ps4()

#Variables--------------------------------------------------------------------
monitor = serial.Serial(port = "/dev/ttyUSB0", \
                      baudrate = 115200, \
                      parity = serial.PARITY_NONE, \
                      stopbits = serial.STOPBITS_ONE, \
                      bytesize = serial.EIGHTBITS, \
                      timeout = 1)
count = 0
period = 0.2
ros_period = 0.02
    
#Function to init the monitor-------------------------------------------------
def monitor_init():
	#Global Variable
	global monitor

	init_finish = False
	while not init_finish:
		if monitor.writable():
			monitor.write('CLS(0);\r\n')
			init_finish = True		

#Function to update DATA---------------------------------------------------
def write_sth_to_monitor():
	global monitor , count
	monitor.write("DS64(2,2,'Yaw: %.2f ',2);\r\n"%count)


def add_one():
	
	global count, period
		
	count = count +1
	
	threading.Timer(period, add_one).start()

def handler(data):
	ps4_out_message.square = data.buttons[0]
	ps4_out_message.cross = data.buttons[1]
	ps4_out_message.circle = data.buttons[2]
	ps4_out_message.triangle = data.buttons[3]
	ps4_out_message.l1 = data.buttons[4]
	ps4_out_message.r1 = data.buttons[5]
	ps4_out_message.l2 = data.buttons[6]
	ps4_out_message.r2 = data.buttons[7]
	ps4_out_message.share = data.buttons[8]
	ps4_out_message.option = data.buttons[9]
	ps4_out_message.l3 = data.buttons[10]
	ps4_out_message.r3 = data.buttons[11]
	ps4_out_message.ps = data.buttons[12]
	ps4_out_message.touch = data.buttons[13]
	ps4_out_message.leftx = int(round(data.axes[0]*-100)) + 100
	ps4_out_message.lefty = int(round(data.axes[1]*100)) + 100
	ps4_out_message.rightx = int(round(data.axes[2]*-100)) + 100
	ps4_out_message.analog_l2 = int(round(data.axes[3]*100)) + 100
	ps4_out_message.analog_r2 = int(round(data.axes[4]*100)) + 100
	ps4_out_message.righty = int(round(data.axes[5]*100)) + 100
	ps4_out_message.left_right_pad = int(data.axes[6])
	ps4_out_message.up_down_pad = int(data.axes[7])
	ps4_out_message.header.stamp = rospy.Time.now()
	 
#functions-----------------------------------------------
def ros_init():

	rospy.init_node('show_spinner_data')
	rospy.init_node('PS4_Processing_Node')

	#publisher
	PS4_Message_Publisher = rospy.Publisher('Auto_Bot_PS4',ps4, queue_size=100)
	pub_cur_time = rospy.Publisher('cur_time', Float32, queue_size = 100)
	
	#subscriber
	rospy.Subscriber('joy', Joy, handler)
	
	rate = rospy.Rate(2)

def pub_at_50Hz_loop():

	global count , ros_period , ps4_message
	
	#PUBLISHING EVENT-------------------------------------
	pub_cur_time.publish(count)
	PS4_Message_Publisher.publish(ps4_out_message)
	
	threading.Timer(ros_period, pub_at_50Hz_loop).start()
	
	
		
    
#Main function----------------------------------------------------------------
if __name__ == '__main__':
	
	rospy.init_node('show_spinner_data')
	rospy.init_node('PS4_Processing_Node')

	#publisher
	PS4_Message_Publisher = rospy.Publisher('Auto_Bot_PS4',ps4, queue_size=100)
	pub_cur_time = rospy.Publisher('cur_time', Float32, queue_size = 100)
	
	#subscriber
	rospy.Subscriber('joy', Joy, handler)
	
	rate = rospy.Rate(2)
	
	#Init Monitor
	monitor_init()
	
	add_one()
	
	pub_at_50Hz_loop()
	
	while True:
		write_sth_to_monitor()

