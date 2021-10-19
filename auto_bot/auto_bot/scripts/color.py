#!/usr/bin/env python
# up = 0 , down = 1 , left = 2 , right = 3 , triangle = 4, cross = 5 , square = 6 , circle = 7 , 
# L1 = 8 , R1 = 9 , L2 = 10 , R2 = 11 , L3 = 12 , R3 = 13 
#
import rospy
import serial
	
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from sensor_msgs.msg import Joy

from auto_bot.msg import ps4
	#subject to change "package name" to match your project
from std_msgs.msg import Header 

#PS4 message type-----------------------------------------------------
ps4_msg = ps4()

#global var for buttons ------------------------------------
cur_square = False
cur_triangle = False 
cur_cross = False
cur_circle =  False

cur_l1 = False
cur_r1 = False
cur_l2 = False
cur_r2 = False
cur_l3 = False
cur_r3 = False

cur_up = False
cur_dpwn = False
cur_left = False
cur_right = False

cur_share = False
cur_option = False
cur_ps = False

pre_square = False
pre_triangle = False 
pre_cross = False
pre_circle =  False

pre_l1 = False
pre_r1 = False
pre_l2 = False
pre_r2 = False
pre_l3 = False
pre_r3 = False

pre_up = False
pre_down = False
pre_left = False
pre_right = False

pre_share = False
pre_option = False
pre_ps = False

#Variables--------------------------------------------------------------------
monitor = serial.Serial(port = "/dev/ttyUSB0", \
                      baudrate = 115200, \
                      parity = serial.PARITY_NONE, \
                      stopbits = serial.STOPBITS_ONE, \
                      bytesize = serial.EIGHTBITS, \
                      timeout = 1)

red = 0.0
green = 0.0
blue = 0.0
clear = 0.0
                          
                         
#Function to init the monitor-------------------------------------------------
def monitor_init():
	#Global Variable
	global monitor

	init_finish = False
	while not init_finish:
		if monitor.writable():
			monitor.write('CLS(0);\r\n')
			init_finish = True		
			
#PS4 message formation function---------------------------------------


def handler(data):

	global pre_up , pre_down , pre_left, pre_right , pre_square , pre_circle , pre_cross , pre_triangle , pre_l1 , pre_l2, pre_l3 , pre_r1 , pre_r2 , pre_r3 , pre_share , pre_option , pre_ps	

	ps4_msg.square = data.buttons[3]

	ps4_msg.cross = data.buttons[0]

	ps4_msg.circle = data.buttons[1]

	ps4_msg.triangle = data.buttons[2]

	ps4_msg.l1 = data.buttons[4]

	ps4_msg.r1 = data.buttons[5]

	ps4_msg.l2 = data.buttons[6]

	ps4_msg.r2 = data.buttons[7]

	ps4_msg.share = data.buttons[8]

	ps4_msg.option = data.buttons[9]

	ps4_msg.l3 = data.buttons[11]

	ps4_msg.r3 = data.buttons[12]

	ps4_msg.ps = data.buttons[10]

	ps4_msg.leftx = int(round(data.axes[0]*-100)) + 100

	ps4_msg.lefty = int(round(data.axes[1]*100)) + 100

	ps4_msg.rightx = 100 - int(round(data.axes[3]*100)) 
	
	ps4_msg.righty = int(round(data.axes[4]*100)) + 100

	ps4_msg.analog_l2 = 100 - int(round(data.axes[2]*100))

	ps4_msg.analog_r2 = 100 - int(round(data.axes[5]*100))
	
	if (data.axes[6] == -1):
		ps4_msg.right = True
		ps4_msg.left = False

	elif (data.axes[6] == 1):
		ps4_msg.left = True
		ps4_msg.right = False

	elif (data.axes[6] == 0):
		ps4_msg.right = False
		ps4_msg.left = False
		
	if (data.axes[7] == -1):
		ps4_msg.down = True
		ps4_msg.up = False

	elif (data.axes[7] == 1):
		ps4_msg.up = True
		ps4_msg.down = False

	elif (data.axes[6] == 0):
		ps4_msg.up = False
		ps4_msg.down = False

	#update preveious data------------------------------

	pre_up = ps4_msg.up
	pre_down = ps4_msg.down
	pre_left = ps4_msg.left
	pre_right = ps4_msg.right

	pre_l1 = ps4_msg.l1
	pre_r1 = ps4_msg.r1

	pre_circle = ps4_msg.circle
	pre_square = ps4_msg.square
	pre_cross = ps4_msg.cross
	pre_triangle = ps4_msg.triangle
	
	ps4_msg.header.stamp = rospy.Time.now()
	

def write_sth_to_monitor(event):

	global monitor , red , green , blue , clear

	if monitor.writable():
	
		monitor.write("DS64(2,2,'Read The Fucking Color',2);")
		monitor.write("DS64(2,75,'Red: %s   ',2);"%red)
		monitor.write("DS64(2,140,'Green: %s    ',2);"%green)
		monitor.write("DS64(2,200,'Blue: %s     ',2);"%blue)
		monitor.write("DS64(2,260,'Clear: %s      ',2);\r\n"%clear)

#subscriber-----------------------------------------------
def sub_red_callback(msg):
	global red
	red = msg.data

def sub_green_callback(msg):
	global green
	green = msg.data

def sub_blue_callback(msg):
	global blue
	blue = msg.data
	
def sub_clear_callback(msg):
	global clear
	clear = msg.data

#init function-----------------------------------------------------------
def init_for_ros():
	
	rospy.init_node('PS4_Processing_Node')

	PS4_Message_Publisher = rospy.Publisher('Auto_Bot_PS4',ps4, queue_size=100)
	
	rospy.Subscriber('joy', Joy, handler)
	sub_red = rospy.Subscriber('red', Float32 , sub_red_callback)
	sub_green = rospy.Subscriber('green', Float32 , sub_green_callback)
	sub_blue = rospy.Subscriber('blue', Float32 , sub_blue_callback)
	sub_clear = rospy.Subscriber('clear', Float32 , sub_clear_callback)

def pub_at_50Hz_loop(event):

	global ps4_message ,  monitor 
	
	PS4_Message_Publisher.publish(ps4_msg)
	
	
#Main function----------------------------------------------------------------
if __name__ == '__main__':
	
	#Init ROS
	init_for_ros()
	
	#Init Monitor
	monitor_init()
	
	#Timer Interrupt for differnet freq. loop
	rospy.Timer(rospy.Duration(.2), pub_at_50Hz_loop)
	rospy.Timer(rospy.Duration(.3), write_sth_to_monitor)
	
	rospy.spin()


	

			
			
