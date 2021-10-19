#!/usr/bin/env python


#instructions:
#1. change pkg_name(default:auto_bot)
#2. put ps4.msg in msg folder
#3. create msg (follow ros tutorial)
#4. sample application functions at bottom(after Main Function)


import rospy

from sensor_msgs.msg import Joy

from auto_bot.msg import ps4
	#subject to change "package name" to match your project
from std_msgs.msg import Header 



#PS4 message type-----------------------------------------------------


ps4_out_message = ps4()



#PS4 message formation function---------------------------------------


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


	#DEBUG OUTPUT

	#rospy.loginfo(ps4_out_message)



#PS4 message publishing function-------------------------------------


def ps4_message_publishing(event):


	#Global Variable

	global ps4_message



	PS4_Message_Publisher.publish(ps4_out_message)



#Main function----------------------------------------------------------------


if __name__ == '__main__':


	#ROS Node Init

	rospy.init_node('PS4_Processing_Node')



	#ROS Publisher

	PS4_Message_Publisher = rospy.Publisher('Auto_Bot_PS4',ps4, queue_size=100)



	#ROS Subscriber

	rospy.Subscriber('joy', Joy, handler)



	#ROS Interrupt

	rospy.Timer(rospy.Duration(.05), ps4_message_publishing)


	
#ROS Loop

	rospy.spin()


#Sample application functions for using the messages in other .py---------------
if False:
	#copy from here onwards-------------------------------------------------
	#delete items that you do not need

	#initialize values for variables
	square = False
	cross = False
	circle = False
	triangle = False
	l1 = False
	r1 = False
	l2 = False
	r2 = False
	share = False
	option = False
	l3 = False
	r3 = False
	ps = False
	touch = False
	leftx = 100
	lefty = 100
	rightx = 100
	righty = 100
	analog_l2 = 0.0
	analog_r2 = 0.0
	left_right_pad = 0
	up_down_pad = 0

	#PS4 message handling function------------------------------------------
	def ps4_handler(ps4_income_message):

		#Global Variables
		global square, cross, circle, triangle, l1, l2, r1, r2, share, option, l3, r3, leftx, lefty, rightx, righty, analog_l2, analog_r2, ps, touch, left_right_pad, up_down_pad

		#Updating ps4 inputs
		square = ps4_income_message.square
		cross = ps4_income_message.cross
		circle = ps4_income_message.circle
		triangle = ps4_income_message.triangle
		l1 = ps4_income_message.l1
		r1 = ps4_income_message.r1
		l2 = ps4_income_message.l2
		r2 = ps4_income_message.r2
		share = ps4_income_message.share
		option = ps4_income_message.option
		l3 = ps4_income_message.l3
		r3 = ps4_income_message.r3
		ps = ps4_income_message.ps
		touch = ps4_income_message.touch
		leftx = float(ps4_income_message.leftx)
		lefty = float(ps4_income_message.lefty)
		rightx = float(ps4_income_message.rightx)
		righty = float(ps4_income_message.righty)
		analog_l2 = float(ps4_income_message.analog_l2)
		analog_r2 = float(ps4_income_message.analog_r2)
		left_right_pad = int(ps4_income_message.left_right_pad)
		up_down_pad = int(ps4_income_message.up_down_pad)

	#copy end from here