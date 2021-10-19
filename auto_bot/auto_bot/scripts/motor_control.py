#!/usr/bin/env python
#Author: Tim
#Modify to fit system: Ah Jo

import rospy
import math

from auto_bot.msg import chassic_speed #TODO modified pkg_name
from std_msgs.msg import Float32

chassic_speed_message = chassic_speed()
x_vector = 0.0
y_vector = 0.0

def sub_xv_callback(msg):
	global x_vector
	x_vector = msg.data

	#For debugging
	rospy.loginfo("xv: %f" % (msg.data))

def sub_yv_callback(msg):
	global y_vector
	y_vector = msg.data

	#For debugging
	rospy.loginfo("yv: %f" % (msg.data))

def move(x_vector, y_vector, r_vector, speedRatio):

	x_new = x_vector * math.cos(45*math.pi/180)
	y_new = y_vector * math.cos(45*math.pi/180)

	FLM = x_new + y_new + r_vector
	FRM = x_new - y_new + r_vector
	BLM = y_new - x_new + r_vector
	BRM = r_vector - x_new - y_new
    
	ref = max(max(abs(FLM),abs(FRM)),max(abs(BLM),abs(BRM)))

	if ref != 0.0:
		FLM /= ref
		FRM /= ref
		BLM /= ref
		BRM /= ref
    
	chassic_speed_message.motor0_power = 0.5 + FLM * 0.4 * speedRatio 
        chassic_speed_message.motor1_power = 0.5 + FRM * 0.4 * speedRatio
        chassic_speed_message.motor2_power = 0.5 + BLM * 0.4 * speedRatio
       	chassic_speed_message.motor3_power = 0.5 + BRM * 0.4 * speedRatio

	chassic_speed_pub.publish(chassic_speed_message)

if __name__ =="__main__": 

	rospy.init_node('motor_control')

	chassic_speed_pub = rospy.Publisher('Auto_Bot_Chassic_Control', chassic_speed, queue_size=100)

	x_vector_sub = rospy.Subscriber('x_vector', Float32, sub_xv_callback)
	y_vector_sub = rospy.Subscriber('y_vector', Float32, sub_yv_callback)

	rate = rospy.Rate(2)
	
	while not rospy.is_shutdown():
		move(x_vector, y_vector, 0.0, 1.0)
		rate.sleep()
