#!/usr/bin/env python
#Author: Tim
#Modify to fit system: Ah Jo

#This program is for moving the robot to position(x,y)
#Implemented without using action

#TODO 1) check msg datatype
#     2) implement with action (Optional)

import rospy
from std_msgs.msg import Float32

cur_x_coord = 0.0
cur_y_coord = 0.0

destination = [0.0, 0.0]

def sub_x_callback(msg):
	global cur_x_coord
	cur_x_coord = msg.data

	#For debugging
	rospy.loginfo("x: %f" % (msg.data))

	x_vector = destination[0] - cur_x_coord

	pub_x_vector.publish(x_vector)

def sub_y_callback(msg):
	global cur_y_coord
	cur_y_coord = msg.data

	#For debugging
	rospy.loginfo("y: %f" % (msg.data))

	y_vector = destination[1] - cur_y_coord

	pub_y_vector.publish(y_vector)

def goto_position(x_coord, y_coord):
	global destination
	x_coord = float(x_coord)
	y_coord = float(y_coord)
	destination[0] = x_coord
	destination[1] = y_coord		

if __name__ =="__main__": 

	rospy.init_node('GoToPosition')
	
	goto_position(10, 0)

	pub_x_vector = rospy.Publisher('x_vector', Float32, queue_size=100)

	pub_y_vector = rospy.Publisher('y_vector', Float32, queue_size=100)

	sub_x_coord = rospy.Subscriber('Xaxis', Float32, sub_x_callback)

	sub_y_coord = rospy.Subscriber('Yaxis', Float32, sub_y_callback)

	rospy.spin()





