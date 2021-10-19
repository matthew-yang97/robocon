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


#gaming properties ---------------------------------------

#array for storing pressed pattern
pattern = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 ]
place = 0
success = False
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

count = 0.0

period = 0.2
ros_period = 0.02
// whichone=0;  set a value to decide which route is chose.
mode = True #mode true is spinner , mode false is arm


cur_deg = 0.0
cur_rpm = 0.0
tar_rpm = 0.0
tar_deg = 0.0
do_once = False
reset = False
stop_motor = False
enable = False
release_grab = False
    
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

	#ps4_msg.touch = data.buttons[10]

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

	check_button()
	write_pattern_array()
	
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

	global monitor , tar_rpm , tar_deg , do_once , mode , cur_rpm , cur_deg , success

	if monitor.writable():

		if(mode == True): #true is spinner
			
			monitor.write("DS64(2,2,'Spinner Bot',2);")
		
		if(mode == False):
		
			monitor.write("DS64(2,2,'Arm Bot         ',2);")
		
		monitor.write("DS64(2,75,'tar_rpm: %s   ',2);"%tar_rpm)
		#monitor.write("DS64(300,2,'LY: %s ',2);"%ps4_msg.lefty)
		
		monitor.write("DS64(425,75,'tar_deg: %s   ',2);"%tar_deg)
		
		monitor.write("DS64(2,140,'cur_deg: %s    ',2);"%cur_deg)
		
		monitor.write("DS64(2,200,'do_once: %s     ',2);"%do_once)
		
		monitor.write("DS64(2,260,'stop motor: %s     ',2);"%stop_motor)
		
		monitor.write("DS64(2,320,'reset: %s     ',2);"%reset)
		
		monitor.write("DS64(2,400,'%s   %s',2);\r\n"%(pattern ,place))
		


	
def pub_at_50Hz_loop(event):

	global count , ros_period , ps4_message ,  monitor , tar_rpm , tar_deg
	
	PS4_Message_Publisher.publish(ps4_msg)
	pub_tar_rpm.publish(tar_rpm)
	pub_tar_deg.publish(tar_deg)
	
	#check pattern here--------------------
	check_pattern()
	#write_sth_to_monitor()


def check_button():

	global count , tar_rpm, tar_deg , do_once , stop_motor , reset , mode , enable
	global pre_up , pre_down , pre_left, pre_right , pre_square , pre_circle , pre_cross , pre_triangle , pre_l1 , pre_l2, pre_l3 , pre_r1 , pre_r2 , pre_r3 , pre_share , pre_option , pre_ps	
	
	#for tar_rpm ( up down )-------------------------------------
	if( ps4_msg.l1 == False and pre_up == False and ps4_msg.up == True):
		tar_rpm = tar_rpm + 1
	if( ps4_msg.l1 == False  and pre_down == False and ps4_msg.down == True):
		tar_rpm = tar_rpm - 1
	
	if( ps4_msg.l1 == True and pre_up == False and ps4_msg.up == True):
		tar_rpm = tar_rpm + 10
	if( ps4_msg.l1 == True and pre_down == False and ps4_msg.down == True):
		tar_rpm = tar_rpm - 10

	#for tar_deg ( lefy right )---------------------------------
	if( ps4_msg.l1 == False and pre_left == False and ps4_msg.left == True):
		tar_deg = tar_deg - 1
	if( ps4_msg.l1 == False  and pre_right == False and ps4_msg.right == True):
		tar_deg = tar_deg + 1
	
	if( ps4_msg.l1 == True and pre_right == False and ps4_msg.right == True):
		tar_deg = tar_deg + 10
	if( ps4_msg.l1 == True and pre_left == False and ps4_msg.left == True):
		tar_deg = tar_deg - 10
	
	#switch mode : true = spinner , false = arm ( L + R + x)------------------
	
	if( ps4_msg.l1 == True and ps4_msg.r1 == True and pre_cross == False and ps4_msg.cross == True):
		mode = not mode	

	#do_once (circle)----------------------------------------
	if(pre_circle == False and ps4_msg.circle == True):
		do_once = True
		pub_do_once.publish(True)
	
	if(pre_circle == True and ps4_msg.circle == False):
		do_once = False

	#stop_motor (square)----------------------------------------
	if(pre_square == False and ps4_msg.square == True):
		stop_motor = True
		pub_stop_motor.publish(True)
	
	if(pre_square == True and ps4_msg.square == False):
		stop_motor = False

	#release_grab (triangle)----------------------------------------
	if(pre_triangle == False and ps4_msg.triangle == True):
		pub_release_grab.publish(True)

	#enable/disable (r1 button)----------------------------------------
	if(pre_r1 == False and ps4_msg.r1 == True):
		enable = not enable 
		pub_enable.publish(enable)

		
	#reset (ps button)----------------------------------------
	if(pre_ps == False and ps4_msg.ps == True):
		reset = True
		pub_reset.publish(True)
	
	if(pre_ps == True and ps4_msg.ps == False):
		reset = False		


def write_pattern_array():

	global pattern , place

	if(place > 8):
		place = 0
		pattern = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 ]

	if(pre_up == False and ps4_msg.up == True):
		pattern[place] = 0
		place = place + 1

	if(pre_down == False and ps4_msg.down == True):
		pattern[place] = 1
		place = place + 1

	if(pre_left == False and ps4_msg.left == True):
		pattern[place] = 2
		place = place + 1		
		
	if(pre_right == False and ps4_msg.right == True):
		pattern[place] = 3
		place = place + 1		

	if(pre_triangle == False and ps4_msg.triangle == True):
		pattern[place] = 4
		place = place + 1		

	if(pre_cross == False and ps4_msg.cross == True):
		pattern[place] = 5
		place = place + 1		

	if(pre_square == False and ps4_msg.square == True):
		pattern[place] = 6
		place = place + 1		

	if(pre_circle == False and ps4_msg.circle == True):
		pattern[place] = 7
		place = place + 1		

	if(pre_l1 == False and ps4_msg.l1 == True):
		pattern[place] = 8
		place = place + 1		

	if(pre_r1 == False and ps4_msg.r1 == True):
		pattern[place] = 9
		place = place + 1		

	if(pre_l2 == False and ps4_msg.l2 == True):
		pattern[place] = 10
		place = place + 1		

	if(pre_r2 == False and ps4_msg.r2 == True):
		pattern[place] = 11
		place = place + 1		

	if(pre_l3 == False and ps4_msg.l3 == True):
		pattern[place] = 12
		place = place + 1

	if(pre_r3 == False and ps4_msg.r3 == True):
		pattern[place] = 13
		place = place + 1	
		
	
		
	
		

def check_pattern():
	global pattern , place
	
	# check for "up down left right square square L1 R1" pattern
	if( place > 6 and pattern[place-1] == 9 and pattern[place - 2] == 8 and pattern[place - 3] == 6 and pattern[place - 4] == 6 and pattern[place - 5] == 3 and pattern[place - 6] == 2 and pattern[place - 7] == 1 and pattern[place - 8] == 0):
		
		pub_release_grab.publish(True)
		success = True
		pattern = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
		place = 0
		
#-----------------------------------------------

def sub_cur_deg_callback(msg):
	global cur_deg
	cur_deg = msg.data

#Main function----------------------------------------------------------------
if __name__ == '__main__':
	

	rospy.init_node('PS4_Processing_Node')

	rospy.Subscriber('joy', Joy, handler)

	PS4_Message_Publisher = rospy.Publisher('Auto_Bot_PS4',ps4, queue_size=100)
	pub_tar_rpm = rospy.Publisher('tar_rpm', Float32, queue_size = 100)
	pub_tar_deg = rospy.Publisher('tar_deg', Float32, queue_size = 100)
	pub_do_once = rospy.Publisher('do_once', Bool, queue_size = 100)
	pub_reset = rospy.Publisher('reset', Bool, queue_size = 100)
	pub_enable = rospy.Publisher('enable', Bool, queue_size = 100)
	pub_release_grab = rospy.Publisher('release_grab', Bool, queue_size = 100)
	pub_stop_motor = rospy.Publisher('stop_motor', Bool, queue_size = 100)
	
	sub_cur_deg = rospy.Subscriber('cur_deg', Float32 , sub_cur_deg_callback)
	
	#Init Monitor
	monitor_init()
	
	rospy.Timer(rospy.Duration(.2), pub_at_50Hz_loop)
	rospy.Timer(rospy.Duration(.3), write_sth_to_monitor)
	#rospy.Timer(rospy.Duration(.2), write_pattern_array)
	

	rospy.spin()
	
