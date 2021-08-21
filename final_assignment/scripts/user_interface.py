#! /usr/bin/env python

# This file contains the user interface for the project.
# The user interface contains a subscriber on the /odom topic for retreiving the robot position.
# When the node is started the funcion select_mod() is called to make the user select the
# modality of the robot.

import rospy
import time
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

condition = False
cond = True
# service callback

x = 0
y = 0

# Callback for the Odometry subscriber, it retreives the current position of
# the robot and prints it.
def odom_clbk(position):
	global x, y, pub, condition
	if not condition:
		x = position.pose.pose.position.x
		y = position.pose.pose.position.y
		print("\nRobot is in position: x = " + str(x) + ", y = " + str(y))
		condition = True

# In this function is requested to the user to insert a number from 1 to 4 for deciding the modality.
# Once the input is inserted correctly, the parameter 'state' is set with the modality chosen
def select_mod():
	global cond
	time.sleep(1)
	print("Choose a modality\n1: random target\n2: manual target\n3: follow walls\n4: stop in last position")
	mod = float(input("modality: "))
	while(cond):
		if mod!= 1 and mod!=2 and mod!=3 and mod!=4:
			print("invalid input, try again")
			mod = float(input("modality: "))
		else:
			cond = False
			rospy.set_param('state', mod)	
	return []

# In the main function are initialized the node and the subscriber to the /odom topic
def main():
	global pub
	rospy.init_node('user_interface')
    
	sub = rospy.Subscriber('/odom', Odometry, odom_clbk)
	
	time.sleep(1)
	select_mod()
		
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == '__main__':
    main()
