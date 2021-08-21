#! /usr/bin/env python

# This node is used for controlling the behaviour of the robot.
# There are four functions, one for every possible modality of the robot, one as
# the callback for the /odom subscriber and a function for updating the state when possible.

import rospy
import time
import math
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from final_assignment.srv import *
from geometry_msgs.msg import Twist

pos_x = 0
pos_y = 0
x = 0 
y = 0
state = 5
cond = True
cond2= True
cond3 = True

# Callback for the subscriber to the /odom topic, used for
# retreiving the current robot position.
def odom_clbk(position):
	global pos_x, pos_y
	pos_x = position.pose.pose.position.x
	pos_y = position.pose.pose.position.y

# Function for updating the state: it takes an input from the user and, if the
# input in one of the possible 4, it sets the parameter 'state' and calls the 
# respective function.
# 1-> random position, 2-> manual position, 3-> wall follow mode, 4-> stop
def update_state():
	global cond, state
	time.sleep(2)
	print ("Change modality: ")
	
	state = float(input(""))
	rospy.set_param('state', state)
	if state == 1:
		randompos()	  
	elif state == 2:
		manual()
	elif state == 3:
		wall_foll()
	elif state == 4:	
		stay()
	else:
		print("invalid input")
					
# Function for random position, it calls the random position service with the min and max number,
# takes the responde and puts it in two variables x and y as the target position and publish them to
# the move_base/goal topic. ALso this function prints, every 4 seconds, the distance from the target.
# When the target is reached the update_state() function is called to ask the user a new modality. 
def randompos():
	global x, y, cli, pub, cli2, pos_x, pos_y
	resp = cli2(False)	
	targ = MoveBaseActionGoal()
	time.sleep(1)
	print("random mode selected")
	resp = cli (1, 6)
	x = resp.x
	y = resp.y
	
	targ.goal.target_pose.pose.position.x = x
	targ.goal.target_pose.pose.position.y = y
	targ.goal.target_pose.header.frame_id = "map"
	targ.goal.target_pose.pose.orientation.w = 1
	pub.publish(targ)
	print("random target chosen: (%s,%s)" %(x,y))
	cont = 1
	while (cont == 1):
		time.sleep(4)
		dist_x = (pos_x - x)
		dist_y = (pos_y - y)
		print ("distance from target x: %s\ndistance from target y: %s\n" %(dist_x,dist_y))
		if (dist_x <= 0.5 and dist_y <= 0.5) and (dist_x>= -0.5 and dist_y>= -0.5):
			print ("Target reached")
			update_state()
			cont = 0
	return []
		
		
# Function for choosing a manual positon. It asks the user a target between the six possible options.
# Once the input is correctly chosen the target is published on the move_base/goal topic.
# Every 4 seconds the distance from the target is printed and when the target is reached the 
# update_state() is called to ask the user a new modality.	
def manual():
	global pub, x, y, pos_x, pos_y, cond, cond2, cond3, cli2
	resp = cli2(False)	
	targ = MoveBaseActionGoal()
	time.sleep(1)
	cond2 = True
	cond3 = True
	print("Manual target selected, insert one between these possible targets:\n(-4,-3) (-4,2) (-4,7)\n(5,-7) (5,-3) (5,1)")
	x = float(input('x :'))
	while (cond2):
		if x!=-4 and x!=5:
			print("Invalid x value, try again")
			x = float(input('x :'))	
		else:
			cond2 = False
			y = float(input('y :'))
			while(cond3):
				if (x==-4 and (y==-3 or y==2 or y==7)) or (x==5 and (y==-7 or y==-3 or y==1)):
					cond3 = False
				else:
					print("Invalid y value, try again")
					y = float(input('y :'))
					
	targ.goal.target_pose.pose.position.x = x
	targ.goal.target_pose.pose.position.y = y
	targ.goal.target_pose.header.frame_id = "map"
	targ.goal.target_pose.pose.orientation.w = 1
	print("Thanks! Let's reach the next position")
	pub.publish(targ)

	cont = 1
	while (cont == 1):
		time.sleep(4)
		dist_x = (pos_x - x)
		dist_y = (pos_y - y)
		print ("distance from target x: %s\ndistance from target y: %s\n" %(dist_x,dist_y))
		if (dist_x <= 0.5 and dist_y <= 0.5) and (dist_x>= -0.5 and dist_y>= -0.5):
			print ("Target reached")
			update_state()
			cont = 0
	return []
	
# The wall_foll() function calls the wall_follower_switch and the wall_follow_service node,
# putting at True the request, for making the robot following the walls. During this action the update_state() 
#is called and the user can change the modality at every moment.
def wall_foll():
	global cli2
	time.sleep(1)
	print ("Wall follow mode selected")
	resp = cli2(True)
	update_state()
	
# This function is used to stop the robot in the position. It can be called after the
# "target reached" in the manual and random modality or at everty moment during the wall_follow mode.
# It puts at False the request for the wall_follow service and puts the linear and angular
# velocity at 0. Also in this case the update_state() is called and the modality can be changed
# at every moment.
def stay():
	global cli2, pub2, pos_x, pos_y
	time.sleep(1)
	print("stop selected\nthe robot will stay in position: (%s,%s)" %(pos_x, pos_y))
	resp = cli2(False)
	vel = Twist()
	vel.linear.x = 0.0
	vel.linear.y = 0.0
	vel.angular.z = 0.0
	pub2.publish(vel)
	
	time.sleep(1)
	update_state()
		
	
# In the main function are initialized the subsciber for the /odom topic, publishers for
# move_base/goal and cmd_vel, and clients for the random position and wall follower service.
# It takes the first state from the parameter 'state' and depending on it calls the corresponding
# function.
def main():
	global pub, pub2, state, cli, cli2, cond
	rospy.init_node('controller')
	sub = rospy.Subscriber('/odom', Odometry, odom_clbk)
	pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)	
	pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	cli = rospy.ServiceProxy('random_pos', random_targ)
	cli2 = rospy.ServiceProxy('/wall_follower_switch', SetBool)
	
	while cond == True:
		state = rospy.get_param('state') 
		if state == 1 or state == 2 or state == 3 or state == 4:
			cond = False
			
	if state == 1:
		randompos()	  
	elif state == 2:
		manual()
	elif state == 3:
		wall_foll()
	elif state == 4:	
		stay()
		
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()
		



if __name__ == '__main__':
    main()
