#! /usr/bin/env python

# This file contains the server for calculating a random number.
# It takes as a request two values, min and max (1 and 6 for this project)
# and computes a random integer number in between them using the function random.randint()


from final_assignment.srv import *
import rospy
import math
import random

# In this function the random number is computed. 
# depending on the output number one of the six possible targets is selected and returned:
# 1-> (-4,3), 2-> (-4,2), 3-> (-4,7), 4-> (5,-7), 5-> (5,-3), 6-> (5,1)
def myrandom(req):
	resp = random_targResponse()
	out = random.randint(req.min, req.max)
	if out == 1:
		resp.x = -4
		resp.y = -3
	elif out == 2:
		resp.x = -4
		resp.y = 2
	elif out == 3:
		resp.x = -4
		resp.y = 7
	elif out == 4:
		resp.x = 5
		resp.y = -7
	elif out == 5:
		resp.x = 5
		resp.y = -3
	elif out == 6:
		resp.x = 5
		resp.y = 1
	return resp
	
	
# In the main function the server is initialized:
# the service name is 'random_pos' and its a server of type random_targ
def main():

	rospy.init_node('random_serv')
	serv = rospy.Service('random_pos', random_targ, myrandom)
	rospy.spin()

if __name__ == '__main__':
    main()
