#!/usr/bin/python3

import rospy
from sc649_assign_q.msg import Landmark
from sc649_assign_q.msg import Trilateration
from sc649_assign_q.msg import Robot_Position
import numpy as np
import time

Robot_Position_x = 0
Robot_Position_y = 0

state_x1 = 0
state_y1 = 0
state_r1 = 0

state_x2 = 0
state_y2 = 0
state_r2 = 0

state_x3 = 0
state_y3 = 0
state_r3 = 0

def Trilateration_info(data):
	
	global state_x1 
	global state_y1 
	global state_r1 

	global state_x2  
	global state_y2 
	global state_r2 

	global state_x3 
	global state_y3 
	global state_r3 

	
	state_x1 = data.landmarkA.x
	state_y1 = data.landmarkA.y
	state_r1 = data.landmarkA.distance

	state_x2 = data.landmarkB.x
	state_y2 = data.landmarkB.y
	state_r2 = data.landmarkB.distance

	state_x3 = data.landmarkC.x
	state_y3 = data.landmarkC.y
	state_r3 = data.landmarkC.distance


def programflow():

	rospy.init_node('Robot_Position_Finder')
	rospy.Subscriber('/trilateration_data',Trilateration,Trilateration_info)

	rate = rospy.Rate(10) #10 Hz

	while not rospy.is_shutdown():

		A = (-2*state_x1 + 2*state_x2)
		B = (-2*state_y1 + 2*state_y2)
		C = (state_r1)**2 - (state_r2)**2 - (state_x1)**2 + (state_x2)**2 - (state_y1)**2 + (state_y2)**2 
		D = (-2*state_x2 + 2*state_x3)
		E = (-2*state_y2 + 2*state_y3)
		F = (state_r2)**2 - (state_r3)**2 - (state_x2)**2 + (state_x3)**2 - (state_y2)**2 + (state_y3)**2
		
		Robot_Position_x = (C*E - F*B)/(E*A - B*D + 0.000001)
		Robot_Position_y = (C*D - A*F)/(B*D - A*E + 0.000001) 
		#if E*A != B*D:	
			#Robot_Position_x = (C*E - F*B)/(E*A - B*D)
			#Robot_Position_y = (C*D - A*F)/(B*D - A*E)
		#else:
			#Robot_Position_x = (C*E - F*B)/(E*A - B*D + 0.000001)
			#Robot_Position_y = (C*D - A*F)/(B*D - A*E + 0.000001)
			

		Robot_Pose_msg = Robot_Position(Robot_Position_x,Robot_Position_y)

		PosePublisher = rospy.Publisher('/robot_pose',Robot_Position,queue_size=5)
		PosePublisher.publish(Robot_Pose_msg)

		# rospy.loginfo("Position_x: {:.2f}".format(Robot_Position_x))
		# rospy.loginfo("Position_y: {:.2f}".format(Robot_Position_y))

		rate.sleep()



if __name__ == '__main__':
	try:
		time.sleep(5)
		programflow()
	except rospy.ROSInterruptException:
		pass 
