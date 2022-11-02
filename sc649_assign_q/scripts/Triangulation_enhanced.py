#!/usr/bin/python3

import rospy
from sc649_assign_q.msg import Measurement, Landmark, Robot_Position
from geometry_msgs.msg import Twist
from math import tan, radians
import numpy as np
import time
import matplotlib.pyplot as plt

#global initialisation
Robot_derived_Position_x = 0
Robot_derived_Position_y = 0
Robot_actual_Position_x = 0
Robot_actual_Position_y = 0
Robot_actual_Theta_deg = 0
x1 = 0
y1 = 0
alpha1_deg = 0
x2 = 0
y2 = 0
alpha2_deg = 0
x3 = 0
y3 = 0
alpha3_deg = 0


def Landmark_info(data):
	#landamark data read
	global x1 
	global y1 
	global alpha1_deg 

	global x2  
	global y2 
	global alpha2_deg 

	global x3 
	global y3 
	global alpha3_deg 

	# clockwise sequence is landmarkB - landmarkA - landmarkC, hence
	x2 = data.landmarkA.x
	y2 = data.landmarkA.y
	alpha2_deg = data.landmarkA.bearing

	x1 = data.landmarkB.x
	y1 = data.landmarkB.y
	alpha1_deg = data.landmarkB.bearing

	x3 = data.landmarkC.x
	y3 = data.landmarkC.y
	alpha3_deg = data.landmarkC.bearing


def Robot_actual_info(data):
    #robot position read
    global Robot_actual_Position_x
    global Robot_actual_Position_y
    global Robot_actual_Theta_deg

    Robot_actual_Position_x = data.Robot_position_x
    Robot_actual_Position_y = data.Robot_position_y
    Robot_actual_Theta_deg = data.Robot_orientation


def programflow():
    rospy.init_node('controller')
    rospy.Subscriber('robot_pose_data',Robot_Position,Robot_actual_info)
    rospy.Subscriber('measurement_data',Measurement, Landmark_info)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate=rospy.Rate(10)
    
    #local initialisation
    length=95
    ignore = 2
    Robot_actual_Position_x_arr = []
    Robot_actual_Position_y_arr = []
    Robot_actual_Theta_deg_arr = []  
    x1_arr = []
    y1_arr = []
    alpha1_deg_arr = []
    x2_arr = []
    y2_arr = []
    alpha2_deg_arr = []
    x3_arr = []
    y3_arr = []
    alpha3_deg_arr = []
    Robot_derived_Position_x_arr = []
    Robot_derived_Position_y_arr = []
    MSE = 0

    for x in range(length):
        if (alpha1_deg != alpha2_deg):

            #computing modified landmark coordinates
            x1_dash = x1-x2
            y1_dash = y1-y2
            x3_dash = x3-x2
            y3_dash = y3-y2

            #compute the three cot:
            T12 = 1/tan(radians(alpha2_deg - alpha1_deg))
            T23 = 1/tan(radians(alpha3_deg - alpha2_deg))
            T31 = (1 - (T12 * T23))/(T12+T23)

            #compute modified center coordinates
            x12_dash = x1_dash + (T12*y1_dash)
            y12_dash = y1_dash - (T12*x1_dash)
            x23_dash = x3_dash - (T23*y3_dash)
            y23_dash = y3_dash + (T23*x3_dash)
            x31_dash = (x3_dash+x1_dash) + (T31*(y3_dash-y1_dash))
            y31_dash = (y3_dash+y1_dash) - (T31*(x3_dash-x1_dash))

            #compute K31_dash
            k31_dash = (x1_dash*x3_dash) + (y1_dash*y3_dash) + (T31*((x1_dash*y3_dash)-(x3_dash*y1_dash)))

            #compute D
            D = ((x12_dash - x23_dash)*(y23_dash - y31_dash)) - ((y12_dash - y23_dash)*(x23_dash - x31_dash))

            #compute robot posiiton {xr,yr}
            if (D != 0.0):
                Robot_derived_Position_x = x2 + (k31_dash*(y12_dash-y23_dash)/D)
                Robot_derived_Position_y = y2 + (k31_dash*(x23_dash-x12_dash)/D)

        else:
            Robot_derived_Position_x = 0
            Robot_derived_Position_y = 0

        #commands to robot
        v_lin = 1
        v_ang = 1
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)

        #aggreagting data into arrays
        if(x>ignore-1):
            Robot_actual_Position_x_arr.append(Robot_actual_Position_x)
            Robot_actual_Position_y_arr.append(Robot_actual_Position_y)
            Robot_actual_Theta_deg_arr.append(Robot_actual_Theta_deg)
            Robot_derived_Position_x_arr.append(Robot_derived_Position_x)
            Robot_derived_Position_y_arr.append(Robot_derived_Position_y)
            x1_arr.append(x1)
            y1_arr.append(y1)
            alpha1_deg_arr.append(alpha1_deg)
            x2_arr.append(x2)
            y2_arr.append(y2)
            alpha2_deg_arr.append(alpha2_deg)
            x3_arr.append(x3)
            y3_arr.append(y3)
            alpha3_deg_arr.append(alpha3_deg)
            MSE = MSE + ((Robot_actual_Position_x - Robot_derived_Position_x)**2) + ((Robot_actual_Position_y - Robot_derived_Position_y)**2 )
        
        print("robot pos x = " + str(Robot_actual_Position_x))
        print("robot pos y = " + str(Robot_actual_Position_y))
        print("derived pos x = " + str(Robot_derived_Position_x))
        print("derived pos y = " + str(Robot_derived_Position_y))
        rate.sleep()
    
    out = np.c_[x1_arr,y1_arr,alpha1_deg_arr,x2_arr,y2_arr,alpha2_deg_arr,x3_arr,y3_arr,alpha3_deg_arr,Robot_actual_Position_x_arr,Robot_actual_Position_y_arr,Robot_actual_Theta_deg_arr,Robot_derived_Position_x_arr,Robot_derived_Position_y_arr]
    
    #calculating mean square error
    MSEN = MSE/(length - ignore)
    MSE_text = "Mean Square Error = " + f'{MSEN:.4f}'
    print(MSE_text)

    #plotting
    plt.plot(Robot_actual_Position_x_arr, Robot_actual_Position_y_arr,color='r',label='actual pos')
    plt.plot(Robot_derived_Position_x_arr, Robot_derived_Position_y_arr,color='b', label='derived pos')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Derived Position vs Actual position")
    plt.text(0, 0, MSE_text, fontsize=12, bbox=dict(facecolor='red', alpha=0.5))
    plt.legend()
    plt.savefig('positions_enhanced.png')
    plt.show()

    #logging data to file
    mat=np.matrix(out)
    with open('positions_enhanced.txt','w+') as f:
        f.write("x1 y1 alpha1_deg x2 y2 alpha2_deg x3 y3 alpha3_deg robot_pos_x robot_pos_y robot_theta_deg derived_pos_x derived_pos_y\n")
        for line in mat:
            np.savetxt(f,line,fmt='%.3f')
    f.close()


if __name__ == '__main__':
	try:
		time.sleep(5)
		programflow()
	except rospy.ROSInterruptException:
		pass 
