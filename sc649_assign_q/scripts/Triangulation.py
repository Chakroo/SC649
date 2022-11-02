#!/usr/bin/python3

import rospy
from sc649_assign_q.msg import Measurement, Landmark, Robot_Position
from geometry_msgs.msg import Twist
from math import tan, radians
import numpy as np
import time
import matplotlib.pyplot as plt

#global Inititalisation
Robot_derived_landAB_Position_x = 0
Robot_derived_landAB_Position_y = 0
Robot_derived_landBC_Position_x = 0
Robot_derived_landBC_Position_y = 0
Robot_derived_landCA_Position_x = 0
Robot_derived_landCA_Position_y = 0
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
	#landmark data read
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
    #robot position data read
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
    length = 95
    ignore = 2
    MSE_AB=0
    MSE_BC=0
    MSE_CA=0
    Robot_actual_Position_x_arr = []
    Robot_actual_Position_y_arr = []
    Robot_actual_Theta_deg_arr = []
    Robot_derived_landAB_Position_x_arr = []
    Robot_derived_landAB_Position_y_arr = []
    Robot_derived_landBC_Position_x_arr = []
    Robot_derived_landBC_Position_y_arr = []
    Robot_derived_landCA_Position_x_arr = []
    Robot_derived_landCA_Position_y_arr = []
    x1_arr = []
    y1_arr = []
    alpha1_deg_arr = []
    x2_arr = []
    y2_arr = []
    alpha2_deg_arr = []
    x3_arr = []
    y3_arr = []
    alpha3_deg_arr = []


    for x in range(length):
        T1AB=tan(radians(alpha1_deg)+radians(Robot_actual_Theta_deg))
        T2AB=tan(radians(alpha2_deg)+radians(Robot_actual_Theta_deg))
        if (T1AB != T2AB):
            Robot_derived_landAB_Position_x = (y1 - y2 - x1*T1AB + x2*T2AB)/(T2AB - T1AB)
            Robot_derived_landAB_Position_y = ((x2 - x1)*T1AB*T2AB + y1*T2AB - y2*T1AB)/(T2AB - T1AB)
        else:
            Robot_derived_landAB_Position_x = 0
            Robot_derived_landAB_Position_y = 0

        T1CA=tan(radians(alpha2_deg)+radians(Robot_actual_Theta_deg))
        T2CA=tan(radians(alpha3_deg)+radians(Robot_actual_Theta_deg))
        if (T1CA!=T2CA):
            Robot_derived_landCA_Position_x = (y2 - y3 - x2*T1CA + x3*T2CA)/(T2CA - T1CA)
            Robot_derived_landCA_Position_y = ((x3 - x2)*T1CA*T2CA + y2*T2CA - y3*T1CA)/(T2CA - T1CA)
        else:
            Robot_derived_landCA_Position_x = 0
            Robot_derived_landCA_Position_y = 0

        T1BC=tan(radians(alpha3_deg)+radians(Robot_actual_Theta_deg))
        T2BC=tan(radians(alpha1_deg)+radians(Robot_actual_Theta_deg))
        if(T1BC!=T2BC):
            Robot_derived_landBC_Position_x=(y3-y1-x3*T1BC+x1*T2BC)/(T2BC-T1BC)
            Robot_derived_landBC_Position_y=((x1-x3)*T1BC*T2BC+y3*T2BC-y1*T1BC)/(T2BC-T1BC)
        else:
            Robot_derived_landBC_Position_x=0
            Robot_derived_landBC_Position_y=0

        #command to the robot
        v_lin = 1
        v_ang = 1
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)

        #aggregating data into arrays
        if(x>ignore-1):
            Robot_actual_Position_x_arr.append(Robot_actual_Position_x)
            Robot_actual_Position_y_arr.append(Robot_actual_Position_y)
            Robot_actual_Theta_deg_arr.append(Robot_actual_Theta_deg)
            Robot_derived_landAB_Position_x_arr.append(Robot_derived_landAB_Position_x)
            Robot_derived_landAB_Position_y_arr.append(Robot_derived_landAB_Position_y)
            Robot_derived_landBC_Position_x_arr.append(Robot_derived_landBC_Position_x)
            Robot_derived_landBC_Position_y_arr.append(Robot_derived_landBC_Position_y)
            Robot_derived_landCA_Position_x_arr.append(Robot_derived_landCA_Position_x)
            Robot_derived_landCA_Position_y_arr.append(Robot_derived_landCA_Position_y)
            x1_arr.append(x1)
            y1_arr.append(y1)
            alpha1_deg_arr.append(alpha1_deg)
            x2_arr.append(x2)
            y2_arr.append(y2)
            alpha2_deg_arr.append(alpha2_deg)
            x3_arr.append(x3)
            y3_arr.append(y3)
            alpha3_deg_arr.append(alpha3_deg)
            MSE_AB = MSE_AB + ((Robot_actual_Position_x - Robot_derived_landAB_Position_x)**2) + ((Robot_actual_Position_y - Robot_derived_landAB_Position_y)**2 )
            MSE_BC = MSE_BC + ((Robot_actual_Position_x - Robot_derived_landBC_Position_x)**2) + ((Robot_actual_Position_y - Robot_derived_landBC_Position_y)**2 )
            MSE_CA = MSE_CA + ((Robot_actual_Position_x - Robot_derived_landCA_Position_x)**2) + ((Robot_actual_Position_y - Robot_derived_landCA_Position_y)**2 )

        print("robot pos x = " + str(Robot_actual_Position_x))
        print("robot pos y = " + str(Robot_actual_Position_y))
        print("landAB derived pos x = " + str(Robot_derived_landAB_Position_x))
        print("landAB derived pos y = " + str(Robot_derived_landAB_Position_y))
        print("landBC derived pos x = " + str(Robot_derived_landBC_Position_x))
        print("landBC derived pos y = " + str(Robot_derived_landBC_Position_y))
        print("landCA derived pos x = " + str(Robot_derived_landCA_Position_x))
        print("landCA derived pos y = " + str(Robot_derived_landCA_Position_y))
 
        rate.sleep()
    
    out = np.c_[x1_arr,y1_arr,alpha1_deg_arr,x2_arr,y2_arr,alpha2_deg_arr,x3_arr,y3_arr,alpha3_deg_arr,Robot_actual_Position_x_arr,Robot_actual_Position_y_arr,Robot_actual_Theta_deg_arr,Robot_derived_landAB_Position_x_arr,Robot_derived_landAB_Position_y_arr,Robot_derived_landBC_Position_x_arr,Robot_derived_landBC_Position_y_arr,Robot_derived_landCA_Position_x_arr,Robot_derived_landCA_Position_y_arr]

    #calculating MSE for each combination
    MSEN_AB = MSE_AB/(length - ignore)
    MSE_AB_text = "Mean Square Error = " + f'{MSEN_AB:.4f}'
    print(MSE_AB_text)

    MSEN_BC = MSE_BC/(length - ignore)
    MSE_BC_text = "Mean Square Error = " + f'{MSEN_BC:.4f}'
    print(MSE_BC_text)
    
    MSEN_CA = MSE_CA/(length - ignore)
    MSE_CA_text = "Mean Square Error = " + f'{MSEN_CA:.4f}'
    print(MSE_CA_text)

    #plotting
    plt.plot(Robot_actual_Position_x_arr, Robot_actual_Position_y_arr,color='r',label='actual pos')
    plt.plot(Robot_derived_landAB_Position_x_arr, Robot_derived_landAB_Position_y_arr,color='b', label='derived pos')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Derived Position_landmark AB vs Actual position")
    yA=(plt.gca().get_ylim()[1])+0.35
    xA=((plt.gca().get_xlim()[0]) + (plt.gca().get_xlim()[1]))/2
    plt.text(xA, yA, MSE_AB_text, fontsize=12, bbox=dict(facecolor='red', alpha=0.5))
    plt.legend()
    plt.savefig('positions_AB.png')
    plt.show()

    plt.plot(Robot_actual_Position_x_arr, Robot_actual_Position_y_arr,color='r',label='actual pos')
    plt.plot(Robot_derived_landBC_Position_x_arr, Robot_derived_landBC_Position_y_arr,color='b', label='derived pos')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Derived Position_landmark BC vs Actual position")
    yB=(plt.gca().get_ylim()[1])+0.35
    xB=((plt.gca().get_xlim()[0]) + (plt.gca().get_xlim()[1]))/2
    plt.text(xB, yB, MSE_BC_text, fontsize=12, bbox=dict(facecolor='red', alpha=0.5))
    plt.legend()
    plt.savefig('positions_BC.png')
    plt.show()

    plt.plot(Robot_actual_Position_x_arr, Robot_actual_Position_y_arr,color='r',label='actual pos')
    plt.plot(Robot_derived_landCA_Position_x_arr, Robot_derived_landCA_Position_y_arr,color='b', label='derived pos')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Derived Position_landmark CA vs Actual position")
    yC=(plt.gca().get_ylim()[1])+0.35
    xC=((plt.gca().get_xlim()[0]) + (plt.gca().get_xlim()[1]))/2
    plt.text(xC, yC, MSE_CA_text, fontsize=12, bbox=dict(facecolor='red', alpha=0.5))
    plt.legend()
    plt.savefig('positions_CA.png')
    plt.show()

    mat=np.matrix(out)
    with open('positions.txt','w+') as f:
        f.write("x1 y1 alpha1_deg x2 y2 alpha2_deg x3 y3 alpha3_deg robot_pos_x robot_pos_y robot_theta_deg landAB_derive_pos_x landAB_derive_pos_y landBC_derive_pos_x landBC_derive_pos_y landCA_derive_pos_x landCA_derive_pos_y\n")
        for line in mat:
            np.savetxt(f,line,fmt='%.3f')
    f.close()


if __name__ == '__main__':
	try:
		time.sleep(5)
		programflow()
        
	except rospy.ROSInterruptException:
		pass 
