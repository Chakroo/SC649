#!/usr/bin/env python

from cmath import sqrt
from math import atan
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

class SC649:

    def __init__(self):

        #receive data
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.bot_pose = [4.0, 0.0] #intialising to 4 to get correct results
        self.bot_th = 0.0
        #send data
        self.pub_vel = rospy.Publisher('/Error',Pose2D,queue_size=10)
        self.r = rospy.Rate(10) #10Hz,0.1s


    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.bot_pose[0] = data.pose.pose.position.x
        self.bot_pose[1] = data.pose.pose.position.y
        self.bot_th = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

if __name__ == '__main__':
    rospy.init_node('Error_Generator',anonymous=True)
    robot = SC649()
    waypointdata = np.loadtxt("waypoints.txt", skiprows=1)
    length = len(waypointdata)
    Epos_arr = []
    Etheta_arr = []
    time_arr = []
    x_target_arr = []
    y_target_arr = []
    x_current_arr = []
    y_current_arr = []
    theta_current_arr =[]
    for x in range(length):
        print(robot.bot_pose[0])
        print(robot.bot_pose[1])
        #generate errors
        time1=waypointdata[x,0]
        x_target=waypointdata[x,1]
        y_target=waypointdata[x,2]
        x_current=robot.bot_pose[0]
        y_current=robot.bot_pose[1]
        theta_current=robot.bot_th
        Epos = abs(sqrt((x_target-x_current)**2 + (y_target-y_current)**2))
        ratio = (y_current-y_target)/(x_current-x_target)
        Etheta =  atan(ratio)
        
        #publish to topic "Error" with message of custom message type "Pose2D" from node named "Error_Generator"
        #pub=rospy.Publisher('/Error',Pose2D,queue_size=10)
        err_msg = Pose2D()
        err_msg.x = Epos
        err_msg.theta = Etheta
        robot.pub_vel.publish(err_msg)

        Epos_arr.append(Epos)
        Etheta_arr.append(Etheta)
        time_arr.append(time1)
        x_target_arr.append(x_target)
        y_target_arr.append(y_target)
        x_current_arr.append(x_current)
        y_current_arr.append(y_current)
        theta_current_arr.append(theta_current)
        robot.r.sleep()
    
    out = np.c_[time_arr,x_target_arr, y_target_arr,x_current_arr, y_current_arr, theta_current_arr, Epos_arr,Etheta_arr]
    print(out)

    plt.subplot(221)
    plt.plot(time_arr, Epos_arr)
    plt.xlabel('time')
    plt.ylabel('Position')
    plt.title("Position error")

    plt.subplot(222)
    plt.plot(time_arr, Etheta_arr)
    plt.xlabel('time')
    plt.ylabel('angle in degrees')
    plt.title("Theta error")

    plt.subplot(223)
    plt.plot(x_current_arr, y_current_arr)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Followed trajectory")

    plt.subplot(224)
    plt.plot(x_target_arr, y_target_arr)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Target trajectory")

    plt.savefig('errors.png')
    plt.show()

    mat=np.matrix(out)

    with open('errors.txt','w+') as f:
        f.write("t target_x target_y bot_pos_x bot_pos_y bot_th Epos Etheta\n")
        for line in mat:
            np.savetxt(f,line,fmt='%.3f')
    f.close()