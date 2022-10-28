#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, Twist
from math import cos

class SC649:

    def __init__(self):

        #receive data from topic "Error" of class "Pose2D"
        rospy.Subscriber('/Error', Pose2D, self.callback_error)
        #send data
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(10) #10Hz,0.1s
        self.Epos = 0.0
        self.Etheta = 0.0

    def callback_error(self, data):
        '''
        Get robot data
        '''
        self.Epos = data.x
        self.Etheta = data.theta

if __name__ == '__main__':
    rospy.init_node('Controller',anonymous=True)
    robot=SC649()
    K1= input("Enter K1 constt: ")
    print(f"Entered K1 = {K1}")
    K2=input("Enter K2 constt: ")
    print(f"Entered K2 = {K2}")
    rate = rospy.Rate(10) # 10hz / 0.1sec
    while not rospy.is_shutdown():
        Epos = robot.Epos
        Etheta = robot.Etheta
        v_lin = float(K1)*Epos*cos(Etheta)
        v_ang = -1*float(K2)*Etheta

        print(v_lin)
        print(v_ang)

        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        robot.pub_vel.publish(vel_msg)
        robot.r.sleep()