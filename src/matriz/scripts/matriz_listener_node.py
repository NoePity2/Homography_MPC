#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import casadi as opt
import numpy as np
from geometry_msgs.msg import  Twist

homo_matrix = Float64MultiArray()
def homograpy_callback(msg):
    global homo_matrix
    homo_matrix = msg


rospy.init_node("matriz_listener_node_py")
rospy.loginfo('Program has started') #Beginning terminal msg
rospy.logwarn("To start the movement, the simulation must be running\n\n")
rate = rospy.Rate(10) #Node frequency (Hz) 

rospy.Subscriber("/homograpy_numerical", Float64MultiArray,homograpy_callback)
optimal_controls_pub = rospy.Publisher("optimal_controls", Twist, queue_size=10)

optimal_cotrols = Twist()


while(not rospy.is_shutdown()):
    if(homo_matrix.data == []):
        continue
    rospy.loginfo("I listened %s",homo_matrix.data)
    optimal_cotrols.linear.x = homo_matrix.data[0]
    optimal_cotrols.linear.y = homo_matrix.data[1]
    optimal_cotrols.linear.z = homo_matrix.data[2]
    optimal_cotrols.angular.x = homo_matrix.data[3]
    optimal_cotrols.angular.y = homo_matrix.data[4]
    optimal_cotrols.angular.z = homo_matrix.data[5]
    optimal_controls_pub.publish(optimal_cotrols)
    rate.sleep()

