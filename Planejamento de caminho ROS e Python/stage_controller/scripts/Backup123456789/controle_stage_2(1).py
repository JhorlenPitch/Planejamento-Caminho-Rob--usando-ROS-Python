#!/usr/bin/env python

import rospy
from os import system
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import random
import math

#================================ Definições Globais ===================================

#TARGETs [X,Y]
target = np.array([7.0,  -6.0]) #red

# Distância mínima para chegar ao alvo
min_distance = 0.5

#================================ Funções ===================================

odometry_msg = Odometry()
velocity = Twist()

def odometry_callback(data):
	global odometry_msg
	odometry_msg = data
	

def move_frente():
	velocity.linear.x = 0.5
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def para():
	velocity.linear.x = 0.0
	velocity.angular.z = 0.0
	pub.publish(velocity)  

	
#================================ Função Principal ===================================

if __name__ == "__main__": 
	# Node
	rospy.init_node("controle_stage_node", anonymous=False)  

	# Subscribers
	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	# Publishers
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub_pos = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
  
	rate = rospy.Rate(10) #10hz
  
	while not rospy.is_shutdown():
      
		move_frente()
		
		x = odometry_msg.pose.pose.position.x
		y = odometry_msg.pose.pose.position.y
		
		distance = math.sqrt((x-target[0])**2 + (y-target[1])**2)  

		rospy.loginfo("T: (%.2f, %.2f)" % (target[0], target[1]))
		rospy.loginfo("P: (%.2f, %.2f)" % (x, y)) 
		rospy.loginfo("D: %.2f" % distance)

		if( distance < 0.5 ):
			para()

		rate.sleep()
