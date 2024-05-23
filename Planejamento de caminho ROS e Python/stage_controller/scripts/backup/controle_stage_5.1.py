#!/usr/bin/env python3.8

import rospy
from os import system
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import random
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#================================ Definições Globais ===================================

#TARGETs [X,Y]
#### Quad 4 ####
#target = np.array([7.0,  -5.0]) #cima
#target = np.array([7.0,  -7.5]) #mesma linha
#target = np.array([7.0,  -10.0]) #baixo

#target = np.array([-1.0,  0.0]) #cima
#target = np.array([-5.0,  0.0]) #mesma linha
#target = np.array([-9.0,  0.0]) #baixo

#target = np.array([-9.0, -5]) #red

#target = np.array([-1.0,  -12.0]) #cima
#target = np.array([-5.0,  -12.0]) #mesma linha
#target = np.array([-9.0,  -12.0]) #baixo					####


#target = np.array([-3.0,  3.0]) #red
#### Quad 3 ####
target = np.array([0.0,  0.0]) #baixo

#target = np.array([0.0,  0.0]) #red
gx = target[0]
gy = target[1]


px = 0.0
py = 0.0
pz = 0.0
yaw = 0.0

odometry_msg = Odometry()
velocity = Twist()

#================================ Funções ===================================

def finish():
	gx=0
	gy=0
	print("Destino Alcançado")

def odometry_callback(data):
	global odometry_msg
	odometry_msg = data
	
def Localizacao():
	global px,py,pz,ox,oy,oz,ow,yaw,theta

	px = odometry_msg.pose.pose.position.x
	py = odometry_msg.pose.pose.position.y
	pz = odometry_msg.pose.pose.position.z
	orientation = odometry_msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])	
		

def ModeloCinematico():
	global px, py, theta

	wheel_radius = 0.05  	# meters
	wheel_base = 0.1  		# meters

	dx = gx - px
	dy = gy - py
	distance = math.sqrt(dx ** 2 + dy ** 2)
	angle = math.atan2(dy, dx)

	# Calculate the angular velocity to turn towards the goal
	angular_vel = 0.5 * (angle - theta)

	# If the robot is close enough to the goal, stop moving
	if distance < 0.5:
	    linear_vel = 0.0
	    angular_vel = 0.0
	    
	else:
		# Calculate the linear velocity to move towards the goal
		linear_vel = 0.2 * distance

	# Calculate the wheel speeds using the differential kinematics model
	left_wheel_speed = (linear_vel - (angular_vel * wheel_base / 2)) / wheel_radius
	right_wheel_speed = (linear_vel + (angular_vel * wheel_base / 2)) / wheel_radius

	# Create a Twist message and publish it
	velocity.linear.x = linear_vel
	velocity.angular.z = angular_vel
	pub.publish(velocity);
	
#================================ Função Principal ===================================

if __name__ == "__main__": 
	# Node
	rospy.init_node("controle_stage_node", anonymous=False)  

	# Subscribers
	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	# Publishers
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
  
	rate = rospy.Rate(10) #10hz
	while not rospy.is_shutdown():
		
      
		Localizacao()
		
		ModeloCinematico()
		
		rospy.loginfo("T1: (%.2f, %.2f)" % (gx, gy))
		rospy.loginfo("P: (%.2f, %.2f)" % (px, py)) 
		

		rate.sleep()
