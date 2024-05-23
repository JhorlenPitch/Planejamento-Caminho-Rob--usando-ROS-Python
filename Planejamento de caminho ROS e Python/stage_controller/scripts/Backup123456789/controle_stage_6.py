#!/usr/bin/env python

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
target = np.array([8.0,  8.0]) #baixo

#target = np.array([0.0,  0.0]) #red
gx = target[0]
gy = target[1]
#goalX = gy
#goalY = gx

# Distância mínima para chegar ao alvo
min_distance = 0.5

px = 0.0
py = 0.0
pz = 0.0
ox = 0.0
oy = 0.0
oz = 0.0
ow = 0.0
ang = 0.0
yaw = 0.0

kp = 3.0
ka = 8.0
kb = -0.1;

anteriorB = 0.0;
anteriorHip = 0.0;
anteriorVA = 0.0;

theta = 0.0

mat1 = np.zeros((3,3));

odometry_msg = Odometry()
velocity = Twist()

#================================ Funções ===================================

def odometry_callback(data):
	global odometry_msg
	odometry_msg = data
	
"""
def move_frente():
	velocity.linear.x = 0.5
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def para():
	velocity.linear.x = 0.0
	velocity.angular.z = 0.0
	pub.publish(velocity)  
"""


def Localizacao():
	global px,py,pz,ox,oy,oz,ow,yaw #,ang

	px = odometry_msg.pose.pose.position.x
	py = odometry_msg.pose.pose.position.y
	pz = odometry_msg.pose.pose.position.z
	ox = odometry_msg.pose.pose.orientation.x
	oy = odometry_msg.pose.pose.orientation.y
	oz = odometry_msg.pose.pose.orientation.z
	ow = odometry_msg.pose.pose.orientation.w

	orientation = odom_msg.pose.pose.orientation
	(roll, pitch, self.theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])	
		
	(roll, pitch, yaw) = euler_from_quaternion ([ox, oy, oz, ow])
	ang = yaw
	rospy.loginfo("====================>> (%.2f, %.2f)" % (yaw, ang)) 


def ModeloCinematico(i):
	global px, py #, yaw, gx, gy, goalX, goalY
	global anteriorHip, anteriorB, anteriorVA
	qSteps = 1000000

	# Calculo das distâncias entre a posição atual e posiçaõ desejada.
	dx = (gx-px)
	dy = (gy-py)
	print("dx: ", dx, " - px: ", px, " - gx: ", gx)
	print("dy: ", dy, " - py: ", py, " - gy: ", gy)

	# Calculo dos ângulos de orientação para movimentação do robô na direção correta.
	alpha = math.atan2(dx,dy)
	beta = -0+math.atan2(dx,dy)


	if( abs(px-gy)<=1 and abs(py-gx)<=1 ):
		if(i==qSteps):
			velocity.linear.x = 0
			velocity.linear.y = 0
			velocity.angular.z = 0
			print("PAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARRRRRRRRRRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAA")
		else:
			i+=1
			print("i: ", i)


	else:

		if(dx>0):

			if(alpha>0.5 and yaw-alpha<0.5):
				print("*******************************************************111", alpha, " - ", yaw-alpha )
				"""
				velocity.linear.x = -(gx-py)*(1*yaw-alpha)
				velocity.linear.y = -(gy-px)*(1*alpha)
				velocity.angular.z = 0
				"""
				velocity.linear.x = (gx-px)*(1*yaw-alpha)
				velocity.linear.y = -(gy-py)*(1*alpha)
				velocity.angular.z = 0

			else:
				print("*******************************************************222222", alpha, " - ", yaw-alpha)
				"""
				velocity.linear.x = (gx-py)*(8*yaw-alpha)
				velocity.linear.y = -(gy-px)*(1*alpha)
				velocity.angular.z = 0
				"""
				velocity.linear.x = (gx-px)*(8*yaw-alpha)
				velocity.linear.y = -(gy-py)*(1*alpha)
				velocity.angular.z = 0

		elif(dx<=0):

			if(alpha>0.35 and yaw-alpha<0.35):
				print("*******************************************************333333333")
				"""
				velocity.linear.x = -(gx-py)*(1*yaw-alpha)
				velocity.linear.y = (gy-px)*(1*alpha)
				velocity.angular.z = 0
				"""
				velocity.linear.x = -(gx-px)*(1*yaw-alpha)
				velocity.linear.y = (gy-py)*(1*alpha)
				velocity.angular.z = 0

			else:
				"""
				print((gx-py)*(8*yaw-alpha))
				print((gy-px)*(1*alpha))
				print("_______________________________________________________")
				velocity.linear.x = (gx-py)*(8*yaw-alpha)
				velocity.linear.y = (gy-px)*(1*alpha)
				velocity.angular.z = 0
				"""
				print("*******************************************************444444444444")
				print((gx-px)*(8*yaw-alpha))
				print((gy-py)*(1*alpha))
				print("_______________________________________________________")
				velocity.linear.x = (gx-px)*(8*yaw-alpha)
				velocity.linear.y = (gy-py)*(1*alpha)
				velocity.angular.z = 0


	pub.publish(velocity);

	return i
	
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
  	
	i = 0
  	
	while not rospy.is_shutdown():
      
		Localizacao()
		
		distance = math.sqrt((px-gx)**2 + (py-gy)**2)  
#		distance = math.sqrt((px-goalX)**2 + (py-goalY)**2)  

#		move_frente()
		ModeloCinematico(i)
		
	#	velocity.linear.x = 0.5
	#	velocity.linear.y = 0.5
	#	velocity.angular.z = 0.0
	#	pub.publish(velocity)

		rospy.loginfo("T1: (%.2f, %.2f)" % (gx, gy))
#		rospy.loginfo("T2: (%.2f, %.2f)" % (goalX, goalY))
		rospy.loginfo("P: (%.2f, %.2f)" % (px, py)) 
		rospy.loginfo("Y: (%.2f)" % (yaw)) 
		rospy.loginfo("D: %.2f" % distance)

	#	if( distance < 0.5 ):
	#		para()

		rate.sleep()
