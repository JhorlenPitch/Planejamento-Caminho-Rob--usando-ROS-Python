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

# Lista de pontos de destino
targets = [
    np.array([6.0, -7.0]),  # Ponto 1
    np.array([6.0, 2.0]),   # Ponto 2
    np.array([2.0, 2.0]),
    np.array([-6.0, 2.0]),    # Ponto 3
    np.array([-6.0, 7.0]),   # Ponto 3
    np.array([8.0, 9.0]),    # Ponto 3
    np.array([8.0, 8.0])   # Ponto 3
]

# Índice do alvo atual
current_target_index = 0
gx, gy = targets[current_target_index]  # Define o alvo atual como o primeiro ponto

#target = np.array([0.0,  0.0]) #red
#gx = targets[0]
#gy = targets[1]

px = 0.0
py = 0.0
pz = 0.0
yaw = 0.0

odometry_msg = Odometry()
velocity = Twist()

#================================ Funções ===================================
def finish():
    global gx, gy
    gx = 0.0
    gy = 0.0
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
    global px, py, theta, current_target_index, gx, gy

    wheel_radius = 0.05  	# metros
    wheel_base = 0.1  		# metros

    dx = gx - px
    dy = gy - py
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    # Calcula a velocidade angular para virar em direção ao objetivo
    angular_diff = math.atan2(math.sin(angle - theta), math.cos(angle - theta))
    angular_vel = 0.5 * angular_diff

    # Se o robô estiver perto o suficiente do objetivo, vá para o próximo alvo
    if distance < 0.8:
    
        # Verifica se é o ponto de destino final (end x, y)
        if np.allclose([gx, gy], [8.0, 8.0], atol=0.1):
            finish()
        else:
            current_target_index = (current_target_index + 1) % len(targets)
            gx, gy = targets[current_target_index]
            rospy.loginfo("Indo para o alvo %d: (%.2f, %.2f)" % (current_target_index + 1, gx, gy))
            return

    # Calcula a velocidade linear para mover em direção ao objetivo
    linear_vel = 0.5 * distance

    # Calcula as velocidades das rodas usando o modelo cinemático diferencial
    left_wheel_speed = (linear_vel - (angular_vel * wheel_base / 2)) / wheel_radius
    right_wheel_speed = (linear_vel + (angular_vel * wheel_base / 2)) / wheel_radius

    # Cria uma mensagem Twist e a publica
    velocity.linear.x = linear_vel
    velocity.angular.z = angular_vel
    pub.publish(velocity)
	
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
		
		#Encerra o loop quando o destino é alcançado (gx e gy são ambos 0)
		if gx == 0 and gy == 0:
		    break  #Encerra o loop while
