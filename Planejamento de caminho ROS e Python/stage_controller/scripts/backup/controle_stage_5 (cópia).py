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
from sklearn.neighbors import KDTree

#================================ Definições Globais ===================================

#Lista de pontos de destino
targets = [
    np.array([5.0, -10.0]),
    np.array([5.0, -1.0]),
    np.array([-5.0, 0.0]),
    np.array([-5.0, 11.0]),
    np.array([8.0, 12.0]),  #Novo ponto de destino
]

#Variável para rastrear o índice do ponto de destino atual
indice_ponto_alvo_atual = 0

gx = None  # Coordenada X
gy = None  # Coordenada Y

px = 0.0
py = 0.0
pz = 0.0
yaw = 0.0

odometry_msg = Odometry()
velocity = Twist()

#================================ Funções ===============================================

def finish():
    global gx, gy
    gx = 0
    gy = 0
    print("Destino Alcançado")

def odometry_callback(data):
    global odometry_msg
    odometry_msg = data

def Localizacao():
    global px, py, pz, ox, oy, oz, ow, yaw, theta

    px = odometry_msg.pose.pose.position.x
    py = odometry_msg.pose.pose.position.y
    pz = odometry_msg.pose.pose.position.z
    orientation = odometry_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])  

def ModeloCinematico():
    global px, py, theta, indice_ponto_alvo_atual, gx, gy

    wheel_radius = 0.05   # metros
    wheel_base = 0.1      # metros

    # Obtém as coordenadas do ponto de destino atual
    gx = targets[indice_ponto_alvo_atual][0]  # Coordenada X
    gy = targets[indice_ponto_alvo_atual][1]  # Coordenada Y

    dx = gx - px
    dy = gy - py
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    # Calculate the angular velocity to turn towards the goal
    angular_vel = 1 * (angle - theta)

    # Se o robô estiver próximo o suficiente do ponto de destino, pare de se mover
    if distance < 1:
        linear_vel = 0.5
        angular_vel = 0.0

        # Verifica se é o ponto de destino (8, 12)
        if gx == 8.0 and gy == 12.0:
            finish()
        else:
            # Atualize o índice do ponto de destino atual para avançar para o próximo ponto
            indice_ponto_alvo_atual += 1
            if indice_ponto_alvo_atual >= len(targets):
                indice_ponto_alvo_atual = 0
    else:
        # Aumente o valor de linear_vel para aumentar a velocidade linear
        linear_vel = 1.0 * distance

    # Calculate the wheel speeds using the differential kinematics model
    left_wheel_speed = (linear_vel - (angular_vel * wheel_base / 2)) / wheel_radius
    right_wheel_speed = (linear_vel + (angular_vel * wheel_base / 2)) / wheel_radius

    # Create a Twist message and publish it
    velocity.linear.x = linear_vel
    velocity.angular.z = angular_vel
    pub.publish(velocity)

#================================ Função Principal =======================================

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
        
        rospy.loginfo("T1: %.2f, %.2f" % (gx, gy))
        rospy.loginfo("P: (%.2f, %.2f)" % (px, py)) 

        rate.sleep()

        if gx == 0 and gy == 0:
            break

