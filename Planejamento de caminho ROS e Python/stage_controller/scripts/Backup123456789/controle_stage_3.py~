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
target = np.array([-9.0,  -12.0]) #baixo


#target = np.array([-3.0,  3.0]) #red
#### Quad 3 ####
#target = np.array([8.0,  8.0]) #baixo

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
	

def move_frente():
	velocity.linear.x = 0.5
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def para():
	velocity.linear.x = 0.0
	velocity.angular.z = 0.0
	pub.publish(velocity)  

def Localizacao():
	global px,py,pz,ox,oy,oz,ow,yaw #,ang

	px = odometry_msg.pose.pose.position.x
	py = odometry_msg.pose.pose.position.y
	pz = odometry_msg.pose.pose.position.z
	ox = odometry_msg.pose.pose.orientation.x
	oy = odometry_msg.pose.pose.orientation.y
	oz = odometry_msg.pose.pose.orientation.z
	ow = odometry_msg.pose.pose.orientation.w
	
	rospy.loginfo("1>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> (%.2f, %.2f, %.2f)" % (px, py, pz)) 
	rospy.loginfo("2>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> (%.2f, %.2f, %.2f, %.2f)" % (ox, oy, oz, ow)) 
		
	(roll, pitch, yaw) = euler_from_quaternion ([ox, oy, oz, ow])
	ang = yaw
	rospy.loginfo("====================>> (%.2f, %.2f)" % (yaw, ang)) 


def ModeloCinematico():
	global px, py #, yaw, gx, gy, goalX, goalY
	global anteriorHip, anteriorB, anteriorVA

	# Calculo das distâncias entre a posição atual e posiçaõ desejada.
	print("______ ", gx," - ", gy)
	print(">>>>>> ", px," - ", py)
	dx = (gx-px)-0.05
	dy = (gy-py)-0.05
	hip = math.sqrt( (dx*dx) + (dy*dy) )

	# Calculo dos ângulos de orientação para movimentação do robô na direção correta.
	alpha = -theta + math.atan2(dx,dy)
	beta = -theta-alpha

	# Cálculo das coordenadas polares 
	mat1[0][0] = (-kp * hip * math.cos(alpha))
	mat1[0][0] = mat1[0][0] + (mat1[0][0]-anteriorHip)

	pa = (kp*math.sin(alpha) - ka*alpha - kb*beta)
	pb = (-kp * math.sin(alpha))

	# Condição para determinar para fazer com que o robô siga o trajeto definido pelo modelo cinemático.
	# No entanto, seguir este trajeto gera oscilações.
	if( pb <= (anteriorB ) ):				# retornar
		mat1[1][0] = (-pa) 
		mat1[2][0] = (-pb)
	else:
		mat1[1][0] = (pa)
		mat1[2][0] = pb

	anteriorHip = hip
	anteriorB = pb
		
	# Cálculo dos dados da entrada de controle.
	# Velocidade linear e angular.
	v = kp*mat1[0][0]
	va = (ka*mat1[1][0] + kb*mat1[2][0])

	# Condições para determinar quais sinais devem ser aplicados às velocidade.
	# É definido se o robô deve se deslocar para direita, para esquerda, para frente ou para trás.
	if( abs(dx)<0.5 and abs(dy)<0.5 ):
		print("............................")
		velocity.linear.x = 0
		velocity.angular.z = 0
	elif( px<=0 and py<=0 ):				#	quadrante 4		// theta == 0 (+)	theta == 90 (-)
		print("4............................", np.degrees(alpha), " - ", dy)
		velocity.linear.x = 0 # v/2
		"""
		if( alpha >= 1.5708 and alpha < 4.71239 ):		# cima
			velocity.linear.x = v/2				# + (tras) - frente
			print("4################################")
		else:									# baixo
			velocity.linear.x = -v/2				# + (tras) - frente
			print("4*******************************")
		"""
		velocity.angular.z = 0 # (va+anteriorVA)/1.8		# O uso da Velocidade angular anterior ajuda a minizar a oscilação e seguir o comportamento previsto no modelo cinemático
	elif( px>0 and py<0 ):				#	quadrante 3		// theta == 180 (-)	theta == 90 (+)
		print("3............................")
		if( alpha >= 1.5708 and alpha < 4.71239 ):	
			velocity.linear.x = v/2 					# baixo
			print("3################################")
		else:									# cima
			velocity.linear.x = -v/2		
			print("3*******************************")
		velocity.angular.z = (va+anteriorVA)/2			# O uso da Velocidade angular anterior ajuda a minizar a oscilação e seguir o comportamento previsto no modelo cinemático		
	elif( px<0 and py>0 ):				#	quadrante 2		// theta == 270 (-)	theta == 0 (+)
		print("2............................")
		if( alpha >= 1.5708 and alpha < 4.71239 ):		# baixo
			velocity.linear.x = v/2
			print("2################################")
		else:
			velocity.linear.x = -v/2					# cima
			print("2*******************************")
			
		velocity.angular.z = (va+anteriorVA)/2 			# O uso da Velocidade angular anterior ajuda a minizar a oscilação e seguir o comportamento previsto no modelo cinemático			
	elif( px>0 and py>0 ):				#	quadrante 1		// theta == 270 (+)	theta == 180 (-)
		print("1............................")
		if( alpha >= 1.5708 and alpha < 4.71239 ):		# baixo
			velocity.linear.x = v/2					
			print("1################################")
		else:									# cima
			velocity.linear.x = -v/2
			print("1*******************************")
		
		velocity.angular.z = (va+anteriorVA)/1.9 		# O uso da Velocidade angular anterior ajuda a minizar a oscilação e seguir o comportamento previsto no modelo cinemático


	# -
	# -
	# +
	# +

	anteriorVA=-va


	"""
#	goalX = gx
#	goalY = gy
	
#	rospy.loginfo("__________________ (%.2f, %.2f)" % (goalX, goalY))
	rospy.loginfo("yaw___________ (%.2f, %.2f)" % (yaw, ang)) 


	dx = (gx-px)-0.05
	dy = (gy-py)-0.05

	rospy.loginfo("__________________ (%.2f, %.2f)" % (dx, dy))

	alpha = math.atan2(dx,dy);
	alpha2 = -0+math.atan2(dx,dy);

	rospy.loginfo("alpha__________________ (%.2f, %.2f)" % (alpha, alpha2))

	if(dx>0):			# Direita
		if(alpha>0.5 and yaw-alpha<0.5):		# baixo
			rospy.loginfo("1__________________")
			velocity.linear.x = 1 #-(gy-py)*(1*yaw-alpha);		# + (trás)
			velocity.linear.y = -3 #(gx-px)*(1*alpha);			#-
			velocity.angular.z = 0;
		else:
			rospy.loginfo("2__________________")
			velocity.linear.x = (gy-py)*(8*yaw-alpha);
			velocity.linear.y = -(gx-px)*(1*alpha);
			velocity.angular.z = 0;

	elif(dx<=0):		# Esquerda
		if(alpha>0.35 and yaw-alpha<0.35):
			rospy.loginfo("3__________________")
			velocity.linear.x = -(gy-py)*(1*yaw-alpha);
			velocity.linear.y = (gx-px)*(1*alpha);
			velocity.angular.z = 0;
		else:
			rospy.loginfo("4__________________")
			velocity.linear.x = (gy-py)*(8*yaw-alpha);
			velocity.linear.y = (gx-px)*(1*alpha);
			velocity.angular.z = 0;
	"""

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
      
		Localizacao()
		
		distance = math.sqrt((px-gx)**2 + (py-gy)**2)  
#		distance = math.sqrt((px-goalX)**2 + (py-goalY)**2)  

#		move_frente()
		ModeloCinematico()
		


		rospy.loginfo("T1: (%.2f, %.2f)" % (gx, gy))
#		rospy.loginfo("T2: (%.2f, %.2f)" % (goalX, goalY))
		rospy.loginfo("P: (%.2f, %.2f)" % (px, py)) 
		rospy.loginfo("Y: (%.2f)" % (yaw)) 
		rospy.loginfo("D: %.2f" % distance)

		if( distance < 0.5 ):
			para()

		rate.sleep()
