#!/usr/bin/env python3.8

#17 agosto 2023 0.1

import rospy
import numpy as np
import random
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from collections import deque
from biblioteca import * #Biblioteca local, contendo o rrt e outras funções

from os import system


#================================ Definições Globais =========================================================

#Caminho para a imagem do mapa
image_path = "/home/joe/catkin_ws/src/stage_controller/world/map/mapaObs5.pgm"

#Transformar a imagem em uma matriz de mapa binário
matriz_binaria = transformar_imagem_em_mapa(image_path)

#Verificar caminhos livres e obstáculos na matriz
caminhos_livres, caminhos_ocupados = verificar_caminhos_livres_e_obstaculos(matriz_binaria)

#Ponto de partida e ponto de destino

xstart = -12
ystart = -14

xend = 12
yend = 14

start = (xstart, ystart)
end = (xend, yend)

#Definir o número máximo de iterações e o tamanho máximo das conexões (step_size) para o RRT
max_iter = 5000
step_size = 1.0

#Executar o algoritmo RRT para obter o caminho e o grafo resultante
caminho, rrt_result = rrt(caminhos_livres, caminhos_ocupados, start, end, max_iter=max_iter, step_size=step_size)

#Salvar as arestas do grafo do RRT em uma variável
lista_de_arestas = rrt_result.edges()

print("\n")
print("Grafo do RRT:")
print(lista_de_arestas)

#Atribuir as arestas do grafo do RRT a uma variável chamada "graph_edges"
graph_edges = lista_de_arestas

#Criar um grafo a partir das arestas
graph = create_graph(graph_edges)

#Encontrar o caminho mais curto usando o algoritmo de Dijkstra
shortest_path = dijkstra(graph, start, end)

print("\n")
if shortest_path:
    print("Caminho mais curto:", shortest_path)
    print("\n")
else:
    print("Não há caminho entre o vértice de início e o vértice de destino.")

#Lista de pontos de destino
targets = shortest_path

#Imprimir a lista de pontos de destino
#for point in targets:
#    print(point)

#Variável para rastrear o índice do ponto de destino atual
current_target_index = 0

gx, gy = targets[current_target_index]  # Define o alvo atual como o primeiro ponto

px = 0.0    #Coordenada X atual do robô
py = 0.0    #Coordenada Y atual do robô
pz = 0.0    #Coordenada Z atual do robô
yaw = 0.0   #Ângulo de rotação atual do robô

#Mensagem de odometria para publicação
odometry_msg = Odometry()

#Comando de velocidade para o robô
velocity = Twist()

#================================ Fim Definições Globais =====================================================

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

    wheel_radius = 0.05  # metros
    wheel_base = 0.1     # metros

    dx = gx - px
    dy = gy - py
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    # Calcula a velocidade angular para virar em direção ao objetivo
    angular_diff = math.atan2(math.sin(angle - theta), math.cos(angle - theta))
    angular_scale = 2.5  # Ajuste este valor para controlar a velocidade angular
    angular_vel = angular_scale * angular_diff

    # Se o robô estiver perto o suficiente do objetivo, vá para o próximo alvo
    if distance < 0.8:
    
        # Verifica se é o ponto de destino final (end x, y)
        if np.allclose([gx, gy], [xend, yend], atol=0.1):
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
