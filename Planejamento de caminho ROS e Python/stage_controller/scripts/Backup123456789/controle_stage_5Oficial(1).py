#!/usr/bin/env python3.8

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

#================================ Definições Globais =========================================================

#Caminho para a imagem do mapa
image_path = "/home/jhorlen/catkin_ws/src/stage_controller/world/map/mapaObsIcet.pgm"

#Transformar a imagem em uma matriz de mapa binário
matriz_binaria = transformar_imagem_em_mapa(image_path)

#Verificar caminhos livres e obstáculos na matriz
caminhos_livres, caminhos_ocupados = verificar_caminhos_livres_e_obstaculos(matriz_binaria)

#Ponto de partida e ponto de destino
start = (-12, -12)
end = (12, 12)

xend = end[0]
yend = end[1]

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
indice_ponto_alvo_atual = 0

gx = None  #Coordenada X do ponto de destino atual
gy = None  #Coordenada Y do ponto de destino atual

px = 0.0    #Coordenada X atual do robô
py = 0.0    #Coordenada Y atual do robô
pz = 0.0    #Coordenada Z atual do robô
yaw = 0.0   #Ângulo de rotação atual do robô

#Mensagem de odometria para publicação
odometry_msg = Odometry()

#Comando de velocidade para o robô
velocity = Twist()

#================================ Fim Definições Globais =====================================================

#================================ Funções Auxiliares =========================================================

#Função para finalizar o movimento do robô quando o destino é alcançado
def finish():
    global gx, gy
    gx = 0.0
    gy = 0.0
    print("Destino Alcançado")

#Callback para atualizar a mensagem de odometria recebida
def odometry_callback(data):
    global odometry_msg
    odometry_msg = data

#Função para atualizar as coordenadas de localização do robô a partir da mensagem de odometria
def Localizacao():
    global px, py, pz, ox, oy, oz, ow, yaw, theta

    px = odometry_msg.pose.pose.position.x
    py = odometry_msg.pose.pose.position.y
    pz = odometry_msg.pose.pose.position.z
    orientation = odometry_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

#Função para controlar o movimento do robô usando o modelo cinemático diferencial
def ModeloCinematico():
    global px, py, theta, indice_ponto_alvo_atual, gx, gy, odometry_msg

    #Verifica se há uma mensagem de odometria válida
    if odometry_msg is None:
        return

    #Atualiza as coordenadas de localização e orientação do robô
    px = odometry_msg.pose.pose.position.x
    py = odometry_msg.pose.pose.position.y
    orientation = odometry_msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    #Dimensões do robô para o modelo cinemático
    wheel_radius = 0.01  #metros
    wheel_base = 0.2     #metros

    #Obtém as coordenadas do ponto de destino atual
    gx = targets[indice_ponto_alvo_atual][0]  #Coordenada X
    gy = targets[indice_ponto_alvo_atual][1]  #Coordenada Y

    dx = gx - px
    dy = gy - py
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    #Calcula a velocidade angular para girar em direção ao objetivo
    angular_vel = 1 * (angle - theta)

    #Verifica se o robô está próximo o suficiente do ponto de destino para diminuir a velocidade
    if distance < 1.5:
        linear_vel = 0.05
        angular_vel = 0.5

        #Verifica se é o ponto de destino final (end x, y)
        if gx == xend and gy == yend:
            finish()
        else:
            #Atualiza o índice do ponto de destino atual para avançar para o próximo ponto
            indice_ponto_alvo_atual += 1
            if indice_ponto_alvo_atual >= len(targets):
                indice_ponto_alvo_atual = 0
    else:
        #Aumenta o valor de linear_vel para aumentar a velocidade linear
        linear_vel = 1.0 * distance

    #Calcula as velocidades das rodas usando o modelo cinemático diferencial
    left_wheel_speed = (linear_vel - (angular_vel * wheel_base / 2)) / wheel_radius
    right_wheel_speed = (linear_vel + (angular_vel * wheel_base / 2)) / wheel_radius

    #Cria uma mensagem Twist e a publica para controlar o movimento do robô
    velocity.linear.x = linear_vel
    velocity.angular.z = angular_vel
    pub.publish(velocity)


#================================ Fim Funções Auxiliares =====================================================

#================================ Função Principal ===========================================================

if __name__ == "__main__":
    #Inicializa o nó do ROS chamado "controle_stage_node"
    rospy.init_node("controle_stage_node", anonymous=False)

    #Cria um assinante para receber mensagens de odometria do tópico "/base_pose_ground_truth"
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

    #Cria um publicador para enviar comandos de velocidade para o tópico "/cmd_vel"
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)  #Taxa de atualização de 10Hz
    while not rospy.is_shutdown():
        #Atualiza as informações de localização do robô
        Localizacao()

        #Controla o movimento do robô usando o modelo cinemático diferencial
        ModeloCinematico()

        #Exibe informações relevantes no console
        rospy.loginfo("Destino: %.2f, %.2f" % (gx, gy))
        rospy.loginfo("Posição atual: (%.2f, %.2f)" % (px, py))

        rate.sleep()

        #Encerra o loop quando o destino é alcançado (gx e gy são ambos 0)
        if gx == 0 and gy == 0:
            break  #Encerra o loop while

    #Quando o loop while é encerrado, o programa também é encerrado


#================================ Fim Função Principal ========================================================
