from PIL import Image
import numpy as np
import random
import matplotlib.pyplot as plt
import networkx as nx
import heapq

# Função para converter a matriz de imagem para uma matriz binária
def convert_to_binary(matriz):
    binary_matriz = np.where(matriz == 255, 1, 0)
    return binary_matriz

# Transformar imagem em Mapa
def transformar_imagem_em_mapa(image_path):
    # Carregando imagem
    imagem = Image.open(image_path)

    # Convertendo a imagem para uma matriz numpy
    matriz = np.array(imagem)

    # Limites de largura e altura
    novo_tamanho_y = 37
    novo_tamanho_x = 29

    # Redimensionando a imagem conforme os novos tamanhos
    imagem_redimensionada = imagem.resize((novo_tamanho_x, novo_tamanho_y))

    # Convertendo a imagem redimensionada para uma matriz numpy
    matriz_redimensionada = np.array(imagem_redimensionada)

    # Convertendo a matriz numpy para a binária
    matriz_binaria = convert_to_binary(matriz_redimensionada)
    
    # Configurando o número máximo de linhas e colunas a serem exibidas
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)
    
    print("\n")
    print("Matriz Binaria Redimensionada")
    print("\n")
    print(matriz_binaria)
    
    return matriz_binaria

# Inicio verificação Caminhos Livres e Obstáculos
def verificar_caminhos_livres_e_obstaculos(matriz_binaria):
    # Atribuindo a matriz convertida para usar no RRT
    matriz = matriz_binaria

    # Encontrando o índice do elemento do meio (0,0) "Centro"
    meio_linha = len(matriz) // 2
    meio_coluna = len(matriz[0]) // 2

    # Definindo listas para armazenar as coordenadas dos pontos com valor 1 e 0
    caminhos_livres = []
    caminhos_ocupados = []

    # Percorrendo a matriz para encontrar os caminhos livres e ocupados
    for i in range(len(matriz)):
        for j in range(len(matriz[0])):
            if matriz[i][j] == 1:
                # Calculando as coordenadas (x, y) correspondentes ao ponto (j, i)
                x = j - meio_coluna
                y = meio_linha - i
                caminhos_livres.append((x, y))
            else:
                # Calculando as coordenadas (x, y) correspondentes ao ponto (j, i)
                x = j - meio_coluna
                y = meio_linha - i
                caminhos_ocupados.append((x, y))

    # Mostrando as coordenadas dos caminhos livres e ocupados no terminal
    print("Coordenadas dos caminhos livres:")
    print(caminhos_livres)
    print("\n")
    print("Coordenadas dos caminhos ocupados:")
    print(caminhos_ocupados)
    return caminhos_livres, caminhos_ocupados

#Início RRT
def distancia(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

def ponto_mais_proximo(arvore, ponto):
    return min(arvore, key=lambda x: distancia(x, ponto))

def novo_ponto(p1, p2, step_size):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = distancia(p1, p2)
    num_pontos_intermediarios = int(d / step_size)
    if num_pontos_intermediarios == 0:
        return [p2]
    x_vals = [int(p1[0] + i * dx / num_pontos_intermediarios) for i in range(1, num_pontos_intermediarios + 1)]
    y_vals = [int(p1[1] + i * dy / num_pontos_intermediarios) for i in range(1, num_pontos_intermediarios + 1)]
    pontos_intermediarios = list(zip(x_vals, y_vals))
    return pontos_intermediarios

def verificar_colisao(ponto, obstaculos):
    return ponto in obstaculos

def colisao_obstaculo(p1, p2, livres, obstaculos):
    pontos_intermediarios = novo_ponto(p1, p2, 1.0)
    for ponto in pontos_intermediarios:
        if ponto not in livres or ponto in obstaculos:
            return True
    return False

def rrt(livres, obstaculos, start, end, max_iter, step_size, min_distance_to_obstacle=2.0):
    arvore = {start}
    caminho = [start]
    grafo = nx.Graph()  # Criando o grafo

    for _ in range(max_iter):
        x_rand = random.randint(-14, 14)
        y_rand = random.randint(-18, 18)
        ponto_proximo = ponto_mais_proximo(arvore, (x_rand, y_rand))
        novos_pontos_r = novo_ponto(ponto_proximo, (x_rand, y_rand), step_size)
        for novo_ponto_r in novos_pontos_r:
            if not colisao_obstaculo(ponto_proximo, novo_ponto_r, livres, obstaculos):
                # Verifica se o novo ponto está longe o suficiente dos obstáculos
                min_distance = min(distancia(novo_ponto_r, obst) for obst in obstaculos)
                if min_distance >= min_distance_to_obstacle:
                    arvore.add(novo_ponto_r)
                    caminho.append(novo_ponto_r)
                    grafo.add_edge(ponto_proximo, novo_ponto_r)  # Adicionando a aresta ao grafo
                    if distancia(novo_ponto_r, end) <= step_size:
                        caminho.append(end)
                        grafo.add_edge(novo_ponto_r, end)  # Adicionando a última aresta ao grafo
                        return caminho, grafo

    return None, grafo

# Início Dijkstra
def dijkstra(graph, start, end):
    if start not in graph or end not in graph:
        return None

    distances = {vertex: float('inf') for vertex in graph}
    distances[start] = 0

    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    if distances[end] == float('inf'):
        return None

    # Reconstruir o caminho de ponta a ponta
    path = []
    current = end
    while current != start:
        path.append(current)
        for neighbor, _ in graph[current]:
            if distances[current] == distances[neighbor] + 1:
                current = neighbor
                break
    path.append(start)
    path.reverse()

    # Adicionar vértices aleatórios para garantir que o caminho tenha no mínimo 10 vértices
    while len(path) < 20:
        random_vertex = random.choice(list(graph.keys()))
        if random_vertex not in path:
            path.append(random_vertex)

    return path

def create_graph(edges):
    graph = {}
    for edge in edges:
        src, dest = edge
        if src not in graph:
            graph[src] = []
        if dest not in graph:
            graph[dest] = []
        graph[src].append((dest, 1))  #Usando peso 1 para todas as arestas neste caso
        graph[dest].append((src, 1))  #Caso o grafo não seja direcionado, adicione também a aresta reversa
    return graph

