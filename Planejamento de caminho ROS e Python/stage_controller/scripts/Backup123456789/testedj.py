import numpy as np
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, obstacles, step_size, max_iterations):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.tree = {start: None}

    def generate_rrt(self):
        for _ in range(self.max_iterations):
            random_point = self.generate_random_point()
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.steer(nearest_point, random_point)
            if self.check_collision(new_point):
                self.tree[new_point] = nearest_point
                if self.check_goal_reached(new_point):
                    return self.get_path(new_point)

        return None

    def generate_random_point(self):
        # Gera um ponto aleatório dentro dos limites do espaço de configuração
        # Você pode adaptar isso de acordo com as dimensões do seu espaço de configuração
        return (np.random.uniform(-10, 10), np.random.uniform(-10, 10))

    def find_nearest_point(self, point):
        # Encontra o ponto mais próximo na árvore atual
        distances = [self.distance(p, point) for p in self.tree.keys()]
        nearest_index = np.argmin(distances)
        return list(self.tree.keys())[nearest_index]

    def steer(self, from_point, to_point):
        # Realiza uma etapa de avanço em direção ao ponto desejado
        vector = np.array(to_point) - np.array(from_point)
        magnitude = np.linalg.norm(vector)
        if magnitude > self.step_size:
            unit_vector = vector / magnitude
            new_point = tuple(np.array(from_point) + self.step_size * unit_vector)
        else:
            new_point = to_point
        return new_point

    def check_collision(self, point):
        # Verifica se o ponto colide com algum obstáculo
        for obstacle in self.obstacles:
            # Lógica de verificação de colisão específica do seu problema
            # Aqui você precisa implementar a lógica adequada para o seu caso
            if self.is_collision(point, obstacle):
                return False
        return True

    def is_collision(self, point, obstacle):
        # Lógica de verificação de colisão específica do seu problema
        # Aqui você precisa implementar a lógica adequada para o seu caso
        return False

    def check_goal_reached(self, point):
        # Verifica se o ponto alcança o objetivo
        return self.distance(point, self.goal) < self.step_size

    def get_path(self, point):
        # Reconstrói o caminho percorrido a partir do ponto final
        path = [point]
        while point != self.start:
            point = self.tree[point]
            path.append(point)
        return list(reversed(path))

    def distance(self, p1, p2):
        # Calcula a distância entre dois pontos
        return np.linalg.norm(np.array(p1) - np.array(p2))

# Exemplo de uso
start = (0, 0)
goal = (8, 8)
obstacles = [(2, 2), (4, 4), (6, 6)]
step_size = 0.5
max_iterations = 1000

rrt = RRT(start, goal, obstacles, step_size, max_iterations)
path = rrt.generate_rrt()

if path:
    print("Caminho encontrado:")
    for point in path:
        print(point)
else:
    print("Não foi possível encontrar um caminho.")

