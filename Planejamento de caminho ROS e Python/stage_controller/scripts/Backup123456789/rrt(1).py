import numpy as np
import random

class RRT:
    def __init__(self, obstacles, start, goal, step_size=0.5, max_iter=10000):
        self.obstacles = obstacles
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = {}

    def generate_random_point(self):
        x = random.uniform(-10, 10)  # Substitua pelos limites do seu mapa
        y = random.uniform(-10, 10)  # Substitua pelos limites do seu mapa
        return np.array([x, y])

    def nearest_neighbor(self, point):
        distances = [np.linalg.norm(point - p) for p in self.tree.keys()]
        nearest_idx = np.argmin(distances)
        nearest_point = list(self.tree.keys())[nearest_idx]
        return nearest_point

    def new_point(self, nearest_point, random_point):
        direction = random_point - nearest_point
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return random_point
        else:
            unit_direction = direction / distance
            new_point = nearest_point + self.step_size * unit_direction
            return new_point

    def is_collision_free(self, point):
        for obstacle in self.obstacles:
            if np.linalg.norm(point - obstacle) <= 1.0:  # Substitua pelo raio do seu robô
                return False
        return True

    def find_shortest_path(self):
        self.tree[self.start] = None

        for _ in range(self.max_iter):
            random_point = self.generate_random_point()
            nearest_point = self.nearest_neighbor(random_point)
            new_point = self.new_point(nearest_point, random_point)

            if self.is_collision_free(new_point):
                self.tree[new_point] = nearest_point

                if np.linalg.norm(new_point - self.goal) <= self.step_size:
                    path = [new_point]
                    current_point = new_point

                    while self.tree[current_point] is not None:
                        current_point = self.tree[current_point]
                        path.append(current_point)

                    path.append(self.start)
                    path.reverse()
                    return path

        return None
