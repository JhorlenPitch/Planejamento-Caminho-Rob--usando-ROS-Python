from collections import deque

def find_shortest_path(start, goal):
    queue = deque()
    queue.append(start)

    visited = set()
    visited.add(tuple(start))

    predecessors = {}
    found = False

    while queue:
        current = queue.popleft()

        if current == goal:
            found = True
            break

        x, y = current

        # Gerar os vizinhos do ponto atual (incluindo diagonais)
        neighbors = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
            (x - 1, y - 1),
            (x - 1, y + 1),
            (x + 1, y - 1),
            (x + 1, y + 1)
        ]

        for neighbor in neighbors:
            if neighbor not in visited and neighbor:
                queue.append(neighbor)
                visited.add(neighbor)
                predecessors[neighbor] = current

    if not found:
        return None

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = predecessors[current]
    path.append(start)

    path.reverse()

    return path

