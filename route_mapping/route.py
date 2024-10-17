import heapq
import math
import matplotlib as plt
import matplotlib as patches
from matplotlib.animation import FuncAnimation

def distance_from_nodes(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def weight(current_node, neighbor, rings_count):
    dist = distance_from_nodes(current_node, neighbor)
    rings_at_neighbor = len(rings_count.get(neighbor, []))
    priority_adjustment = 1 + rings_at_neighbor
    return dist * priority_adjustment

def dijkstra(start, goal, nodes, rings_count):
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {node: float('inf') for node in nodes}
    distances[start] = 0
    previous_nodes = {node: None for node in nodes}

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == goal:
            break

        for neighbor in get_neighbors(current_node, nodes):
            current_weight = weight(current_node, neighbor, rings_count)
            total_distance = current_distance + current_weight

            if total_distance < distances[neighbor]:
                distances[neighbor] = total_distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (total_distance, neighbor))

    path = []
    current = goal
    while current:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()

    return path, distances[goal]

def get_neighbors(node, nodes, grid_size=0.1):
    neighbors = []
    x, y = node
    for dx in [-grid_size, 0, grid_size]:
        for dy in [-grid_size, 0, grid_size]:
            if dx == 0 and dy == 0:
                continue
            neighbor = (x + dx, y + dy)
            if neighbor in nodes:
                neighbors.append(neighbor)
    return neighbors

def generate_nodes(grid_size = 0.1):
    Field_Max = 1.8
    Field_Min = -1.8
    nodes = []
    x = Field_Min
    while x <= Field_Max:
        y = Field_Min
        while y <= Field_Max:
                nodes.append((round(x, 2)), round(y, 2))
                y += grid_size
                x += grid_size
                
        return nodes
    
    
            
    