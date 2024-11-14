import heapq
import math

PoI = {
        'start': (0, 0),
        'goal': (-1.6, -1.6),
        'ring1': (0.6, 0.6),
        'ring2': (0.6, 1.2),
        'ring3': (1.4, 1.6),
        'ring4': (1.8, 1.8),
        'ring5': (0, 1.5),
        'ring6': (-0.6, -0.6),
        'ring7': (-0.6, -1.2),
        'ring8': (-1.4, -1.6),
        'ring9': (-1.8, -1.8),
        'ring10': (0, 1.5),
        'ring11': (0, -1.5),
    }

rings_count = {
        'ring1': ['ring1'],
        'ring2': ['ring1', 'ring2'],
        'ring3': ['ring1', 'ring2'],
        'ring4': ['ring1', 'ring2','ring3','ring4'],
        'ring5': ['ring1', 'ring2'],
        'ring6': ['ring1'],
        'ring7': ['ring1','ring2'],
        'ring8': ['ring1', 'ring2'],
        'ring9': ['ring1', 'ring2','ring3','ring4'],
        'ring10': ['ring1', 'ring2'],
        'ring11': ['ring1', 'ring2']
    }

graph = {
        'start': {'ring1': 1.2, 'ring2': 1.5},
        'ring1': {'goal': 0.8, 'ring2': 0.5},
        'ring2': {'goal': 0.7, 'ring3': 0.4},
        'ring3': {'goal': 0.9}
    }

def distance_from_nodes(node1, node2):
    x1, y1 = PoI[node1]
    x2, y2 = PoI[node2]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def weight(current_node, neighbor):
    base_distance = graph[current_node][neighbor]
    rings_at_neighbor = len(rings_count.get(neighbor, []))
    priority_adjustment = 1 / (1 + rings_at_neighbor)  
    return base_distance * priority_adjustment

def dijkstra(start, goal):
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {node: float('inf') for node in PoI}
    distances[start] = 0
    previous_nodes = {node: None for node in PoI}
    
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_node == goal:
            break
        for neighbor in graph.get(current_node, {}):
            current_weight = weight(current_node, neighbor)
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

def main():
    start = 'start'
    goal = 'goal'
    path, total_distance = dijkstra(start, goal)
    print("Shortest path:", path)
    print("Total distance:", total_distance)

if __name__ == "__main__":
    main()
