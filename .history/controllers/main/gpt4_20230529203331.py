import heapq
import math
import matplotlib.pyplot as plt
import random
from Coord import *

class Node:
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.priority = cost + heuristic

    def __eq__(self, other):
        return self.position == other.position
        
    def __lt__(self, other):
        return self.priority < other.priority or (self.priority == other.priority and self.cost > other.cost)

def calculate_heuristic(current, goal):
    dx = abs(current[0] - goal[0])
    dy = abs(current[1] - goal[1])
    return min(dx, dy) * 14 + abs(dx - dy) * 10  

def get_neighbors(current, map):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    for dx, dy in directions:
        x = current[0] + dx
        y = current[1] + dy
        if 0 <= x < len(map) and 0 <= y < len(map[0]):
            
            if abs(map[x][y] - 1) < 0.1:
                cost = 1
            elif map[x][y] == 0:
                cost = 10
            elif map[x][y] == -1 or map[x][y] == -2:
                cost = 100
            elif map[x][y] == -3 or map[x][y] == -4:
                cost = 300
            elif map[x][y] == -5 or map[x][y] == -6:
                cost = 1000
            elif map[x][y] == -7 or map[x][y] == -8:
                cost = 20000
            elif map[x][y] == -9 or map[x][y] == -10:
                cost = 50000
            else:
                cost = 10000
                print("shit happened")
    
            if dx != 0 and dy != 0:
                cost *= math.sqrt(2)

            neighbors.append((x, y, cost))
            
    return neighbors 

def reconstruct_path(node):
    path = []
    current = node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]

def astar(map, start, goal):
    open_list = []
    closed_set = set()
    start_node = Node(start)
    start_node.heuristic = calculate_heuristic(start, goal)
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        current_position = current_node.position

        if current_position == goal:
            #print("Cost:", current_node.cost)
            return reconstruct_path(current_node)

        closed_set.add(current_position)
        
        neighbors = get_neighbors(current_position, map)
        
        for neighbor_position in neighbors:
            if neighbor_position[:2] in closed_set:
                continue

            neighbor_cost = current_node.cost + neighbor_position[2]
            neighbor_heuristic = calculate_heuristic(neighbor_position[:2], goal)
            neighbor_node = Node(
                neighbor_position[:2],
                parent=current_node,
                cost=neighbor_cost,
                heuristic=neighbor_heuristic
            )

            if any(node.position == neighbor_node.position for node in open_list):
            #if neighbor_node in open_list:
                existing_node = next(node for node in open_list if node.position == neighbor_node.position)
                #existing_node = open_list[open_list.index(neighbor_node)]
                if existing_node.cost > neighbor_cost:
                    existing_node.cost = neighbor_cost
                    existing_node.parent = current_node
                    existing_node.priority = neighbor_cost + existing_node.heuristic
                    heapq.heapify(open_list)
            else:
                heapq.heappush(open_list, neighbor_node)
    return None

def display_map(map, path):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(0, len(map[0]))
    ax.set_ylim(0, len(map))

    # Display the map
    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == -1:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='black'))
            elif map[i][j] == 1:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='white'))
            elif map[i][j] == 0:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='lightgray'))

    # Mark the path
    if path:
        path_x, path_y = zip(*path)
        ax.plot(path_y, path_x, marker='*', color='red')

    plt.show()

def generate_random_map(rows, cols, probabilities):
    map = []
    for _ in range(rows):
        row = []
        for _ in range(cols):
            value = random.choices([-1, 0, 1], probabilities)[0]
            row.append(value)
        map.append(row)
    map[0][0] = 1
    map[rows-1][cols-1] = 1

    return map

""""
# Example usage:
probabilities = [0.3,0.4,0.4]
map = generate_random_map(int(5.0/0.2), int(3.0/0.2),probabilities)
print(type(map))
print(map)
#
start = (0, 0)
goal = (2, 2)
#
#
path = astar(map, start, goal)
if path:
    print("Path found:", path)
    display_map(map, path)
else:
    print("Path not found!")
"""