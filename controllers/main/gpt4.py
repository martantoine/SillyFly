import heapq
import math
import matplotlib.pyplot as plt
#import random
from Coord import *

class Node:
    def __init__(self, position, parent=None, cost=0, heuristic=0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.priority = cost + heuristic

    def __lt__(self, other):
        return self.priority < other.priority or (self.priority == other.priority and self.cost > other.cost)

def calculate_heuristic(current, goal):
    dx = abs(current.x - goal.x)
    dy = abs(current.y - goal.y)
    return min(dx, dy) * 14 + abs(dx - dy) * 10  

def get_neighbors(current, map):
    neighbors = []
    directions = [Coord(1, 0), Coord(-1, 0), Coord(0, 1), Coord(0, -1), Coord(1, 1), Coord(1, -1), Coord(-1, 1), Coord(-1, -1)]
    
    for d in directions:
        xy = current + d
        if 0 <= xy.x < len(map) and 0 <= xy.y < len(map[0]) and map[xy.x][xy.y] != -1:
            
            if map[xy.x][xy.y] == 1:
                cost = 1
            if map[xy.x][xy.y] == 0:
                cost = 2
    
            if d.x != 0 and d.y != 0:
                cost *= math.sqrt(2)

            neighbors.append(Coord(xy.x, xy.y), cost)
            
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
            print("Cost:", current_node.cost)
            return reconstruct_path(current_node)

        closed_set.add(current_position)
        
        neighbors = get_neighbors(current_position, map)
        
        for neighbor_position in neighbors:
            if neighbor_position[0] in closed_set:
                continue

            neighbor_cost = current_node.cost + neighbor_position[1]
            neighbor_heuristic = calculate_heuristic(neighbor_position[0], goal)
            neighbor_node = Node(
                neighbor_position[0],
                parent=current_node,
                cost=neighbor_cost,
                heuristic=neighbor_heuristic
            )

            if neighbor_node in open_list:
                existing_node = open_list[open_list.index(neighbor_node)]
                if existing_node.cost > neighbor_cost:
                    existing_node.cost = neighbor_cost
                    existing_node.parent = current_node
                    existing_node.priority = neighbor_cost + existing_node.heuristic
                    heapq.heapify(open_list)
            else:
                heapq.heappush(open_list, neighbor_node)

    return None

#def display_map(map, path):
#    fig, ax = plt.subplots()
#    ax.set_aspect('equal')
#    ax.set_xlim(0, len(map[0]))
#    ax.set_ylim(0, len(map))
#
#    # Display the map
#    for i in range(len(map)):
#        for j in range(len(map[0])):
#            if map[i][j] == -1:
#                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='black'))
#            elif map[i][j] == 1:
#                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='white'))
#            elif map[i][j] == 0:
#                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='lightgray'))
#
#    # Mark the path
#    if path:
#        path_x, path_y = zip(*path)
#        ax.plot(path_y, path_x, marker='*', color='red')
#
#    plt.show()

#def generate_random_map(rows, cols, probabilities):
#    map = []
#    for _ in range(rows):
#        row = []
#        for _ in range(cols):
#            value = random.choices([-1, 0, 1], probabilities)[0]
#            row.append(value)
#        map.append(row)
#    map[0][0] = 1
#    map[rows-1][cols-1] = 1

#    return map

# Example usage:
#probabilities = [0.3,0.4,0.4]
#map = generate_random_map(30,15,probabilities)
#
#start = (0, 0)
#goal = (29, 14)
#
#
#path = astar(map, start, goal)
#if path:
#    print("Path found:", path)
#    display_map(map, path)
#else:
#    print("Path not found!")