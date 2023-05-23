import numpy as np
import matplotlib.pyplot as plt
import math

# Global variables
global x_start, y_start, x_goal, y_goal
x_start = 0.0
y_start = 0.0
x_goal = 0.0
y_goal = 0.0

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x  # index of grid
        self.y = y  # index of grid
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(
            self.cost) + "," + str(self.parent_index)

def heuristic(nodeA, nodeB):
    deltaX = abs(a.x - b.x)
    deltaY = abs(a.y - b.y)

    if (deltaX > deltaY):
        return 14*deltaY + 10*(deltaX-deltaY)
    return  14*deltaX + 10*(deltaY-deltaX)

# path planning function
def planning(self, start_x, start_y, goal_x, goal_y):
    """
    A star path search
    input: start position x, y and goal position x, y
    
    output: path
    """
    # Initialize start and goal nodes
    start_node = Node(self.calc_xy_index(start_x, self.min_x),
                          self.calc_xy_index(start_y, self.min_y), 0.0, -1)
    goal_node = Node(self.calc_xy_index(goal_x, self.min_x),
                            self.calc_xy_index(goal_y, self.min_y), 0.0, -1)
    
    return
