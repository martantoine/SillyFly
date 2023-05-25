import numpy as np
import matplotlib.pyplot as plt
import math

# Global variables
global x_start, y_start, x_goal, y_goal
x_start = 0.0
y_start = 0.0
x_goal = 0.0
y_goal = 0.0
class AStarPlanner:
    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize grid map for a star planning
        x_start: x position of start point [m]
        y_start: y position of start point [m]
        x_goal: x position of goal point [m]
        y_goal: y position of goal point [m]
        resolution: grid resolution [m]
        robot_radius: robot radius [m]
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox,oy)

        self.dist_heuristic

        self.deltaX = 0
        self.deltaY = 0


    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def heuristic(self, nodeA, nodeB):
        self.deltaX = abs(nodeA.x - nodeB.x)
        self.deltaY = abs(nodeA.y - nodeB.y)

        if (self.deltaX > self.deltaY):
            result = 14*self.deltaY + 10*(self.deltaX-self.deltaY)
            return result
        result = 14*self.deltaX + 10*(self.deltaY-self.deltaX)
        return result

    # path planning function
    def planning(self, start_x, start_y, goal_x, goal_y):
        """
        A star path search
        input: start position x, y and goal position x, y
        
        output: path
        """
        # Initialize start and goal nodes
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                            self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                                self.calc_xy_index(goal_y, self.min_y), 0.0, -1)
        
        return
