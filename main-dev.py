import math
import numpy as np
from enum import Enum

class Coord:
    COUNT = 0

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __str__(self):
        return "Point(%s,%s)"%(self.x, self.y) 

    def dist(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx**2 + dy**2)

    def angle(self, other):
        angle = np.arctan2((other.y - self.y), (other.x - self.x))
        return angle

    def test():
        '''Returns a point and distance'''
        p1 = Coord(1.0, 1.0)
        print(p1)
        p2 = Coord(1.0,-1.0)
        print(p2)
        print(np.rad2deg(Coord.angle(p1, p2)))
        print(p1.dist(p2))
        
# Super parameters
z_step = 0.05
z_threshold = 0.1
lateral_threshold = 0.1
lateral_speed = 0.2
angle_threshold = np.deg2rad(5)
landing_x_min = 3.5

# Global variables
mode = Enum('SCANNING_MODE', ['DUMB', 'SILLY'])
state = Enum('STATE', ['LIFT_OFF_1', 'GO_TO_LANDING_REGION', 'SCANNING_LANDING_PAD', 'LAND_1', 'TAKE_OFF_2', 'GO_TO_TAKE_OFF_PAD', 'LAND_2'])
z_history = np.zeros((100, 1)) #record the last 100 z range measurement
z_counter = 0
global_goal = Coord()
local_goals = [] #dynamic sized array, size depend on the generated path by the A*

# main loop
Coord.test()
""""
command = {0, 0, 0, 0} #vx, vy, yaw, z ||||| default command

if(z-desired - current-pos.z < some-threshold)
	command.z = current-pos.z + z-step
elif(current-pos.z - z-desired < some-threshold)
	command.z = current-pos.z - z-step
		
switc state
	case TAKE-OFF-1
		if(abs(current-pos.z - z-desired) < some-threshold)
			state = GO-TO-LANDING-REGION
	case GO-TO-LANDING-REGION
		if(dist(current-pos, local-goal) > lateral-threshold
			command.yaw = angle(current-pos, local-goal) + sin(step) * 20 / 90
			if(angle(current-pos, local-goal) < angle-threshold)
				command.vx = lateral-speed
		else
			if(current-pos-x > landing-x-min)
				state = SCANNING-LANDING-PAD
			else
				if(!global-goals.empty())
					local-goal = global-goals[0]
					global-goals.pop_first()
				else
					global-goal = find-x-positive-free-cell()
					globals-path = a-star(current-pos, global-goal)
	case SCANNING-LANDING-PAD
		if dist(local-goal, current-pos) < lateral-threshold
			goal-reached = True
			if(!global-goals.empty())
				local-goal = global-goals[0]
				global-goals.pop_first()
			else
				if mode == SILLY
					global-goal = find-most-interesting-cell()
				else if mode == DUMB
					while obstacle[preplanned-path[i].coords] != -1
						i++ : skip if cell blocked by obstacle
					global-goal = preplanned-path[i]
					i++ increment the counter for future step
				globals-path = a-star(current-pos, global-goal)
		else
			if(dist(current-pos, path-goal) > lateral-threshold
				command.yaw = angle(current-pos, path-goal) + sin(step) * 20 / 90
				if(angle(current-pos, path-goal) < angle-threshold)
					command.vx = lateral-speed
			else
				if(current-pos-x > landing-x-min)
					state = SCANNING-LANDING-PAD
			else
				path-goal = find-x-positive-free-cell()
	case LAND-1
		if(current-pos.z > some-threshold)
			command.z = current-pos.z - z-step
		else
			state = TAKE-OFF-2
	case TAKE-OFF-2

	case GO-TO-TAKE-OFF-PAD
	
	CASE LAND-2

if check-z()
	state = LAND
"""