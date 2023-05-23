# Super variables
mode = {DUMB, SILLY}
z-step = 0.05
z-threshold = 0.1
lateral-threshold = 0.1
lateral-speed = 0.2
angle-threshold = deg2rad(5)
landing-x-min = 3.5

# Global variables
## state
- enum state = {LIFT-OFF-1, GO-TO-LANDING-REGION, SCANNING-LANDING-PAD, LAND-1, TAKE-OFF-2, GO-TO-TAKE-OFF-PAD, LAND-2} : state machine
- int z-history[100] : record the last 100 z range measurement
- int z-counter = 0
- coord global-goal
- coord local-goals[] #dynamic sized array, size depend on the generated path by the A*

## preplanned-path
- int preplanned-path [n-step * 2]: [step0-x, step0-y, step1-x, step1-y, step2-x, step2-y, ...]
	space between each step should be smaller than 30cm (let's say 25cm)
	0     10 12 13     21
	1     9     12     20
	2     8     13     19
	3     7     14     18
	4  5  6     15  16 17

# struct map
- There are 2 maps: one for the obstacle (will be sourced from the example code) and one for the landing pad
## Obstacle map
- int obstacle[resX, resY]
- a cell = -1 means the cell is free
- a cell = 0  means don't know anything about the cell
- a cell >= 1 means the cell is blocked by an obstacle

## Landing pad map
- int explore-status[resX, resY]
- a cell = 0 means don't know if landing pad in it
- a cell >= 1:
	- means no landing pad in it
	- higher value means more certainty there is no landing pad inside in the cell
	- this value is increased every time the SillyFly fly over the cell (thus increase the certainty no pad is located in the cell)

# function [int, int] find-most-interesting-cell()
return most-interesting-cell = closest-cell-free-and-not-explored(map)

# function bool check-z()
z-mean = mean(z-history)
z-history[z-counter] = range-down()
if (z-mean - z-history[z-counter]) > some-threshold
	z-counter++
	return true
else
	z-counter++
	return false


# main loop
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
