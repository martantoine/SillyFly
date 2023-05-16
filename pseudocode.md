# Super variables
mode = {DUMB, SILLY}

# Global variables
## state
- enum state = {LIFT-OFF, GO-TO-LANDING-REGION, SCANNING-LANDING-PAD} : state machine
- int z-history[100] : record the last 100 z range measurement
- int z-counter = 0

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
switc state
	case TAKE-OFF-1
		
	case TRAVEL-TO-LANDING-REGION

	case SCANNING-LANDING-PAD
		if goal-reached == True
			if mode == SILLY
				cell-to-explore = find-most-interesting-cell()
			else if mode == DUMB
				while obstacle[preplanned-path[i].coords] != -1
					i++ : skip if cell blocked by obstacle
				cell-to-explore = preplanned-path[i]
			i++ increment the counter for future step
			goal-reached = False
		if dist(cell-to-explore, current-pos) < 10cm
			goal-reached = True
	case LAND-1

	case TAKE-OFF-2

	case TRAVEL-TO-TAKE-OFF-PAD
	
	CASE LAND-2

if check-z()
	state = LAND
