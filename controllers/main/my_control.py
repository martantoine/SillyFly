# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np
from enum import Enum
import matplotlib.pyplot as plt
import keyboard

plot = False # set to True to plot the map

# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.1 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the point is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if plot and t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
    t +=1

    return map

class State(Enum):
    ON_GROUND = 0
    LANDING = 1
    NEXT_SETPOINT = 2
    OBS_AVOID = 3
    ROTATE = 4
    END = 5

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):

        self.speed = 0.2 # speed of the drone
        self.yaw_p = 1  # proprtional yaw control
        self.yaw_i = 0.2 # integral yaw control
        self.yaw_i_prev = 0.0 # previous integral term
        self.max_yaw_rate = 2.0 # maximum yaw rate
        self.SEARCH_RADIUS = 0.8 # radius of the search area around the drone
        self.map_avoid_dist = 0.3 # distance to obstacle to trigger the avoidance
        self.exit_angle = np.deg2rad(2) # angle to exit the OBS_AVOID state
        self.goal_dist_obstacle_limit = 0.5 # tolerance if goal is inside the obstacle avoidance area
        self.prox_avoid_dist = 0.15 # distance to obstacle to trigger the avoidance
        self.goal_dir_init = 0 # direction of the goal setpoint when entering the OBS_AVOID state
        self.avoid_dir = 0 # 0 = no obstacle, 1 = obstacle on the left, -1 = obstacle on the right
        self.yaw_amp = np.deg2rad(15) # amplitude of the yaw oscillation
        self.yaw_freq = 3 # frequency of the yaw oscillation
        #self.yaw_limit = np.deg2rad(30) # limit of the yaw oscillation
        self.t = 0 # time of the yaw oscillation

        self.start = True
        self.inverted =  False
        self.landing_found = False

        self.flying_height = 0.5
        self.height_desired = 0
        self.avoid_dir = 0
        self.yaw_p = 8.0
        #self.yaw_d = 2.0
        #self.yaw_rate_prev = 0.0
        self.max_yaw_rate = 4.0

        self.counter_runnning = False
        self.count = 0

        self.state = State.ON_GROUND

        self.setpoints = [[3.7, 0.1], [3.7, 2.9], [4.0, 2.9], [4.0, 0.1], [4.3, 0.1], [4.3, 2.9], [4.6, 2.9], [4.6, 0.1], [4.9, 0.1], [4.9,0.1]]
        self.index_current_setpoint = 0

        self.x_landing = 0
        self.y_landing = 0

    def yaw_PI(self, yaw_error):
        yaw_p = self.yaw_p*yaw_error
        yaw_i = self.yaw_i_prev + self.yaw_i*yaw_error

        # limit the integral term
        if yaw_i > self.max_yaw_rate:
            yaw_i = self.max_yaw_rate
        elif yaw_i < -self.max_yaw_rate:
            yaw_i = -self.max_yaw_rate

        self.yaw_i_prev = yaw_i

        yaw_rate = yaw_p + yaw_i

        # limit the yaw rate
        if yaw_rate > self.max_yaw_rate:
            yaw_rate = self.max_yaw_rate
        elif yaw_rate < -self.max_yaw_rate:
            yaw_rate = -self.max_yaw_rate
        return yaw_rate

    # function that makes the drone move in a given direction while oscilating around the direction
    def move(self, x, y, sensor_yaw):
        # compute the direction of the drone
        direction_map = np.arctan2(y, x)
        direction_robot = direction_map - sensor_yaw
        # compute the speed of the drone
        x_command = self.speed * np.cos(direction_robot)
        y_command = self.speed * np.sin(direction_robot)
        # compute the yaw angle to follow
        yaw = direction_map + self.yaw_amp * np.sin(2*np.pi*self.yaw_freq*self.t)
        self.t += 0.01
        yaw_error = yaw - sensor_yaw
        yaw_rate = self.yaw_PI(yaw_error)
        control_command = [x_command, y_command, yaw_rate, self.height_desired]
        return control_command
        
    # returns the direction and distance of the closest 1 on the map, returns distance 0 if no 1 closest than RADIUS
    def closest_obstacle(self, sensor_data):
        global map
        shortest_dist = 100
        x_closest, y_closest = 0, 0
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        # create a 0 matrix of the same size as the map
        dist_map = np.zeros(map.shape)
        # fill the matrix with the distance to the drone, find the closest obstacle
        for i in range (map.shape[0]):
            for j in range (map.shape[1]):
                dist_map[i,j] = np.sqrt((i*res_pos - x_drone)**2 + (j*res_pos - y_drone)**2)
                # if the distance is shorter than the radius and map value is to 1, keep the value, else set to 0
                if dist_map[i,j] < self.SEARCH_RADIUS and map[i,j] == -1.0: # it's not logical, should work with map[i,j] == 1.0
                    # if the distance is shorter than the shortest distance, update the shortest distance and save the coordinates
                    #dist_map[i,j] = dist_map[i,j]
                    if shortest_dist > dist_map[i,j]:
                        shortest_dist = dist_map[i,j]
                        x_closest = i*res_pos
                        y_closest = j*res_pos
                else:
                    dist_map[i,j] = 0

        if plot and t % 50 == 0:
            plt.imshow(np.flip(dist_map,1), vmin=0, vmax=0.5, cmap='gray', origin='lower') # flip the map to match the coordinate system
            plt.savefig("dist_map.png")

        # if no obstacle is found, return 0,100
        if shortest_dist == 100:
            return 0, shortest_dist
        else :
            # compute the direction of the closest obstacle
            direction_obs = np.arctan2(y_closest - y_drone, x_closest - x_drone)
            return direction_obs, shortest_dist

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        global map

        # if detects Q key, land and stop the program
        if keyboard.is_pressed('q'):
            self.state = State.LANDING
            self.landing_found = True
            self.counter_runnning = False
            self.count = 0
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command

        #if programm finished, do nothing
        if self.state == State.END:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # If all setpoints were explored, explore them in the inverse order, to reach more of the space
        if self.index_current_setpoint >= len(self.setpoints)-1:
            self.inverted = True

        if self.counter_runnning:
            self.count += 1

        # Take off
        if self.state == State.ON_GROUND and sensor_data['range_down'] < 0.49:
            self.height_desired = self.flying_height
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        elif self.state == State.ON_GROUND:
            self.state = State.ROTATE
            self.t = 0
            self.counter_runnning = True
            self.count = 0
            if self.start: # save the landing pad position
                self.start = False
                self.x_landing = sensor_data['x_global']
                self.y_landing = sensor_data['y_global']
        
        # Land
        if self.state == State.LANDING:
            #has landed
            if sensor_data['range_down'] < 0.02:
                if self.landing_found: # end of the program
                    self.state = State.END
                else: # landing pad found
                    self.state = State.ON_GROUND
                    self.landing_found = True
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        # Start landing
        # Wait for the robot to be stable (count > 100), then wait the detection of the landing pad to send the landing order
        if self.state == State.NEXT_SETPOINT and not self.landing_found and self.count > 120 and sensor_data['range_down'] < 0.45:
            self.state = State.LANDING
            self.counter_runnning = False
            self.count = 0
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # Get the goal position and drone position to compute ditance to setpoint
        if self.landing_found:
            x_goal, y_goal = self.x_landing, self.y_landing
        else:
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]

        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])

        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
        if (self.state == State.NEXT_SETPOINT or self.state == State.OBS_AVOID) and distance_drone_to_goal < 0.1:
            # Select the next setpoint as the goal position
            if self.landing_found: # for the last landing
                self.state = State.LANDING
                self.counter_runnning = False
                self.count = 0
                self.height_desired -= 0.005
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
            elif self.inverted:
                self.index_current_setpoint -= 1
            else:
                self.index_current_setpoint += 1
            self.avoid_dir = 0
            self.state = State.ROTATE
            self.t = 0

        #get proximity sensor data
        # front_prox = sensor_data['range_front']
        # back_prox = sensor_data['range_back']
        # left_prox = sensor_data['range_left']
        # right_prox = sensor_data['range_right']

        print(self.state, self.avoid_dir)

        # emergency avoidance if too close to an obstacle
        if sensor_data['range_front'] < self.prox_avoid_dist or sensor_data['range_back'] < self.prox_avoid_dist or sensor_data['range_left'] < self.prox_avoid_dist or sensor_data['range_right'] < self.prox_avoid_dist:
            if sensor_data['range_front'] < self.prox_avoid_dist:
                x_command = -1
            elif sensor_data['range_back'] < self.prox_avoid_dist:
                x_command = 1
            else:
                x_command = 0

            if sensor_data['range_left'] < self.prox_avoid_dist:
                y_command = -1
            elif sensor_data['range_right'] < self.prox_avoid_dist:
                y_command = 1
            else:
                y_command = 0

            control_command = [x_command*self.speed/2, y_command*self.speed/2, 0, self.height_desired]
            return control_command

        # if state == NEXT_SETPOINT or OBS_AVOID, update map & check if obstacle is close
        if self.state == State.NEXT_SETPOINT or self.state == State.ROTATE or self.state == State.OBS_AVOID:
            map = occupancy_map(sensor_data)
            dir_obstacle, dist_obstacle = self.closest_obstacle(sensor_data)
            goal_direction = np.arctan2(y_goal - y_drone, x_goal - x_drone)


        if self.state == State.OBS_AVOID:
            #if goal is close to the obstacle, go to the next setpoint
            if distance_drone_to_goal < self.goal_dist_obstacle_limit:
                if self.inverted:
                    self.index_current_setpoint -= 1
                else:
                    self.index_current_setpoint += 1
                self.state = State.ROTATE
                self.t = 0
                self.avoid_dir = 0

            # check if current goal direction is equal to the initial goal direction
            print("angle difference :", np.abs(goal_direction - self.goal_dir_init))
            if np.abs(goal_direction - self.goal_dir_init) < self.exit_angle/2:
                # exit OBS_AVOID state
                self.state = State.NEXT_SETPOINT
                self.avoid_dir = 0
            else:
                # go 90° left or right of the obstacle
                x_command = np.cos(dir_obstacle + self.avoid_dir*np.pi/2)
                y_command = np.sin(dir_obstacle + self.avoid_dir*np.pi/2)
                control_command = self.move(x_command, y_command, sensor_data['yaw'])
                return control_command

        # if state == NEXT_SETPOINT & close map obstacle & obstacle in front of drone -> OBS_AVOID
        if (self.state == State.NEXT_SETPOINT or self.state == State.ROTATE) and dist_obstacle < self.map_avoid_dist and np.abs(dir_obstacle - goal_direction) < np.pi/2:
            self.state = State.OBS_AVOID
            # save current goal direction for later
            self.goal_dir_init = goal_direction

            # see if vector of closest obstacle is left or right of goal vector
            if dir_obstacle - goal_direction > 0:
                self.avoid_dir = -1
            else:
                self.avoid_dir = 1

            # go 90° left or right of the obstacle
            x_command = np.cos(dir_obstacle + self.avoid_dir*np.pi/2)
            y_command = np.sin(dir_obstacle + self.avoid_dir*np.pi/2)
            control_command = self.move(x_command, y_command, sensor_data['yaw'])
            return control_command
        
        # if state == ROTATE, rotate until yaw is close to the goal direction
        if self.state == State.ROTATE:
            # if the yaw is close to the goal direction, exit ROTATE state
            yaw_error = goal_direction - sensor_data['yaw']
            yaw_rate = self.yaw_PI(yaw_error)
            if np.abs(yaw_error) < self.exit_angle:
                self.state = State.NEXT_SETPOINT
                self.avoid_dir = 0
            else:
                # rotate in the direction of the goal
                control_command = [0.0, 0.0, yaw_rate, self.height_desired]
                return control_command
            
        # direction command to next setpoint
        if self.state == State.NEXT_SETPOINT:
            x_command = x_goal - x_drone
            y_command = y_goal - y_drone
            control_command = self.move(x_command, y_command, sensor_data['yaw'])
            return control_command

        #default, hover
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command