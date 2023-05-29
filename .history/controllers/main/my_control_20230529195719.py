"""
Controller for the Crazyflie 2.1, using the high-level commander API.
modified from the example of the crazyflie lib

Modified by: Alec Parrat
last modified: 2023-05-23

units of sensor data:

stateEstimate.x : global x position [m]
stateEstimate.y : global y position [m]
stabilizer.yaw : yaw angle [deg]
range.front : distance to the front wall [mm]
range.back : distance to the back wall [mm]
range.left : distance to the left wall [mm]
range.right : distance to the right wall [mm]
range.zrange : distance to the ground [mm]

units of control commands:

forward speed : [m/s]
left speed : [m/s]
yaw rate : [deg/s]
height : [m]

counter : 40 =~ 1s
"""
import time
import math
import numpy as np
from enum import Enum
import keyboard
import matplotlib.pyplot as plt
from gpt4 import *
from Coord import *
import cv2

class State(Enum): # state machine for the high-level control
    TAKE_OFF_1 = 0
    GO_TO_LANDING_REGION = 1
    SCANNING_LANDING_PAD = 2
    LAND_1 = 3
    TAKE_OFF_2 = 4
    GO_TO_TAKE_OFF_PAD = 5
    LAND_2 = 6
    EMERGENCY = 7

class Mode(Enum): # state machine for the high-level control
    DUMB = 0
    SILLY = 1

class Gait(Enum): # gait of the robot
    STATIC = 0
    STRAIGHT = 1
    OSCILLATION = 2

"""
Super parameters
"""
gait = Gait.STATIC # Gait of the robot
mode = Mode.DUMB
lateral_threshold = 0.15 # in meters

map_min = Coord(0.0, 0.0)
map_max = Coord(5.0, 3.0)
map_res = 0.1
map_nx = int((map_max.x - map_min.x) / map_res)
map_ny = int((map_max.y - map_min.y) / map_res)
range_max = 2.0
conf = 1

plot = False # plot the yaw controller & other data
plot_path = False # plot the path of the robot
plot_PID = False # plot the PID controller

"""
Global variables
"""
t = 0
preplanned_path = []
obstacle_map = np.zeros((map_nx, map_ny)) # 0 = unknown, 1 = free, -1 = occupied

def find_x_positive_free_cell(current_pos, inflated_map): #TODO: implement this function
    tmp = np.zeros((1, map_ny))
    new_x = current_pos.x + 0.5
    for yi in range(map_ny):
        tmpi = Coord.dist(current_pos, Coord(current_pos.x, yi * map_res))
        if inflated_map[c2d(new_x), yi] < 0.0:
            tmpi += 1000.0
        tmp[0, yi] = tmpi
    return Coord(new_x, np.argmin(tmp) * map_res)
    #return current_pos + Coord(0.5, 0.0)

    
def find_most_interesting_cell(current_pos): #TODO: implement this function
    return Coord(0.0, 0.0)

def generate_preplanned_path(step_size, coord_min, coord_max, top, debug=False):
    """
    step_size [in] in m
    coord_min [in] bottom left corner
    coord_max [in] top right corner
    """
    global preplanned_path
    preplanned_path = []
    n_x = int((coord_max.x - coord_min.x) / step_size)
    n_y = int((coord_max.y - coord_min.y) / step_size)
    
    for xi in range(n_x + 1):
        if top:
            for yi in range(0, n_y, 1):
                preplanned_path.extend([coord_min + Coord(xi * step_size, yi * step_size)])
        else:
            for yi in range(n_y-1, -1, -1):
                preplanned_path.extend([coord_min + Coord(xi * step_size, yi * step_size)])
        top = not top
    
    if debug:
        print(map_nx, map_ny)
        for point in preplanned_path:
            print(c2d(point.x), c2d(point.y))

def c2d(continuous):
    return int(np.clip(continuous / map_res, 0, np.inf))

def c2dX(continuous):
    return int(np.clip(continuous / map_res, 0, map_nx-1))

def c2dY(continuous):
    return int(np.clip(continuous / map_res, 0, map_ny-1))

def d2c(discrete):
    return discrete * map_res

def array2Coord(array):
    print("2.75")
    coord_list = []
    #print(array)
    for a in array:
        coord_list.append(Coord(d2c(a[0]), d2c(a[1])))
    return coord_list    

def occupancy_map(current_pos, sensor_data):
    global obstacle_map
    yaw = np.deg2rad(sensor_data['stabilizer.yaw'])
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range.front']
        elif j == 1:
            measurement = sensor_data['range.left']
        elif j == 2:
            measurement = sensor_data['range.back']
        elif j == 3:
            measurement = sensor_data['range.right']
        
        for i in range(int(range_max / map_res)): # range is 2 meters
            dist = i * map_res
            idx_x = int(np.round((current_pos.x - map_min.x + dist*np.cos(yaw_sensor)) / map_res, 0))
            idx_y = int(np.round((current_pos.y - map_min.y + dist*np.sin(yaw_sensor)) / map_res, 0))

            # make sure the point is within the map
            if idx_x < 0 or idx_x >= obstacle_map.shape[0] or idx_y < 0 or idx_y >= obstacle_map.shape[1] or dist > range_max:
                break

            # update the map
            if dist * 1000.0 < measurement:
                if obstacle_map[idx_x, idx_y] != -1: 
                    obstacle_map[idx_x, idx_y] += conf
            else:
                obstacle_map[idx_x, idx_y] -= conf
                break
    
    obstacle_map = np.clip(obstacle_map, -1, 1) # certainty can never be more than 100%
    return obstacle_map

class MyController():
    def __init__(self):
        self.start_pos    = Coord(0.75, 1.5)
        self.current_pos  = Coord(0.0, 0.0)
        self.global_goals = []
        self.state = State.TAKE_OFF_1
        self.i = 0
        self.landing_border = []
        self.zA_hist = np.zeros(50)
        self.zA_i = 0
        self.zB_hist = np.zeros(2)
        self.zB_i = 0
        self.edge_counter = 0
        self.edge = [Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0), Coord(0.0, 0.0)]
        self.pos_landing = 0
        generate_preplanned_path(0.25, Coord(3.5, 0.0), map_max, True, debug=False)

        self.flying_height = 0.4 # m
        self.height_desired = 0
        self.avoid_dir = 0
        self.yaw_p = 0.4
        self.yaw_i = 1.1
        self.yaw_d = 0.5
        self.yaw_err_prev = 0.0
        self.yaw_i_prev = 0.0
        self.max_yaw_rate = 2*360.0
        self.speed = 0.3
        self.current_z = 0
        self.current_yaw = 0.0

        # if gait == OSCILLATION, the robot will oscillate around the desired direction
        self.amp_oscillation = 30 # [deg]
        self.freq_oscillation = 1.0 # [Hz]
        self.count_pid = 0.0

        self.count = 0
        self.pop_count = 0
        self.prev_vx = 0.0
        self.prev_vy = 0.0

        #build table of 100 values
        self.values_100 = []
        self.values_10 = []
        self.sum_100 = 0
        self.sum_10 = 0

    def moving_avergage(self, new_value):
        #save new value
        self.values_100.append(new_value)
        self.values_10.append(new_value)

        self.sum_100 += new_value
        self.sum_10 += new_value

        #compute avergage
        average_100 = self.sum_100 / 100
        average_10 = self.sum_10 / 10

        #remove last value if list is full
        if len(self.values_100) == 100:
            self.sum_100 -= self.values_100[0]
            self.values_100.pop(0)
        if len(self.values_10) == 10:
            self.sum_10 -= self.values_10[0]
            self.values_10.pop(0)

        return average_100, average_10
        

    def yaw_controller(self, yaw_command, sensor_data):
        """
        PID controller for yaw

        input :
        yaw_command : desired yaw angle [deg]
        sensor_data : sensor data from the drone

        output :
        yaw_rate : yaw rate command [deg/s]
        """
        global plot

        yaw_error = - (yaw_command - sensor_data['stabilizer.yaw'])
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        yaw_prop = self.yaw_p * yaw_error
        yaw_int = self.yaw_i_prev + self.yaw_i * yaw_error
        yaw_der = self.yaw_d * (yaw_error - self.yaw_err_prev)

        # limit integral term
        if yaw_int > self.max_yaw_rate:
            yaw_int = self.max_yaw_rate
        elif yaw_int < -self.max_yaw_rate:
            yaw_int = -self.max_yaw_rate

        yaw_rate = yaw_prop + yaw_int + yaw_der

        # limit yaw rate
        if yaw_rate > self.max_yaw_rate:
            yaw_rate = self.max_yaw_rate
        elif yaw_rate < -self.max_yaw_rate:
            yaw_rate = -self.max_yaw_rate

        if plot_PID:
            #print("yaw error : ", yaw_error, " | yaw rate : ", yaw_rate, " | yaw prop : ", yaw_prop, " | yaw int : ", yaw_int)
            # save error, prop, int, yaw_rate in table to plot at the end
            self.yaw_error_table.append(yaw_error)
            self.yaw_prop_table.append(yaw_prop)
            self.yaw_int_table.append(yaw_int)
            self.yaw_der_table.append(yaw_der)
            self.yaw_rate_table.append(yaw_rate)

        return yaw_rate

    def move(self, sensor_data):
        """
        move the robot in the given direction

        input :
        sensor_data : sensor data from the drone

        output :
        control_command : [v_forward, v_left, yaw_rate, height_desired]
        """

        global gait, x_landing, y_landing
        
        direction = Coord.angle(self.current_pos, self.global_goals[0])
        dir2face = 0.0

        if gait == Gait.STATIC:
            #dir2face = np.deg2rad(self.current_yaw)
            dir2face = 0.0
        elif gait == Gait.STRAIGHT:
            dir2face = direction
        elif gait == Gait.OSCILLATION:
            dir2face = direction + np.deg2rad(self.amp_oscillation) * np.sin(2*np.pi*self.freq_oscillation*self.count_pid)
            self.count_pid += 0.02

        yaw_rate = self.yaw_controller(np.rad2deg(dir2face), sensor_data)

        # compute the forward & left speed of the robot according to the desired direction and the current yaw
        direction_robot = np.rad2deg(direction) - sensor_data['stabilizer.yaw']
        while direction_robot > 180:
            direction_robot -= 360
        while direction_robot < -180:
            direction_robot += 360
        v_forward = np.cos(np.deg2rad(direction_robot)) * self.speed
        v_left = np.sin(np.deg2rad(direction_robot)) * self.speed

        control_command = [v_forward, v_left, yaw_rate, self.height_desired]
        return control_command

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        """
        Main control loop, called at each timestep
        Handle the state machine for the high-level control

        Emergency stop : hold space to land slowly, release to stop the motors
        """
        global mode
        global gait
        global lateral_threshold
        global preplanned_path
        global obstacle_map
        global map_res
        global t
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        
        if self.start_pos.x == -1.0:
            self.start_pos = Coord(sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']) + Coord(0.0, 0.0)
            control_command = [vx, vy, vyaw, self.height_desired]
            return control_command
        else:
            self.current_pos = Coord(sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']) + self.start_pos
            #self.current_pos = Coord(sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y'])
        
        self.zA_hist[self.zA_i] = sensor_data['range.zrange']
        self.zA_i += 1
        if self.zA_i == 50:
            self.zA_i = 0
        
        self.zB_hist = sensor_data['range.zrange']
        self.zB_i += 1
        if self.zB_i == 2:
            self.zB_i = 0

        self.previous_z = self.current_z
        self.current_z = sensor_data['range.zrange']
        
        obstacle_map = occupancy_map(self.current_pos, sensor_data)
        obstacle_map_masked = np.copy(obstacle_map)
        for x in range(len(obstacle_map_masked)):
            for y in range(len(obstacle_map_masked[0])):
                if obstacle_map_masked[x][y] != -1:
                    obstacle_map_masked[x][y] = 0
                else:
                    obstacle_map_masked[x][y] = 1
        inflated_mapA = cv2.dilate(obstacle_map_masked, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
        inflated_mapB = cv2.dilate(obstacle_map_masked, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
        inflated_mapC = cv2.dilate(obstacle_map_masked, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        inflated_mapD = cv2.dilate(obstacle_map_masked, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        inflated_map = obstacle_map_masked + inflated_mapA + inflated_mapB + inflated_mapC + inflated_mapD
        inflated_map = np.clip(-2 * inflated_map + obstacle_map, -10, 1)
        

        z_big_mean = np.mean(self.zA_hist)
        z_small_mean = np.mean(self.zB_hist)
        diff = abs(z_big_mean - z_small_mean)
        
        # if detects Q key, land and stop the program, must be the first condition in the function
        if keyboard.is_pressed('space'):
            self.state = State.EMERGENCY        

        #print(self.state.name)

        match self.state:
            case State.EMERGENCY:
                if keyboard.is_pressed('space'):
                    self.height_desired -= 0.005
                else:
                    self.height_desired = -1 # stop the motors
                if plot:
                    print("EMERGENCY END")

            case State.TAKE_OFF_1:
                if sensor_data['range.zrange'] < 390:
                    self.height_desired = self.flying_height
                else:
                    print("take off 1 completed, current pos: ", self.current_pos, "start pos: ", self.start_pos)
                    self.global_goals = []
                    self.state = State.GO_TO_LANDING_REGION #go landing region

            case State.GO_TO_LANDING_REGION:
                if self.global_goals and self.pop_count <= 7: # check if the array is not empty or too long since last check

                    if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                        vx, vy, vyaw, _ = self.move(sensor_data)
                    else:
                        self.global_goals.pop(0) #delete the first element
                        self.pop_count += 1
                else:
                    self.pop_count = 0
                    if self.current_pos.x > 3.5:
                        self.state = State.SCANNING_LANDING_PAD
                    else:
                        tmp = find_x_positive_free_cell(self.current_pos, inflated_map)
                        obs_tmp = inflated_map.astype(int).tolist()  
                        self.global_goals = array2Coord(astar(obs_tmp, (c2dX(self.current_pos.x), c2dY(self.current_pos.y)), (c2dX(tmp.x), c2dY(tmp.y))))

            case State.SCANNING_LANDING_PAD:
                if diff > 60:
                    print("go to landing 1")
                    self.global_goals = []
                    self.state = State.LAND_1
                else:   
                    if self.global_goals: # check if the array is not empty
                        if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                            vx, vy, vyaw, _ = self.move(sensor_data)
                        else:
                            self.global_goals.pop(0) #delete the first element
                    else:
                        if mode == Mode.SILLY:
                            tmp = find_most_interesting_cell()
                        elif mode == Mode.DUMB:
                            if self.i >= len(preplanned_path):
                                self.i = 0
                            while inflated_map[c2dX(preplanned_path[self.i].x), c2dY(preplanned_path[self.i].y)] != 1:
                                self.i += 1 # skip if cell blocked by obstacle
                                if self.i >= len(preplanned_path):
                                    self.i = 0
                                print("skipped preplanned point cause blocked by obstacle")

                            tmp = preplanned_path[self.i]
                        
                            self.i += 1 # increment counter
                        obs_tmp = inflated_map.astype(int).tolist()
                        print("2.5")
                        self.global_goals = array2Coord(astar(obs_tmp, (c2dX(self.current_pos.x), c2dY(self.current_pos.y)), (c2dX(tmp.x), c2dY(tmp.y))))

            case State.LAND_1:
                gait = Gait.STATIC
                self.speed = 0.2
                if not self.global_goals:
                    if self.edge_counter == 0:
                        for edge in self.edge: # security if don't detect an edge
                            edge = self.current_pos
                        self.pos_landing = self.current_pos
                        self.global_goals = [Coord(-0.4, 0.0) + self.pos_landing]
                    elif self.edge_counter == 1:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 2:
                        self.global_goals = [Coord(0.4, 0.0) + self.pos_landing]
                    elif self.edge_counter == 3:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 4:
                        self.global_goals = [Coord(0.0, -0.4) + self.pos_landing]
                    elif self.edge_counter == 5:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 6:
                        self.global_goals = [Coord(0.0, 0.4) + self.pos_landing]
                    elif self.edge_counter == 7:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                        self.speed = 0.1
                    elif self.edge_counter == 8:
                        self.speed == 0.05
                        self.global_goals = [Coord((self.edge[0].x + self.edge[1].x + self.edge[2].x + self.edge[3].x) / 4.0, (self.edge[4].y + self.edge[5].y + self.edge[6].y + self.edge[7].y) / 4.0)]
                else:
                    if Coord.dist(self.current_pos, self.global_goals[0]) > 0.02: 
                        vx, vy, vyaw, _ = self.move(sensor_data)
                        print("state: ", self.edge_counter)
                        if abs(self.previous_z - self.current_z) > 7: #15
                            self.edge[self.edge_counter] = self.current_pos
                            print("state: ", self.edge_counter, "current pos: ", self.current_pos, "goal: ", self.global_goals[0])
                    else:
                        if self.edge_counter == 8:
                            if sensor_data['range.zrange'] < 20:
                                self.state = State.TAKE_OFF_2
                                self.edge_counter = 0
                                #gait = Gait.STRAIGHT
                                self.global_goals = []
                                self.height_desired = -2
                                print("landing 1 completed")
                            else:    
                                self.height_desired -= 0.002
                        else:
                            self.global_goals = []
                            self.edge_counter += 1
                    
            case State.TAKE_OFF_2:
                if sensor_data['range.zrange'] < 390:
                    self.height_desired = self.flying_height
                else:
                    self.state = State.GO_TO_TAKE_OFF_PAD
                    self.global_goals = []
                    self.speed = 0.3
                    print("take off 2 completed")
                    
            case State.GO_TO_TAKE_OFF_PAD:
                if Coord.dist(self.current_pos, self.start_pos) > lateral_threshold:
                    if self.global_goals: # check if the array is not empty
                        if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                            vx, vy, vyaw, _ = self.move(sensor_data)
                        else:
                            print( "current pos: ",self.current_pos, " astar ", self.global_goals[0]) 
                            self.global_goals.pop(0) #delete the first element

                    else:
                        obs_tmp = inflated_map.astype(int).tolist()   
                        self.global_goals = array2Coord(astar(obs_tmp, (c2dX(self.current_pos.x), c2dY(self.current_pos.y)),
                                                (c2dX(self.start_pos.x), c2dY(self.start_pos.y))))
                else:
                    self.state = State.LAND_2

            case State.LAND_2:
                print("land2")
                gait = Gait.STATIC
                if not self.global_goals:
                    if self.edge_counter == 0:
                        for edge in self.edge: # security if don't detect an edge
                            edge = self.current_pos
                        self.pos_landing = self.current_pos
                        self.speed = 0.2
                        self.pos_landing = self.current_pos
                        self.global_goals = [Coord(-0.4, 0.0) + self.pos_landing]
                    elif self.edge_counter == 1:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 2:
                        self.global_goals = [Coord(0.4, 0.0) + self.pos_landing]
                    elif self.edge_counter == 3:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 4:
                        self.global_goals = [Coord(0.0, -0.4) + self.pos_landing]
                    elif self.edge_counter == 5:
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 6:
                        self.global_goals = [Coord(0.0, 0.4) + self.pos_landing]
                    elif self.edge_counter == 7:
                        self.speed = 0.1
                        self.global_goals = [Coord(0.0, 0.0) + self.pos_landing]
                    elif self.edge_counter == 8:
                        self.global_goals = [Coord((self.edge[0].x + self.edge[1].x + self.edge[2].x + self.edge[3].x) / 4.0, (self.edge[4].y + self.edge[5].y + self.edge[6].y + self.edge[7].y) / 4.0)]
    
                else:
                    if Coord.dist(self.current_pos, self.global_goals[0]) > 0.02: 
                        vx, vy, vyaw, _ = self.move(sensor_data)
                        if abs(self.previous_z - self.current_z) > 7:
                            self.edge[self.edge_counter] = self.current_pos
                            print("state: ", self.edge_counter, "current pos: ", self.current_pos, "goal: ", self.global_goals[0])
                    else:
                        if self.edge_counter == 8:
                            if sensor_data['range.zrange'] < 20:
                                self.global_goals = []
                                self.height_desired = -1
                                print("landing 2 completed")
                            else:    
                                self.height_desired -= 0.002
                        else:
                            self.global_goals = []
                            self.edge_counter += 1
        
        if t % 100 == 0:
            obstacle_map_drone = np.copy(obstacle_map)
            inflated_map_drone = np.copy(inflated_map)
            inflated_map_drone[np.clip(c2d(self.current_pos.x), 0, map_nx-1)][np.clip(c2d(self.current_pos.y), 0, map_ny-1)] = 2 
            astar_map_drone = np.copy(obstacle_map)
            astar_map_drone[np.clip(c2d(self.current_pos.x), 0, map_nx-1)][np.clip(c2d(self.current_pos.y), 0, map_ny-1)] = 3
            for small_goal in self.global_goals:
                astar_map_drone[np.clip(c2d(small_goal.x), 0, map_nx-1)][np.clip(c2d(small_goal.y), 0, map_ny-1)] = 2
            i = 0
            for yi in range(map_ny):
                obstacle_map_drone[c2d(3.5)][yi] = 2
            for small_goal in preplanned_path:
                obstacle_map_drone[np.clip(c2d(small_goal.x), 0, map_nx-1)][np.clip(c2d(small_goal.y), 0, map_ny-1)] = 1 + i
                i *= 2
            plt.imsave("map.png", np.flip(obstacle_map_drone.astype(int), 1), vmin=-1, vmax=3, cmap='gray', origin='lower')
            plt.imsave("inflated.png", np.flip(inflated_map_drone.astype(int), 1), vmin=-10, vmax=2, cmap='gray', origin='lower')
            plt.imsave("inflated_raw.png", np.flip(inflated_map.astype(int), 1), vmin=-10, vmax=1, cmap='gray', origin='lower')
            plt.imsave("astar.png", np.flip(astar_map_drone.astype(int), 1), vmin=-1, vmax=3, cmap='gray', origin='lower')
        t +=1

        #if gait != Gait.STATIC:
        #    self.current_yaw = sensor_data['stabilizer.yaw']
        control_command = [vx, vy, vyaw, self.height_desired]
        return control_command