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

import math
import numpy as np
from enum import Enum
import keyboard
from matplotlib import pyplot as plt

class Coord:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __str__(self):
        return "Point(%s,%s)"%(self.x, self.y) 

    def __add__(self, other):
        return Coord(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Coord(self.x - other.x, self.y - other.y)
    
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
        print(p1 + p2)
        print(p1 - p2)
        print(np.rad2deg(Coord.angle(p1, p2)))
        print(Coord.dist(p1, p2))

class State(Enum): # state machine for the high-level control
    TAKE_OFF_1 = 0
    GO_TO_LANDING_REGION = 1
    SCANNING_LANDING_PAD = 2
    LAND_1 = 3
    TAKE_OFF_2 = 4
    GO_TO_TAKE_OFF_PAD = 4
    LAND_2 = 5
    EMERGENCY = 6

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
gait = Gait.STRAIGHT # Gait of the robot
mode = Mode.DUMB
lateral_threshold = 0.1 # in meters

map_min = Coord(0.0, 0.0)
map_max = Coord(5.0, 3.0)
map_res = 0.25
map_nx = int((map_max.x - map_min.x) / map_res)
map_ny = int((map_max.y - map_min.y) / map_res)
range_max = 2
conf = 1

plot = False # plot the yaw controller & other data
plot_path = False # plot the path of the robot
plot_PID = False # plot the PID controller

"""
Global variables
"""
preplanned_path = []
obstacle_map = np.zeros((map_nx, map_ny)) # 0 = unknown, 1 = free, -1 = occupied

def find_x_positive_free_cell(current_pos): #TODO: implement this function
    return Coord(0.0, 0.0)

def find_most_interesting_cell(current_pos): #TODO: implement this function
    return Coord(0.0, 0.0)

def a_star(current_pos, goal): #TODO: implement this function
    """
    current_pos [in]: Coord type object, the start point of the path finding algorithm
    goal [in]: Coord type object, the end point of the path finding algorithm
    Coord[]: an array of coord, the result of the path finding algorithm
    """
    return []

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
            for yi in range(0, n_y + 1, 1):
                preplanned_path.extend([coord_min + Coord(xi * step_size, yi * step_size)])
        else:
            for yi in range(n_y, -1, -1):
                preplanned_path.extend([coord_min + Coord(xi * step_size, yi * step_size)])
        top = not top
    
    if debug:
        for point in preplanned_path:
            print(point)

def occupancy_map(current_pos, sensor_data):
    global obstacle_map
    yaw = sensor_data['stabilizer.yaw']
    
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
        
        for i in range(int(range_max / map_res)): # range is 2 meters
            dist = i * map_res
            idx_x = int(np.round((current_pos.x - map_min.x + dist*np.cos(yaw_sensor)) / map_res, 0))
            idx_y = int(np.round((current_pos.y - map_min.y + dist*np.sin(yaw_sensor)) / map_res, 0))

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
    return map

class MyController():
    def __init__(self):
        self.current_pos  = Coord(0.0, 0.0)
        self.local_goal   = Coord(0.0, 0.0)
        self.global_goals = []
        self.state = State.TAKE_OFF_1

        generate_preplanned_path(0.25, Coord(3.5, 0.0), Coord(5.0, 3.0), True)

        self.flying_height = 0.5 # m
        self.height_desired = 0
        self.avoid_dir = 0
        self.yaw_p = 0.4
        self.yaw_i = 1.1
        self.yaw_d = 0.5
        self.yaw_err_prev = 0.0
        self.yaw_i_prev = 0.0
        self.max_yaw_rate = 2*360.0
        self.speed = 0.3

        # if gait == OSCILLATION, the robot will oscillate around the desired direction
        self.amp_oscillation = 30 # [deg]
        self.freq_oscillation = 1.0 # [Hz]
        self.count_pid = 0.0

        self.count = 0
        

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
            print("yaw error : ", yaw_error, " | yaw rate : ", yaw_rate, " | yaw prop : ", yaw_prop, " | yaw int : ", yaw_int)
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
        
        direction = Coord.angle(self.current_pos, self.local_goal)
        
        if gait == Gait.STATIC:
            dir2face = 0.0
        elif gait == Gait.STRAIGHT:
            dir2face = direction
        elif gait == Gait.OSCILLATION:
            dir2face = direction + self.amp_oscillation * np.sin(2*np.pi*self.freq_oscillation*self.count_pid)
            self.count_pid += 0.02

        yaw_rate = self.yaw_controller(dir2face, sensor_data)

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
        global lateral_threshold
        global obstacle_map
        global map_res
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        
        self.current_pos = Coord(sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y'])
        occupancy_map(self.current_pos, sensor_data)

        
        # if detects Q key, land and stop the program, must be the first condition in the function
        if keyboard.is_pressed('space'):
            self.state = State.EMERGENCY        

        match self.state:
            case State.EMERGENCY:
                if keyboard.is_pressed('space'):
                    self.height_desired -= 0.005
                else:
                    self.height_desired = -1 # stop the motors
                if plot:
                    print("EMERGENCY END")

            case State.TAKE_OFF_1:
                if sensor_data['range.zrange'] < 490:
                    self.height_desired = self.flying_height
                else:
                    self.state = State.GO_TO_LANDING_REGION

            case State.GO_TO_LANDING_REGION:
                if self.global_goals: # check if the array is not empty
                    if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                        vx, vy, vyaw, _ = self.move(sensor_data)
                    else:
                        self.global_goals.pop(0) #delete the first element
                else:
                    if self.current_pos.x > 3.5:
                        self.state = State.SCANNING_LANDING_PAD
                    else:
                        self.globals_path = a_star(self.current_pos, find_x_positive_free_cell(self.current_pos))
            
            case State.SCANNING_LANDING_PAD:
                if self.global_goals: # check if the array is not empty
                    if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                        vx, vy, vyaw, _ = self.move(sensor_data)
                    else:
                        self.global_goals.pop(0) #delete the first element
                else:
                    if mode == Mode.SILLY:
                        tmp = find_most_interesting_cell()
                    elif mode == Mode.DUMB:
                        while obstacle_map[preplanned_path[i].x / map_res, preplanned_path[i].y / map_res] != -1:
                            i += 1 # skip if cell blocked by obstacle
                        tmp = preplanned_path[i]
                        i += 1 #increment the counter for future step
                    self.globals_path = a_star(self.current_pos, tmp)

            case State.LAND_1:
                if sensor_data['range.zrange'] < 20: # True if has landed
                    self.state = State.TAKE_OFF_2
                    print("ground reached")
                else:    
                    self.height_desired -= 0.005

            case State.GO_TO_TAKE_OFF_PAD:
                if self.global_goals: # check if the array is not empty
                    if Coord.dist(self.current_pos, self.global_goals[0]) > lateral_threshold:
                        vx, vy, vyaw, _ = self.move(sensor_data)
                    else:
                        self.global_goals.pop(0) #delete the first element
                else:
                    self.globals_path = a_star(self.current_pos, Coord(0.0, 0.0))

            case State.LAND_2:
                if sensor_data['range.zrange'] < 20: # True if has landed
                    self.height_desired = -1 # stop the motors
                    print("ground reached")
                else:    
                    self.height_desired -= 0.005
        
        control_command = [vx, vy, vyaw, self.height_desired]