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

class Gait(Enum): # gait of the robot
    STATIC = 0
    STRAIGHT = 1
    OSCILLATION = 2

"""
Global parameters to change
"""
x_landing, y_landing = 0.0, 0.0 # Initial landing pad position
gait = Gait.STRAIGHT # Gait of the robot

plot = False # plot the yaw controller & other data
plot_path = False # plot the path of the robot
plot_PID = False # plot the PID controller

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

class State(Enum): # state machine for the high-level control
    TAKE_OFF_1 = 0
    GO_TO_LANDING_REGION = 1
    SCANNING_LANDING_PAD = 2
    LAND_1 = 3
    TAKE_OFF_2 = 4
    GO_TO_TAKE_OFF_PAD = 4
    LAND_2 = 5
    EMERGENCY = 6

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        global x_landing, y_landing

        self.inverted =  False
        self.landing_found = False

        self.flying_height = 0.5 # m
        self.height_desired = 0
        self.avoid_dir = 0
        self.yaw_p = 0.4
        self.yaw_i = 1.1
        self.yaw_d = 0.5
        self.yaw_err_prev = 0.0
        self.yaw_i_prev = 0.0
        #self.yaw_d = 2.0
        #self.yaw_rate_prev = 0.0
        self.max_yaw_rate = 2*360.0
        self.speed = 0.3

        # if gait == OSCILLATION, the robot will oscillate around the desired direction
        self.amp_oscillation = 30 # [deg]
        self.freq_oscillation = 1.0 # [Hz]
        self.count_pid = 0.0

        # plot table
        self.yaw_error_table = []
        self.yaw_prop_table = []
        self.yaw_int_table = []
        self.yaw_der_table = []
        self.yaw_rate_table = []

        self.counter_runnning = False
        self.count = 0

        self.state = State.ON_GROUND

        # initial path planning (no obstacle known)
        self.max_distance_setpoints = 0.25
        #self.init_setpoints = [[x_landing, y_landing], [0.5, 0.1], [0.5, 0.5], [0.8, 0.5], [0.8, 0.0]] # test path
        self.init_setpoints = [[x_landing, y_landing], [3.7, 0.1], [3.7, 2.9], [4.0, 2.9], [4.0, 0.1], [4.3, 0.1], [4.3, 2.9], [4.6, 2.9], [4.6, 0.1], [4.9, 0.1], [4.9,2.9]]
        self.setpoints = self.path_interpolate(self.init_setpoints) # adding sethpoints to have less than max_distance_setpoints between each setpoint
        self.index_current_setpoint = 0

    def path_interpolate(self, init_setpoints):
        """
        Given the initial setpoints, add new setpoints in between
        The space between two setpoints must be <= max_distance_setpoints
        """
        final_setpoints = []
        for current in init_setpoints:
            if current != init_setpoints[0]: # exept first setpoint 
                # check distance with previous setpoint
                delta_x = current[0] - previous[0]
                delta_y = current[1] - previous[1]
                dist = np.linalg.norm([delta_x, delta_y])
                if dist > self.max_distance_setpoints:
                    n = np.floor(dist / self.max_distance_setpoints).astype(int) # compute number of setpoints to add
                    interval = dist / (n+1) # optimal interval to put between each new setpoint
                    dir = np.arctan2(delta_y, delta_x) # direction between the setpoints (radians)
                    interval_x = interval * np.cos(dir)
                    interval_y = interval * np.sin(dir)
                    for j in range(n): # add new setpoints
                        final_setpoints.append([previous[0] + (j+1)*interval_x, previous[1] + (j+1)*interval_y])

            final_setpoints.append(current) # also add current setpoint, even for the first one
            previous = current # store the current setpoint for comparison with the next

        # remove the first setpoint (useless)
        final_setpoints.pop(0)
        
        if plot_path : # print the path + interpolated setpoints
            x = []
            y = []
            for i in final_setpoints:
                x.append(i[0])
                y.append(i[1])
            plt.plot(x,y, marker='x')
            plt.show()

        return final_setpoints

    def yaw_controller(self, yaw_command ,sensor_data):
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

    def move(self, x_goal, y_goal, sensor_data):
        """
        move the robot in the given direction

        input :
        x_goal : x coordinate of the goal
        y_goal : y coordinate of the goal
        sensor_data : sensor data from the drone

        output :
        control_command : [v_forward, v_left, yaw_rate, height_desired]
        """

        global gait, x_landing, y_landing
        
        x_drone = sensor_data['stateEstimate.x'] + x_landing
        y_drone = sensor_data['stateEstimate.y'] + y_landing
        direction = np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone))

        if gait == Gait.STATIC:
            dir2face = 0.0
        elif gait == Gait.STRAIGHT:
            dir2face = direction
        elif gait == Gait.OSCILLATION:
            dir2face = direction + self.amp_oscillation * np.sin(2*np.pi*self.freq_oscillation*self.count_pid)
            self.count_pid += 0.02
            print("dir2face : ", dir2face, " | count : ", self.count_pid)
        else:
            print("ERROR : unknown gait")

        yaw_rate = self.yaw_controller(dir2face, sensor_data)

        # compute the forward & left speed of the robot according to the desired direction and the current yaw
        direction_robot = direction - sensor_data['stabilizer.yaw']
        if direction_robot > 180:
            direction_robot -= 360
        elif direction_robot < -180:
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
        global plot, x_landing, y_landing
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        
        

        if plot:
            print(self.state)
        
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
                if Coord.dist(current_pos, local_goal) > lateral_threshold:
                       if self.state == State.NEXT_SETPOINT:
                        control_command = self.move(np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone)), sensor_data)
                    command.yaw = angle(current-pos, local-goal) + sin(step) * 20 / 90
                    if(angle(current-pos, local-goal) < angle-threshold):
                        command.vx = lateral-speed
                else:
                    if(current-pos-x > landing-x-min):
                        state = SCANNING-LANDING-PAD
                    else:
                        if(!global-goals.empty()):
                            local-goal = global-goals[0]
                            global-goals.pop_first()
                        else:
                            global-goal = find-x-positive-free-cell()
                            globals-path = a-star(current-pos, global-goal)
            
            case State.SCANNING_LANDING_PAD:
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
            
            case State.LAND_1:
                if(current-pos.z > some-threshold)
                    command.z = current-pos.z - z-step
                else
                    state = TAKE-OFF-2

            case State.LAND_2:
                if sensor_data['range.zrange'] < 20: # True if has landed
                    self.height_desired = -1 # stop the motors
                    print("ground reached")
                else:    
                    self.height_desired -= 0.005
            
            if self.counter_runnning:
                self.count += 1
        
        control_command = [vx, vy, vyaw, self.height_desired]

        #wait for the robot to be stable (count > 100), then wait the detection of the landing pad to send the landing order
        if self.state == State.NEXT_SETPOINT and not self.landing_found and self.count > 120 and sensor_data['range.zrange'] < 470:
            self.state = State.LANDING
            self.counter_runnning = False
            self.count = 0
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # Get the goal position and drone position to compute ditance to setpoint
        if self.landing_found:
            x_goal, y_goal = x_landing, y_landing
        else:
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]

        x_drone, y_drone = sensor_data['stateEstimate.x'] + x_landing, sensor_data['stateEstimate.y'] + y_landing
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])

        print("x : ", x_drone, " | y : ", y_drone, " | next_point : ", x_goal, ", ", y_goal)

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
            # if the gait is static, no need to rotate, when oscillation, don't want to rotate
            if gait == Gait.STATIC or gait == Gait.OSCILLATION:
                self.state = State.NEXT_SETPOINT
                self.count_pid = 0.0
            else:
                self.state = State.ROTATE
    

        #get proximity sensor data
        front_prox = sensor_data['range.front']
        left_prox = sensor_data['range.left']
        right_prox = sensor_data['range.right']
            
        # direction command to next setpoint
        if self.state == State.NEXT_SETPOINT:
            control_command = self.move(x_goal, y_goal, sensor_data)
            return control_command

        #default, hover
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command