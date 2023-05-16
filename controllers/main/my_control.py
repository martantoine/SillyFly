# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np
from enum import Enum

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
        self.x_ini = 0
        self.y_ini = 0

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

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):

        #if programm finished, do nothing
        if self.state == State.END:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # If all setpoints were explored, explore them in the inverse order, to reach more of the space
        if self.index_current_setpoint == len(self.setpoints)-1:
            self.inverted = True

        if self.counter_runnning:
            self.count += 1

        # Take off
        if self.state == State.ON_GROUND and sensor_data['range.zrange'] < 490:
            self.height_desired = self.flying_height
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        elif self.state == State.ON_GROUND:
            self.state = State.ROTATE
            self.counter_runnning = True
            self.count = 0
            if self.start: # save the landing pad position
                self.start = False
                self.x_landing = sensor_data['stateEstimate.x'] + self.x_ini
                self.y_landing = sensor_data['stateEstimate.y'] + self.y_ini
        
        # Land
        if self.state == State.LANDING:
            #has landed
            if sensor_data['range.zrange'] < 0.02:
                if self.landing_found: # end of the program
                    self.state = State.END
                else: # landing pad found
                    self.state = State.ON_GROUND
                    self.landing_found = True
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        # Landing command
        #wait for the robot to be stable (count > 100), then wait the detection of the landing pad to send the landing order
        if self.state == State.NEXT_SETPOINT and not self.landing_found and self.count > 120 and sensor_data['range.zrange'] < 0.45:
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

        x_drone, y_drone = sensor_data['stateEstimate.x'] + self.x_ini, sensor_data['stateEstimate.y'] + self.y_ini
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
            
        #if state == ROTATE, rotate until the drone is facing the next setpoint
        if self.state == State.ROTATE:
            diff_angle = np.arctan2(y_goal - y_drone, x_goal - x_drone) - sensor_data['stabilizer.yaw']
            if diff_angle > np.pi:
                diff_angle -= 2*np.pi
            elif diff_angle < -np.pi:
                diff_angle += 2*np.pi
            yaw_rate = self.yaw_p * diff_angle

            if yaw_rate > self.max_yaw_rate:
                yaw_rate = self.max_yaw_rate
            elif yaw_rate < -self.max_yaw_rate:
                yaw_rate = -self.max_yaw_rate
            
            if abs(yaw_rate) > 0.1:
                control_command = [0.0, 0.0, yaw_rate, self.height_desired]
                return control_command
            else:
                self.state = State.NEXT_SETPOINT

        #get proximity sensor data
        front_prox = sensor_data['range.front']
        left_prox = sensor_data['range.left']
        right_prox = sensor_data['range.right']
        
        # if state == NEXT_SETPOINT and a sensor detects an obstacle, go to OBS_AVOID state
        if self.state == State.NEXT_SETPOINT and (front_prox < 0.25 or left_prox < 0.2 or right_prox < 0.2):
            self.state = State.OBS_AVOID

        if self.state == State.OBS_AVOID:
            # check if the obstacle is avoided
            if front_prox > 0.25 and left_prox > 0.2 and right_prox > 0.2:
                self.state = State.ROTATE
                self.counter_runnning = True
                self.count = 0
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
            elif distance_drone_to_goal < 0.4: # if setpoint in obstacle, go to next setpoint
                if self.inverted:
                    self.index_current_setpoint -= 1
                else:
                    self.index_current_setpoint += 1
                self.state = State.ROTATE
                self.avoid_dir = 0
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
            elif front_prox < 0.2: # go back
                control_command = [-0.3, 0.0, 0.0, self.height_desired]
                return control_command
            else:
                if left_prox < 0.2 or right_prox < 0.2: # force to avoid new close obstacle
                    self.avoid_dir = 0
                # go perpendicular of the next setpoint, until the obstacle is avoided
                # always turn in the same direction for the same obstacle
                if self.avoid_dir == 0 and left_prox > right_prox:
                    self.avoid_dir = 1
                elif self.avoid_dir == 0:
                    self.avoid_dir = -1
                control_command = [0, self.avoid_dir * 0.3 , 0.0, self.height_desired]
                return control_command
            
        # direction command to next setpoint
        if self.state == State.NEXT_SETPOINT:
            #compute yaw_rate to head in direction of the next setpoint
            diff_angle = np.arctan2(y_goal - y_drone, x_goal - x_drone) - sensor_data['stabilizer.yaw']
            if diff_angle > np.pi:
                diff_angle -= 2*np.pi
            elif diff_angle < -np.pi:
                diff_angle += 2*np.pi
            yaw_rate = 2*self.yaw_p * diff_angle

            if yaw_rate > self.max_yaw_rate:
                yaw_rate = self.max_yaw_rate
            elif yaw_rate < -self.max_yaw_rate:
                yaw_rate = -self.max_yaw_rate
            # go straight to the next setpoint
            control_command = [0.25, 0, yaw_rate, self.height_desired]
            return control_command

        #default, hover
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command