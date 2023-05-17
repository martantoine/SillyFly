"""
Controller for the Crazyflie 2.1, using the high-level commander API.
modified from the example of the crazyflie lib

Modified by: Alec Parrat
last modified: 2023-05-17

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

plot = False

import numpy as np
from enum import Enum
import keyboard
from matplotlib import pyplot as plt

class State(Enum):
    ON_GROUND = 0
    LANDING = 1
    NEXT_SETPOINT = 2
    OBS_AVOID = 3
    ROTATE = 4
    END = 5
    EMERGENCY = 6
    TEST = 7

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.x_ini = 0
        self.y_ini = 0

        self.start = True
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

        # plot table
        self.yaw_error_table = []
        self.yaw_prop_table = []
        self.yaw_int_table = []
        self.yaw_der_table = []
        self.yaw_rate_table = []

        self.counter_runnning = False
        self.count = 0

        self.state = State.ON_GROUND

        self.setpoints = [[3.7, 0.1], [3.7, 2.9], [4.0, 2.9], [4.0, 0.1], [4.3, 0.1], [4.3, 2.9], [4.6, 2.9], [4.6, 0.1], [4.9, 0.1], [4.9,0.1]]
        self.index_current_setpoint = 0

        self.x_landing = 0
        self.y_landing = 0

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

        yaw_error = -(yaw_command - sensor_data['stabilizer.yaw'])
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

        if plot:
            print("yaw error : ", yaw_error, " | yaw rate : ", yaw_rate, " | yaw prop : ", yaw_prop, " | yaw int : ", yaw_int)
            # save error, prop, int, yaw_rate in table to plot at the end
            self.yaw_error_table.append(yaw_error)
            self.yaw_prop_table.append(yaw_prop)
            self.yaw_int_table.append(yaw_int)
            self.yaw_der_table.append(yaw_der)
            self.yaw_rate_table.append(yaw_rate)

        return yaw_rate

    def move(self, direction, face, sensor_data):
        """
        move the robot in the given direction

        input :
        direction : desired absolute direction [deg]
        face : if True, the robot will face the direction
        sensor_data : sensor data from the drone

        output :
        control_command : [v_forward, v_left, yaw_rate, height_desired]
        """
        
        if face:
            yaw_rate = self.yaw_controller(direction, sensor_data)
        else:
            yaw_rate = 0

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

        global plot

        # if detects Q key, land and stop the program, must be the first condition in the function
        if self.state != State.EMERGENCY and keyboard.is_pressed('space'):
            print("EMERGENCY")
            self.state = State.EMERGENCY
            self.landing_found = True
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # if the drone is in emergency state, stop the motors when the space key is released
        if self.state == State.EMERGENCY and not keyboard.is_pressed('space'):
            print("EMERGENCY END")
            control_command = [0.0, 0.0, 0.0, -1] # stop the motors
            return control_command
        
        # if the the drone is in END state, stop the motors
        if self.state == State.END:
            control_command = [0.0, 0.0, 0.0, -1] # stop the motors
            return control_command
        
        # If all setpoints were explored, explore them in the inverse order, to reach more of the space
        if self.index_current_setpoint == len(self.setpoints)-1:
            self.inverted = True

        if self.counter_runnning:
            self.count += 1

        # test state : move to a given position
        if self.state == State.TEST:
            if self.count < 2*40:
                print("FORWARD")
                dir = 0
            elif self.count < 4*40:
                print("LEFT")
                dir = 90
            elif self.count < 6*40:
                print("BACK")
                dir = 180
            elif self.count < 8*40:
                print("RIGHT")
                dir = -90
            else:
                dir = 0
                self.state = State.LANDING
                self.landing_found = True
                self.counter_runnning = False
                self.count = 0
                self.height_desired -= 0.005
                control_command = [0.0, 0.0, 0.0, self.height_desired]
            #print("counter : ", self.count, " | dir : ", dir, " | dir robot : ", sensor_data['stabilizer.yaw'])
            control_command = self.move(dir, True, sensor_data)
            return control_command

        # Take off
        if self.state == State.ON_GROUND:
            if sensor_data['range.zrange'] < 490:
                self.height_desired = self.flying_height
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
            else:
                #self.state = State.ROTATE
                self.state = State.TEST # for the test
                self.counter_runnning = True
                self.count = 0
                if self.start: # save the landing pad position
                    self.start = False
                    self.x_landing = sensor_data['stateEstimate.x'] + self.x_ini
                    self.y_landing = sensor_data['stateEstimate.y'] + self.y_ini
        
        # Land
        if self.state == State.LANDING or self.state == State.EMERGENCY:
            #has landed
            if sensor_data['range.zrange'] < 20:
                print("ground reached")
                if self.landing_found: # end of the program
                    self.state = State.END

                    #plot table
                    if plot:
                        plt.plot(self.yaw_error_table, label="yaw error")
                        plt.plot(self.yaw_prop_table, label="yaw prop")
                        plt.plot(self.yaw_int_table, label="yaw int")
                        plt.plot(self.yaw_der_table, label="yaw der")
                        plt.plot(self.yaw_rate_table, label="yaw rate")
                        plt.legend()
                        plt.show()

                    control_command = [0.0, 0.0, 0.0, -1] # -1 means stop motors
                    return control_command
                else: # landing pad found
                    self.state = State.ON_GROUND
                    self.landing_found = True
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        # Landing command

        #wait for the robot to be stable (count > 100), then wait the detection of the landing pad to send the landing order
        if self.state == State.NEXT_SETPOINT and not self.landing_found and self.count > 120 and sensor_data['range.zrange'] < 450:
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
            #compute yaw_rate to head in direction of the next setpoint
            yaw_rate = self.yaw_controller(np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone)), sensor_data['stabilizer.yaw'])            
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
            yaw_rate = self.yaw_controller(np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone)), sensor_data['stabilizer.yaw'])
            # go straight to the next setpoint
            control_command = [0.25, 0, yaw_rate, self.height_desired]
            return control_command

        #default, hover
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command