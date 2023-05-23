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


class State(Enum): # state machine for the high-level control
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
        self.init_setpoints = [[x_landing, y_landing], [0.5, 0.1], [0.5, 0.5], [0.8, 0.5], [0.8, 0.0]] # test path
        #self.init_setpoints = [[x_landing, y_landing], [3.7, 0.1], [3.7, 2.9], [4.0, 2.9], [4.0, 0.1], [4.3, 0.1], [4.3, 2.9], [4.6, 2.9], [4.6, 0.1], [4.9, 0.1], [4.9,2.9]]
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

    def move(self, direction, sensor_data):
        """
        move the robot in the given direction

        input :
        direction : desired absolute direction [deg]
        face : if True, the robot will face the direction
        sensor_data : sensor data from the drone

        output :
        control_command : [v_forward, v_left, yaw_rate, height_desired]
        """

        global gait
        

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

        if plot:
            print(self.state)

        # if detects Q key, land and stop the program, must be the first condition in the function
        if self.state != State.EMERGENCY and keyboard.is_pressed('space'):
            if plot:
                print("EMERGENCY")
            self.state = State.EMERGENCY
            self.landing_found = True
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        # if the drone is in emergency state, stop the motors when the space key is released
        if self.state == State.EMERGENCY and not keyboard.is_pressed('space'):
            if plot:
                print("EMERGENCY END")
            control_command = [0.0, 0.0, 0.0, -1] # stop the motors
            return control_command
        
        # if the the drone is in END state, stop the motors
        if self.state == State.END:
            if plot_PID:#plot PID controller
                plt.plot(self.yaw_error_table, label="yaw error")
                plt.plot(self.yaw_prop_table, label="yaw prop")
                plt.plot(self.yaw_int_table, label="yaw int")
                plt.plot(self.yaw_der_table, label="yaw der")
                plt.plot(self.yaw_rate_table, label="yaw rate")
                plt.legend()
                plt.show()
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
                dir = 0.0
            elif self.count < 4*40:
                print("LEFT")
                dir = 90.0
            elif self.count < 6*40:
                print("BACK")
                dir = 180.0
            elif self.count < 8*40:
                print("RIGHT")
                dir = -90.0
            else:
                dir = 0.0
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
                self.state = State.ROTATE
                #self.state = State.TEST # for the test
                self.counter_runnning = True
                self.count = 0
        
        # Land
        if self.state == State.LANDING or self.state == State.EMERGENCY:
            #has landed
            if sensor_data['range.zrange'] < 20:
                print("ground reached")
                if self.landing_found: # end of the program
                    self.state = State.END

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
            
        #if state == ROTATE, rotate until the drone is facing the next setpoint
        if self.state == State.ROTATE:
            #compute yaw_rate to head in direction of the next setpoint
            yaw_rate = self.yaw_controller(np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone)), sensor_data)            
            if abs(yaw_rate) > 4:
                control_command = [0.0, 0.0, yaw_rate, self.height_desired]
                return control_command
            else:
                self.state = State.NEXT_SETPOINT
                self.count_pid = 0.0

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
            control_command = self.move(np.rad2deg(np.arctan2(y_goal - y_drone, x_goal - x_drone)), sensor_data)
            return control_command

        #default, hover
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command