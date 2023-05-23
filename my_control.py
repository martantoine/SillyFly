# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np

time_step = 0 # à priori inutile

min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.2 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied



# forward state : state = 0
# check state : state = 1 
# yaw > 0 pour tourner à droite

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.state = 0
        self.obstacle = False
        self.x_init = 0
        self.y_init = 0
        self.dist_final_zone = 3.5
        self.final_zone = False
        self.land = False
        self.obstacle_front = False
        self.obstacle_back = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.y_max = 3 # range max terrain en y
        self.x_max = 5
        self.move_left = False
        self.move_right = False
        self.move_front = False
        self.move_back = False
        self.move_front_right = False
        self.take_off = False
        
    # Don't change the method name of 'step_control'
    def local_avoidance(self, sensor_data, desired_direction):
        control_command = [0.0, 0.0, 0.0, self.height_desired]
        
        if sensor_data['range_front'] < 0.2:
            self.obstacle_front = True
        elif sensor_data['range_back'] < 0.2:
            self.obstacle_back = True
        elif sensor_data['range_left'] < 0.2:
            self.obstacle_left = True
        elif sensor_data['range_right'] < 0.2:
            self.obstacle_right = True
        else:
            self.obstacle_front = False
            self.obstacle_back = False
            self.obstacle_left = False
            self.obstacle_right = False
        
        if desired_direction == 'front':
                    
            if self.obstacle_front:
                while (sensor_data['range_front'] < 0.3):
                    
                    if (sensor_data['range_left'] > sensor_data['range_right'] or 
                        sensor_data['range_left'] > sensor_data['y_global']):
                        
                        # go left
                        control_command = [0.0, 0.35, 0.0, self.height_desired]
                    elif (sensor_data['range_right'] > self.y_max - sensor_data['y_global']):
                        control_command = [0.0, -0.35, 0.0, self.height_desired]
                    else:
                    
                        # go right
                        control_command = [0.0, -0.35, 0.0, self.height_desired]
                    return control_command
                self.obstacle_front = False
            elif self.obstacle_right:
                control_command = [0.0, 0.2, 0.0, self.height_desired]
                return control_command
                self.obstacle_right = False
            elif self.obstacle_left:
                control_command = [0.0, -0.2, 0.0, self.height_desired]
                self.obstacle_left = False 
            else:
                #print('front')
                control_command = [0.3, 0.0, 0.0, self.height_desired]
            return control_command
    
        if desired_direction == 'left':
                    
            if self.obstacle_left:
                print('obstacle_left')
                while (sensor_data['range_left'] < 0.3):
                    
                    if (sensor_data['range_back'] > sensor_data['range_front'] or 
                        sensor_data['range_back'] > sensor_data['x_global']):
                        #----- potentiellelent x_globall à changer
                        # go back
                        control_command = [-0.35, 0.0, 0.0, self.height_desired]
                    elif (sensor_data['range_front'] > self.x_max - sensor_data['x_global']):
                        control_command = [0.35, 0.0, 0.0, self.height_desired]
                    else:
                    
                        # go front
                        control_command = [0.35, 0.0, 0.0, self.height_desired]
                    return control_command
                self.obstacle_left = False
            elif self.obstacle_front:
                print('obstacle_front')
                control_command = [0.2, 0.0, 0.0, self.height_desired]
                return control_command
                self.obstacle_front = False
            elif self.obstacle_back:
                print('obstacle_Back')
                control_command = [-0.2, 0.0, 0.0, self.height_desired]
                self.obstacle_bakc = False 
            else:
                #print('left')
                control_command = [0.0, 0.3, 0.0, self.height_desired]
            return control_command
        
        if desired_direction == 'right':
            if self.obstacle_right:
                while (sensor_data['range_right'] < 0.3):
                    
                    if (sensor_data['range_front'] > sensor_data['range_back'] or 
                        sensor_data['range_front'] > sensor_data['x_global']):
                        
                        # go front
                        control_command = [0.35, 0.0, 0.0, self.height_desired]
                    elif (sensor_data['range_back'] > self.x_max - sensor_data['x_global']):
                        control_command = [0.35, 0.0, 0.0, self.height_desired]
                    else:
                    
                        # go back
                        control_command = [-0.35, 0.0, 0.0, self.height_desired]
                    return control_command
                self.obstacle_right = False
            elif self.obstacle_back:
                control_command = [0.2, 0.0, 0.0, self.height_desired]
                return control_command
                self.obstacle_back = False
            elif self.obstacle_front:
                control_command = [-0.2, 0.0, 0.0, self.height_desired]
                self.obstacle_front = False 
            else:
                #print('right')
                control_command = [0.0, -0.3, 0.0, self.height_desired]
            return control_command
    
    
    
    
    
    def browse_area(self, sensor_data):
    
         # tant pas tout à gauche, je vais tout à gauche
            self.move_left = True
            if (self.y_max - sensor_data['y_global'] > 0.25
                and self.move_left
                and self.move_front == False 
                and self.move_right == False):
                control_command = self.local_avoidance(sensor_data, 'left')
                return control_command
                
            if (self.y_max - sensor_data['y_global'] <= 0.25 
                and self.move_front == False 
                and self.move_right == False):
                    self.x_init = sensor_data['x_global']
                    self.move_left = False
                    self.move_front = True
                        
            # Avance 
            if (sensor_data['x_global'] - self.x_init < 0.3 
                and self.move_front
                and self.move_front_right == False):
                control_command = self.local_avoidance(sensor_data, 'front')
                return control_command
                
            if sensor_data['x_global'] - self.x_init >= 0.3:
                self.move_front = False
                self.move_right = True
                self.move_left = False
                
            # Va tout à droite
            if (sensor_data['y_global'] > 0.2 and self.move_right):
                control_command = self.local_avoidance(sensor_data, 'right')
                return control_command
            
            if (sensor_data['y_global'] <= 0.2
                and self.move_front == False
                and self.move_left == False
                and self.move_front_right == False):
                self.x_init = sensor_data['x_global']
                self.move_right = False
                self.move_front = True    
                      
            # Now: implémenter avancer un peu
             
            if (sensor_data['x_global'] - self.x_init < 0.3 and self.move_front):
                control_command = self.local_avoidance(sensor_data, 'front')
                self.move_front_right = True
                return control_command
                
            if sensor_data['x_global'] - self.x_init >= 0.3:
                self.move_front = False
                self.move_right = False
                self.move_left = True
                self.move_front_right = False
                print('boum')
                
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
    
    
    
    
    def step_control(self, sensor_data):
        global time_step # à priori inutile
        time_step += 1 # à priori inutile
         
        self.z_pad = sensor_data['range_down']
        
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        else: 
            self.on_ground = False
             
        # Go to final zone 
        if (sensor_data['x_global'] < self.dist_final_zone and self.final_zone == False):
            command_control = self.local_avoidance(sensor_data,'front')
            return command_control
            
        # Arrived in final zone
        else:
            self.final_zone = True
            
            
            if sensor_data['range_down'] < (self.height_desired - 0.03 ):
                self.land = True # --> go to land 
                
            else:
                control_command = self.browse_area(sensor_data)
                
                
            # Land
            if self.land:
                self.height_desired -= 0.005
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                self.on_ground = False
                print(sensor_data['range_down'])
                print('HEIGHT ',self.height_desired)
                if self.height_desired < 0.02:
                    print('take off')
                    self.land = False
                    self.height_desired = 0.5
            
            
            
                
            return control_command
        # --------------------------- MAPPING -------------------------------------
        
        # Occupancy map based on distance sensor

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
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
    t +=1

    return map
        
        
            
        