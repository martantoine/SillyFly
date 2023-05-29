# Main simulation file called by the Webots

import numpy as np
from pid_control import pid_velocity_fixed_height_controller
from my_control import MyController
import time, random

import logging
from threading import Timer
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E709')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Set 'True' to enable random positions of obstacles and the drone
enable_random_environment = True
# Set seed to replicate the random environment
# random.seed(3000)

class Logging:        
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self):
        """ Initialize and run the example with the specified link_uri """

        global uri

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._sensor_reading = {}

        print('Connecting to %s' % uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.zrange')

        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        t = Timer(50, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg): 
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        for name, value in data.items():
            #print(f'{name}: {value:3.3f} ', end='')
            self._sensor_reading[name] = value
        #self._sensor_reading = data.items()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':

    cflib.crtp.init_drivers()


    le = Logging()

    # Initialize the drone
    drone = le._cf

    drone.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    drone.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)


    my_controller = MyController()

    control_commands = [0, 0, 0, 0]

    # Simulation loops
    for step in range(100000):

        # Read sensor data
        sensor_data = le._sensor_reading

        # data received : conection successful
        if sensor_data != {}: 
            #print("height :", sens            # Control commands with [v_forward, v_left, yaw_rate, altitude]
            # ---- Select only one of the following control methods ---- #
            # control_commands = drone.action_from_keyboard()or_data['range.zrange'])

            #print("command", control_commands)
            # control_commands = example.obstacle_avoidance(sensor_data)
            # control_commands = example.path_plan ning(sensor_data)
            # map = example.occupancy_map(sensor_data)
            # ---- end --- #

            #print(sensor_data['range_down'])

            # Update the drone status in simulation
            # drone.step(control_commands, sensor_data)

            # Send control commands to the drone
            control_commands = my_controller.step_control(sensor_data)
            if control_commands[3] == -2:
                drone.commander.send_stop_setpoint()
                time.sleep(1)
            elif control_commands[3] == -1:
                drone.commander.send_stop_setpoint()
                break
            else:
                drone.commander.send_hover_setpoint(control_commands[0], control_commands[1], control_commands[2], control_commands[3])    

        time.sleep(0.01)

        # Grading function based on drone states and world information