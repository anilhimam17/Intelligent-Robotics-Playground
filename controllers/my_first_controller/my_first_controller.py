"""my_first_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:

    # Constructor for the controller
    def __init__(self, robot):
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1 # M/s 
        
        # Configuring all the motors of the robot
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        
        # Allowing the motors to operate for infinite distance
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        
        # Initialising the velocity of the motors
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Instance variables to control the velocity of the motors
        self.velocity_left = 0
        self.velocity_right = 0
        
        # --------------------------------------------------------------------
        
        # Configuring the Proximity Sensors
        self.proximity_sensors = []
        
        # All the sensors are present on the robot in clockwise direction
        for i in range(8):
            sensor_name = "ps" + str(i)
            self.proximity_sensors.append(
                self.robot.getDevice(sensor_name)
            )
            self.proximity_sensors[i].enable(self.time_step)
            
        # --------------------------------------------------------------------
        
        # Configuring the Ground Sensors (3)
        self.left_ir = self.robot.getDevice("gs0")
        self.center_ir = self.robot.getDevice("gs1")
        self.right_ir = self.robot.getDevice("gs2")
        
        self.left_ir.enable(self.time_step)
        self.center_ir.enable(self.time_step)
        self.right_ir.enable(self.time_step)
        
        # --------------------------------------------------------------------
        
        # Data Inputs
        self.inputs = []
        self.inputsPrevious = []
        self.flag_turn = 0
        
        
    # Constraints for the Speed
    def clip_value(self, value, min_max):
        if value > min_max:
            return min_max
        elif value < -min_vax:
            return -min_max
        return value
        
        
    # Sensor computing and movement
    def sense_compute_actuate(self):
    
        # Checking if the robot has started collecting information
        if len(self.inputs) > 0 and len(self.inputsPrevious) > 0:
        
            # Checking for a possible collision using all the proximity sensors with a timestamp from detection
            if np.max(self.inputs[3:11]) > 2000:
                time = datetime.now()
                print(f"{time.second} - {time.microsecond}: Object or Wall was detected")
            
            # Checking if the robot is about to execute a turn; Makeing it turn right
            if self.flag_turn:
                self.velocity_left = -0.3
                self.velocity_right = 0.3
                
                # If no more collisions detected reset the turn flag
                if np.min(self.inputs[:3]) < 0.35:
                    self.flag_turn = 0
                    
            else:
                # Checking for the end of the Line
                if (np.min(self.inputs[:3]) - np.min(self.inputsPrevious[:3])) > 0.2:
                    self.flag_turn = 1
                    
                # Following the Line
                else:
                    
                    # Line detected on the left side 
                    if self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]:
                        self.velocity_left = 0.5
                        self.velocity_right = 1
                        
                    # Line detected on the center
                    elif self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]:
                        self.velocity_left = 1
                        self.velocity_right = 1
                        
                    # Line detected on the right side
                    elif self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]:
                        self.velocity_left = 1
                        self.velocity_right = 0.5
        
        # Transmitting the velocity to the motors based on the sensing
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
    
    
    # Automating the robot main loop
    def run_robot(self):
        count = 0
        inputs_avg = []
        
        while self.robot.step(self.time_step) != -1:
            
            # Initialising the list of input values for each Time Step
            self.inputs = []
            
            # Acquiring the Readings from the Ground Sensors
            left_ground = self.left_ir.getValue()
            center_ground = self.center_ir.getValue()
            right_ground = self.right_ir.getValue()
            
            # Calibrating the Readings from the Ground Sensors
            min_val = 0
            max_val = 1000
            
            if left_ground > max_val: left_ground = max_val
            if center_ground > max_val: center_ground = max_val
            if right_ground > max_val: right_ground = max_val
            if left_ground < min_val: left_ground = min_val
            if center_ground < min_val: center_ground = min_val
            if right_ground < min_val: right_ground = min_val
            
            # Storing the Readings from the Ground Sensors after scaling to between 0 and 1
            self.inputs.append((left_ground - min_val) / (max_val - min_val))
            self.inputs.append((center_ground - min_val) / (max_val - min_val))
            self.inputs.append((right_ground - min_val) / (max_val - min_val))
            
            # --------------------------------------------------------------------
            
            # Acquiring the Readings from the Proximity Sensors
            for i in range(8):
                if i in [0, 1, 2, 5, 6, 7]:
                    sensor_read = self.proximity_sensors[i].getValue()
                    
                    # Calibrating and Scaling the Values
                    min_ds = 0
                    max_ds = 2400
                    if sensor_read > max_ds: sensor_read = max_ds
                    if sensor_read < min_ds: sensor_read = min_ds
                    
                    self.inputs.append((sensor_read - min_ds) / (max_ds - min_ds))
                    
            # --------------------------------------------------------------------
            
            # Smoothening the inputs and actuation with a filter
            smooth = 30
            if (count == smooth):
                
                # Taking the sum of all the inputs from each sensor over different time_steps
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x / smooth for x in inputs_avg]
                
                # Computing Movement based on Sensor Input
                self.sense_compute_actuate()
                
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
                
            else:
                inputs_avg.append(self.inputs)
                count += 1
                
                
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()












