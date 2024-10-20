from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:

    # Constructor
    def __init__(self, robot):
        self.robot = robot
        self.time_step = 32
        self.max_speed = 1
        
        # Configuring and Enabling the Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Motor Velocity Variables
        self.velocity_left = 0
        self.velocity_right = 0
        
        # Configuring and Enabling the Distance Sensors
        self.distance_sensors = []
        for i in range(8):
            sensor_name = "ps" + str(i)
            self.distance_sensors.append(
                self.robot.getDevice(sensor_name)
            )
            self.distance_sensors[i].enable(self.time_step)
            
        # Configuring and Enabling the Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = "ls" + str(i)
            self.light_sensors.append(
                self.robot.getDevice(sensor_name)
            )
            self.light_sensors[i].enable(self.time_step)
            
        # Configuring and Enabling the Ground Sensors
        self.left_ground = self.robot.getDevice("gs0")
        self.center_ground = self.robot.getDevice("gs1")
        self.right_ground = self.robot.getDevice("gs2")
        
        self.left_ground.enable(self.time_step)
        self.center_ground.enable(self.time_step)
        self.right_ground.enable(self.time_step)
        
        # Data Storage properties
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag to keep track of turns
        self.flag_turn = 0
        
    # Value Constraint
    def clip_value(self, value, min_max):
        if value < -min_max:
            return -min_max
        elif value > min_max:
            return min_max
        return value
        
    # Sensor Compute and Actuation
    def sense_compute_actuate(self):
    
        # Checking if the robot has began collecting information
        if len(self.inputs) > 0:
        
            # Checking for a collision, Last 7 inputs correspond to Distance Sensors
            if np.max(self.inputs[11:]) > 0.4:
                time = datetime.now()
                self.velocity_left = 0
                self.velocity_right = 0
                print(f"{time.second} - {time.microsecond}: Detected a Wall or Obstacle")
                
            # Continue with Light Tracking
            else:
                
                # The robot is in the center of the light path
                if self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0:
                    self.velocity_left = 1
                    self.velocity_right = 1
                    
                # The Light source is to the right of the robot
                elif self.inputs[5] == 0:
                    self.velocity_left = 1
                    self.velocity_right = 0.5
                    
                # The Light source is to the left of the robot
                elif self.inputs[8] == 0:
                    self.velocity_left = 0.5
                    self.velocity_right = 1
                    
                # The Light source is behind the robot
                elif self.inputs[6] == 0 and self.inputs[7] == 0:
                    self.velocity_left = 1
                    self.velocity_right = -1
                    
        # Transmitting the Computed Velocity to the Motors
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        
    # Automating the Behaviour of the Robot
    def run_robot(self):
        count = 0
        inputs_avg = []
        
        min_ls = 0
        max_ls = 4300
        
        # Main looping the program of the robot
        while self.robot.step(self.time_step) != -1:
        
            # Resetting the Values of the Inputs for each Iteration
            self.inputs = []
            
            # Acquiring the Readings from the Ground Sensors
            left_gr = self.left_ground.getValue()
            center_gr = self.center_ground.getValue()
            right_gr = self.right_ground.getValue()
            
            # Constraining the Values
            min_gs = 0
            max_gs = 1000
            if left_gr > max_gs: left_gr = max_gs
            if center_gr > max_gs: center_gr = max_gs
            if right_gr > max_gs: right_gr = max_gs
            if left_gr < min_gs: left_gr = min_gs
            if center_gr < min_gs: center_gr = min_gs
            if right_gr < min_gs: right_gr = min_gs
            
            # Calibrating and Storing the Values
            self.inputs.append((left_gr - min_gs) / (max_gs - min_gs))
            self.inputs.append((center_gr - min_gs) / (max_gs - min_gs))
            self.inputs.append((right_gr - min_gs) / (max_gs - min_gs))
            
            # Acquiring the Readings from the Light Sensors
            for i in range(8):
                sensor_read = self.light_sensors[i].getValue()
                
                # Constraints
                if sensor_read > max_ls: sensor_read = max_ls
                if sensor_read < min_ls: sensor_read = min_ls
                
                # Calibrating and Storing the Values
                self.inputs.append((sensor_read - min_ls) / (max_ls - min_ls))
                
            # Acquiring the Readings from the Distance Sensors
            for i in range(8):
                sensor_read = self.distance_sensors[i].getValue()
                
                # Constraints
                if sensor_read > max_ls: sensor_read = max_ls
                if sensor_read < min_ls: sensor_read = min_ls
                
                # Calibrating and Storing the Values
                self.inputs.append((sensor_read - min_ls) / (max_ls - min_ls))
                
            # Computing the Next Step utilising all the information collected from the sensors
            self.sense_compute_actuate()
            

# Driver Code
if __name__ == "__main__":
    robot = Robot()
    controller = Controller(robot)
    controller.run_robot()
            
        
    















