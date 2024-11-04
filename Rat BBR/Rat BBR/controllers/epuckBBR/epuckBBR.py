from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
   
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0

        # Enable Distance Sensors
        self.distance_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.distance_sensors.append(self.robot.getDevice(sensor_name))
            self.distance_sensors[i].enable(self.time_step)
    
        # Enable Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
               
        # Persistent Light Detection
        self.light_detected = False
        
        # Wall Detection Timing
        self.wall_detected_time = None
        self.wall_detection_duration = 0
        self.wall_detection_threshold = 5 #5 seconds for stopping

     
    def clip_value(self, value, min_max):
        if value > min_max:
            return min_max
        elif value < -min_max:
            return -min_max
        return value
        
    def read_sensors(self):
        # Read and normalize values from ground sensors
        self.inputs = []
        left = self.clip_value(self.left_ir.getValue(), 1000)
        center = self.clip_value(self.center_ir.getValue(), 1000)
        right = self.clip_value(self.right_ir.getValue(), 1000)
        
        self.inputs.append((left / 1000))
        self.inputs.append((center / 1000))
        self.inputs.append((right / 1000))

        # Read and normalize values from light sensors
        for i in range(8):
            temp = self.light_sensors[i].getValue()
            temp = self.clip_value(temp, 4300)
            self.inputs.append((temp / 4300))

        # Read and normalize values from distance sensors
        for i in range(8):
            temp = self.distance_sensors[i].getValue()
            temp = self.clip_value(temp, 4300)
            self.inputs.append((temp / 4300))
            
    def detect_obstacle_ahead(self):
        # Check the front sensors (ps0 and ps7) for an obstacle ahead
        OBSTACLE_THRESHOLD = 80  # Adjust this based on environment tuning
        front_left = self.distance_sensors[0].getValue()
        front_right = self.distance_sensors[7].getValue()
        
        # If either front sensor detects an obstacle closer than the threshold, return True
        return front_left > OBSTACLE_THRESHOLD or front_right > OBSTACLE_THRESHOLD
    
    def update_wall_detection_timer(self):
        # If obstacle detected, start or update timer
        if self.detect_obstacle_ahead():
            if self.wall_detected_time is None:
                self.wall_detected_time = self.robot.getTime()
            else:
                # Calculate detection duration
                self.wall_detection_duration = self.robot.getTime() - self.wall_detected_time
                if self.wall_detection_duration > self.wall_detection_threshold:
                    return True  # Wall detected for more than 5 seconds
        else:
            # Reset wall detection timer
            self.wall_detected_time = None
            self.wall_detection_duration = 0
        return False

    def move_forward(self):
        # Move both motors forward at max speed
        self.left_motor.setVelocity(self.max_speed)
        self.right_motor.setVelocity(self.max_speed)

    def turn_right(self):
        # Set the motors to turn right
        self.left_motor.setVelocity(self.max_speed * 0.5)
        self.right_motor.setVelocity(-self.max_speed * 0.5)
        
    
    def turn_left(self):
        # Set the motors to turn right
        self.left_motor.setVelocity(-self.max_speed * 0.5)
        self.right_motor.setVelocity(self.max_speed * 0.5)
       
    def robot_stop(self):
        # Set the motors to turn right
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
                

    def sense_compute_and_actuate(self):
            
        # Check if light is detected from any sensor
        light_detected_now = self.inputs[5] == 0 or self.inputs[8] == 0
        
        if light_detected_now:
            self.light_detected = True  # Set persistent light detection flag
            
        if self.update_wall_detection_timer():
            self.robot_stop()
            return
                
        if self.light_detected == True:
            if self.detect_obstacle_ahead():
                self.turn_timer = 15
                # Detected an obstacle ahead, switch to turning left
                self.turn_right()
                self.turn_timer -= 1  # Set turn timer for turning left
            else:
                self.move_forward()              
        else:
            if self.detect_obstacle_ahead():
                self.turn_timer = 15
                # Detected an obstacle ahead, switch to turning left
                self.turn_left()
                self.turn_timer -= 1  # Set turn timer for turning left
            else:
                self.move_forward()
                    
                 
    def run(self):
        # Main Loop
        while self.robot.step(self.time_step) != -1:
            # Read sensors
            self.read_sensors()
            # Compute and actuate based on sensor readings
            self.sense_compute_and_actuate()

# Instantiate and run the robot controller
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run()