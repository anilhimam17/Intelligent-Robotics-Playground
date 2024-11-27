from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 5  # m/s
   
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

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
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        # Light detection for robot
        self.light_detected = False
     
    def clip_value(self, value, min_max):
        if value > min_max:
            return min_max
        elif value < -min_max:
            return -min_max
        return value
        
    def read_sensors(self):
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
            
        
    def detect_obstacle_ahead_left(self):
        """Return true when detecting an obstacle to the front left (p7 sensor)
        and the back right (p3 sensor). Otherwise return false."""
        OBSTACLE_THRESHOLD1 = 90
        OBSTACLE_THRESHOLD2 = 40  
        front_left = self.distance_sensors[7].getValue()
        back_right = self.distance_sensors[3].getValue() 
        
        return front_left > OBSTACLE_THRESHOLD1 and back_right > OBSTACLE_THRESHOLD2
    
    def detect_obstacle_ahead_right(self):
        """Return true when detecting an obstacle to the front right (p0 sensor)
        and the back left (p4 sensor). Otherwise return false."""        
        OBSTACLE_THRESHOLD1 = 90
        OBSTACLE_THRESHOLD2 = 40  
        front_right = self.distance_sensors[0].getValue()
        back_left = self.distance_sensors[4].getValue()
        
        return front_right > OBSTACLE_THRESHOLD1 and back_left > OBSTACLE_THRESHOLD2

    
    def detect_obstacle_ahead_all(self):
        """Check for two possible configurations of sensor values, both indicating that the robot has
        reached the end of the maze, return true if this is the case, otherwise return false"""
        OBSTACLE_THRESHOLD1 = 100
        OBSTACLE_THRESHOLD2 = 1000
        p0 = self.distance_sensors[0].getValue()  
        p1 = self.distance_sensors[1].getValue()              
        p2 = self.distance_sensors[2].getValue()
        p3 = self.distance_sensors[3].getValue()
        p5 = self.distance_sensors[5].getValue()
        p6 = self.distance_sensors[6].getValue()
        p7 = self.distance_sensors[7].getValue()
        
        return (p2 > OBSTACLE_THRESHOLD2 and p1 > OBSTACLE_THRESHOLD1 and p3 > OBSTACLE_THRESHOLD1) or (p6 > OBSTACLE_THRESHOLD2 and p5 > OBSTACLE_THRESHOLD2 and p2 > 90 and p0 > 90 and p7 > 100)
        
    def detect_light(self):
        """Return true if the robot's light sensor detects light, otherwise
        return false."""
        return self.light_sensors[2].getValue() <= 10
    
    def move_forward(self):
      """Move the robot forwards at maximum speed"""
      self.left_motor.setVelocity(self.max_speed)
      self.right_motor.setVelocity(self.max_speed)

    def turn_right(self):
        """Turn the robot in a clockwise direction"""
        self.left_motor.setVelocity(self.max_speed)
        self.right_motor.setVelocity(-self.max_speed)
    
    def turn_left(self):
        """Turn the robot in an anti-clockwise direction"""
        self.left_motor.setVelocity(-self.max_speed)
        self.right_motor.setVelocity(self.max_speed)
     
    def drift_right(self):
        """Move the robot forward, while a moderate dtift to the right
        (in the clockwise direction)"""
        self.left_motor.setVelocity(self.max_speed)
        self.right_motor.setVelocity(self.max_speed*0.85)
        
    def drift_left(self):
        """Move the robot forward, with a small drift to the left
        (in the anti-clockwise direction)."""
        self.left_motor.setVelocity(self.max_speed * 0.985)
        self.right_motor.setVelocity(self.max_speed)
        
    def robot_stop(self):
        """Make the robot stop moving."""
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)        

    def sense_compute_and_actuate(self):
        """Makes the robot go through the necessary motions of navigating the maze,
        if the light is on it stops at the end of the right path, otherwise it stops
        at the end of the left path."""
                  
        if self.detect_obstacle_ahead_all():
        # If the robot has reached a dead-end, it must stop
            self.robot_stop()
        
        elif self.detect_obstacle_ahead_right():
        # If obstacles are detected to the front right and back left of the robot, it should turn to the left
            self.turn_left()
        
        elif self.detect_obstacle_ahead_left():
               # If obstacles are detected to the front right and back left of the robot, it should turn to the left
            self.turn_right()
            
        elif self.detect_light():
            # When light is detected, the robot must make a small turn to the right, this orients it correctly for turning at the junction
            self.drift_right()
          
        else:
            # In the abscense of obstacle or light inputs, the robot must move forwards with a small drift to the left
            self.drift_left()
                         
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