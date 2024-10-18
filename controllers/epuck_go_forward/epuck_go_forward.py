"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# Timestep of the World
TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# Creating the Motor device instances
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

"""
The setPosition function is used to navigate the robot at maximum speed for
a given number of radians.
"""
leftMotor.setPosition(50.0)
rightMotor.setPosition(50.0)

# Making the robot to loop as long as the timestep is active
while robot.step(TIME_STEP) != -1:
    pass

# Enter here exit cleanup code.
