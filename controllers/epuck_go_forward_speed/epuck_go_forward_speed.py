"""epuck_go_forward_speed controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = 64

# Setting the Max Speed of the Robot
MAX_SPEED = 6.28

# Creating instances of the Motors for the Robot
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Setting the position to be infinite for the robot to move 
# independent of its position
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

# Setting the Velocity of the Robot
leftMotor.setVelocity(0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    pass

# Enter here exit cleanup code.
