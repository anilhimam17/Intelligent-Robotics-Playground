from controller import Supervisor
import sys

class SupervisorLight:
    # Constructor
    def __init__(self):
        self.time_step = 32
        self.time_light = 40
        self.flag_light = 1 # Flag to keep track of the different positions of the light node
        
        # Initialising the Supervisor
        self.supervisor = Supervisor()
        
        # Acquiring the Robot node from the Environment
        self.robot_node = self.supervisor.getFromDef("Controller")
        
        # If the robot node doesnt exist Error
        if self.robot_node is None:
            sys.stderr.write("No DEF SpotLight node found in the current file")
            sys.exit(1)
            
        # Acquiring the data corresponding to required fields from the Robot node
        self.trans_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")
        
        # Acquiring the Light node from the Environment
        self.light_node = self.supervisor.getFromDef("Light")
        
        # If the light node doesnt exist Error
        if self.light_node is None:
            sys.stderr.write("No DEF SpotLight node found in the current file")
            sys.exit(1)
          
        # Acquiring the data corresponding to required fields from the Light node  
        self.location_field = self.light_node.getField("location")
        self.direction_field = self.light_node.getField("direction")
     
    # Changing the location of the nodes wrt seconds
    def run_seconds(self, seconds):
        
        # Calculating the no of max iterations based on the time_step of the robot
        stop = int((seconds * 1000) / self.time_step)
        iterations = 0
        
        # Main Loop and count the number of iterations until the stop value
        while self.supervisor.step(self.time_step) != -1:
        
            # When stop value is met reset.
            if stop == iterations:
                iterations = 0
                
                # Updating the Params for the Robot
                # Translation
                INITIAL_TRANS = [0.35, 0.20, 0]
                self.trans_field.setSFVec3f(INITIAL_TRANS)
                
                # Rotation
                INITIAL_ROTATION = [0, 1, 0, -0.0]
                self.rotation_field.setSFRotation(INITIAL_ROTATION)
                self.robot_node.resetPhysics()
                
                # Updating the Params for the SpotLight
                # Location
                INITIAL_LOCATION = [1.2, 1, 0.2]
                self.location_field.setSFLocation(INITIAL_LOCATION)
                
                # Direction
                INITIAL_DIRECTION = [-0.5, -0.5, -0.5]
                self.direction_field.setSFDirection(INITIAL_DIRECTION)
                
            iterations += 1
            
    def run_demo(self):
        # Reset physics of the robot (position and rotation)
        # Position
        INITIAL_TRANS = [0.35, 0.20, 0]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        
        # Rotation
        INITIAL_ROT = [0, 1, 0, -0.0]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
        
        # Applying supervisor to alter the nodes based on timesteps
        self.run_seconds(self.time_light)
        
if __name__ == "__main__":
    model = SupervisorLight()
    model.run_demo()
























