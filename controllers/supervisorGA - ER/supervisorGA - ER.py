from controller import Supervisor
from controller import Keyboard
from controller import Display
import config2 as config

import numpy as np
import math
import ga,os,sys,struct

class SupervisorGA:
    def __init__(self):
        # Simulation Parameters
        # Please, do not change these parameters
        self.time_step = 32 # ms
        self.time_experiment = 60 # s
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Check if the robot node exists in the current world file
        self.robot_node = self.supervisor.getFromDef("Controller")
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the robots translation and rotation current parameters    
        self.trans_field = self.robot_node.getField("translation")  
        self.rot_field = self.robot_node.getField("rotation")
        
        # Check Receiver and Emitter are enabled
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)
        
        # Initialize the receiver and emitter data to null
        self.receivedData = "" 
        self.receivedWeights = "" 
        self.receivedFitness = "" 
        self.emitterData = ""
        
        ###########
        ### DEFINE here the 3 GA Parameters:
        self.num_generations = config.NUM_GENERATIONS
        self.num_population = config.NUM_POPULATION
        self.num_elite = config.NUM_ELITE
        self.fitness_paper_weight = config.PAPER_WEIGHT
        self.fitness_distance_weight = 1 - self.fitness_paper_weight
        
        # size of the genotype variable
        self.num_weights = 0
        
        # Creating the initial population
        self.population = []
        
        # All Genotypes
        self.genotypes = []
        
        # Display: screen to plot the fitness values of the best individual and the average of the entire population
        self.display = self.supervisor.getDevice("display")
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.prev_best_fitness = 0.0;
        self.prev_average_fitness = 0.0;
        self.display.drawText("Fitness (Best - Red)", 0,0)
        self.display.drawText("Fitness (Average - Green)", 0,10)
        self.max_distance = 0.63
        
        # Black mark
        self.mark_node = self.supervisor.getFromDef("Mark")
        if self.mark_node is None:
            sys.stderr.write("No DEF Mark node found in the current world file\n")
            sys.exit(1)
        #self.mark_trans_field = self.mark_node.getField("translation")
        self.mark_loc_field = self.mark_node.getField("location")     

    def createRandomPopulation(self):
        # Wait until the supervisor receives the size of the genotypes (number of weights)
        if(self.num_weights > 0):
            #  Size of the population and genotype
            pop_size = (self.num_population,self.num_weights)
            # Create the initial population with random weights
            self.population = np.random.uniform(low=-1.0, high=1.0, size=pop_size)

    def handle_receiver(self):
        while(self.receiver.getQueueLength() > 0):
            #Webots 2022: 
            #self.receivedData = self.receiver.getData().decode("utf-8")
            #Webots 2023: 
            self.receivedData = self.receiver.getString()
            typeMessage = self.receivedData[0:7]
            # Check Message 
            if(typeMessage == "weights"):
                self.receivedWeights = self.receivedData[9:len(self.receivedData)] 
                self.num_weights = int(self.receivedWeights)
            elif(typeMessage == "fitness"):  
                self.receivedFitness = float(self.receivedData[9:len(self.receivedData)])
            self.receiver.nextPacket()
        
    def handle_emitter(self):
        if(self.num_weights > 0):
            # Send genotype of an individual
            string_message = str(self.emitterData)
            string_message = string_message.encode("utf-8")
            #print("Supervisor send:", string_message)
            self.emitter.send(string_message)     
        
    def run_seconds(self,seconds):
        #print("Run Simulation")
        stop = int((seconds*1000)/self.time_step)
        iterations = 0
        while self.supervisor.step(self.time_step) != -1:
            self.handle_emitter()
            self.handle_receiver()
            if(stop == iterations):
                break    
            iterations = iterations + 1
                
    def evaluate_genotype(self,genotype,generation):
        # Here you can choose how many times the current individual will interact with both environments
        # At each interaction loop, one trial on each environment will be performed
        numberofInteractionLoops = 3
        currentInteraction = 0
        fitnessPerTrial = []
        while currentInteraction < numberofInteractionLoops:
            #######################################
            # TRIAL: TURN RIGHT
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the black mark position and physics
            #Webots 2022:  
            # INITIAL_TRANS = [0.01, -0.03425, 0.193]
            #Webots 2023:
            #Used the original mark code for the light, just swapping the position to be more suitable for the spotlight
            INITIAL_TRANS = [0.1, 0, 0.2]
            self.mark_loc_field.setSFVec3f(INITIAL_TRANS)
            self.mark_node.resetPhysics()
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness_paper = self.receivedFitness

            
            # Check for Reward and add it to the fitness value here
            #RIGHT
            #right - 0.378, -0.00361, -0.156
            target_position_right = [0.378, -0.00361, -0.156]
            current_position = self.trans_field.getSFVec3f()
            
            distance_right = np.sqrt(
                (current_position[0] - target_position_right[0]) ** 2 +
                (current_position[1] - target_position_right[1]) ** 2 +
                (current_position[2] - target_position_right[2]) ** 2
                )

            fitness_distance = max(0.1, 1 - (distance_right / self.max_distance))
            
            
            fitness = self.fitness_paper_weight * fitness_paper + self.fitness_distance_weight * fitness_distance
            
            print("Fitness: {}".format(fitness))     
                        
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            #######################################
            # TRIAL: TURN LEFT
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the black mark position and physics
            INITIAL_TRANS = [0.2, -10, 0.2]
            self.mark_loc_field.setSFVec3f(INITIAL_TRANS)
            self.mark_node.resetPhysics()
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness_paper = self.receivedFitness
            
            # Check for Reward and add it to the fitness value here
            #left - -0.373,-0.00115, -0.152
            target_position_left = [-0.373, -0.00115, -0.152]
            current_position = self.trans_field.getSFVec3f()

            distance_left = np.sqrt(
                (current_position[0] - target_position_left[0]) ** 2 +
                (current_position[1] - target_position_left[1]) ** 2 +
                (current_position[2] - target_position_left[2]) ** 2
                )

            fitness_distance = max(0.1, 1 - (distance_left / self.max_distance))
             
            fitness = self.fitness_paper_weight * fitness_paper + self.fitness_distance_weight * fitness_distance
         
            print("Fitness: {}".format(fitness))
            
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            # End 
            currentInteraction += 1
            
        print(fitnessPerTrial)    
        
        fitness = np.mean(fitnessPerTrial)
        current = (generation,genotype,fitness)
        self.genotypes.append(current)  
        
        return fitness

    def run_demo(self):
        # Read File
        genotype = np.load("Best.npy")
        
        # Turn Left
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
        
        # Reset the black mark position and physics
        INITIAL_TRANS = [0.2, -10, 0.2]
        self.mark_loc_field.setSFVec3f(INITIAL_TRANS)
        self.mark_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment) 
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without reward or penalty: {}".format(fitness))
        
        # Turn Right
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
        
        # Reset the black mark position and physics
        #INITIAL_TRANS = [0.01, -0.03425, 0.193]
        INITIAL_TRANS = [0.1, 0, 0.2]
        self.mark_loc_field.setSFVec3f(INITIAL_TRANS)
        self.mark_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment)  
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without reward or penalty: {}".format(fitness))    
    
    def run_optimization(self):
        # Wait until the number of weights is updated
        while(self.num_weights == 0):
            self.handle_receiver()
            self.createRandomPopulation()
        
        print(">>>Starting Evolution using GA optimization ...\n")
        
        # For each Generation
        for generation in range(self.num_generations):
            print("Generation: {}".format(generation))
            current_population = []   
            # Select each Genotype or Individual
            for population in range(self.num_population):
                genotype = self.population[population]
                # Evaluate
                fitness = self.evaluate_genotype(genotype,generation)
                #print(fitness)
                # Save its fitness value
                current_population.append((genotype,float(fitness)))
                #print(current_population)
                
            # After checking the fitness value of all indivuals
            # Save genotype of the best individual
            best = ga.getBestGenotype(current_population);
            average = ga.getAverageGenotype(current_population);
            np.save("Best.npy",best[0])
            self.plot_fitness(generation, best[1], average);
            
            # Generate the new population using genetic operators
            if (generation < self.num_generations - 1):
                self.population = ga.population_reproduce(current_population,self.num_elite);
        
        #print("All Genotypes: {}".format(self.genotypes))
        print("GA optimization terminated.\n")   
    
    
    def draw_scaled_line(self, generation, y1, y2): 
        # the scale of the fitness plot
        XSCALE = int(self.width/self.num_generations);
        YSCALE = 100;
        self.display.drawLine((generation-1)*XSCALE, self.height-int(y1*YSCALE), generation*XSCALE, self.height-int(y2*YSCALE));
    
    def plot_fitness(self, generation, best_fitness, average_fitness):
        if (generation > 0):
            self.display.setColor(0xff0000);  # red
            self.draw_scaled_line(generation, self.prev_best_fitness, best_fitness);
    
            self.display.setColor(0x00ff00);  # green
            self.draw_scaled_line(generation, self.prev_average_fitness, average_fitness);
    
        self.prev_best_fitness = best_fitness;
        self.prev_average_fitness = average_fitness;
  
    
if __name__ == "__main__":
    # Call Supervisor function to initiate the supervisor module   
    gaModel = SupervisorGA()
    
    # Function used to run the best individual or the GA
    keyboard = Keyboard()
    keyboard.enable(50)
    
    # Interface
    print("***************************************************************************************************")
    print("To start the simulation please click anywhere in the SIMULATION WINDOW(3D Window) and press either:")
    print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
    print("***************************************************************************************************")
    
    while gaModel.supervisor.step(gaModel.time_step) != -1:
        resp = keyboard.getKey()
        if(resp == 83 or resp == 65619):
            gaModel.run_optimization()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        elif(resp == 82 or resp == 65619):
            gaModel.run_demo()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        
