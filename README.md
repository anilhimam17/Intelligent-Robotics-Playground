# Execution
To run the ER program using webots access the world file present in worlds/e-puck_Robotics_TMaze_Webots2023.wbt, to alter the hyperparameters use the files config1 and config2 in controllers/epuck_python - ER/config1.py and controllers/supervisorGA - ER/config2.py respectively.

To run the BBR program using webots access the world file present in Rat BBR/Rat BBR/worlds/e-puck_Robotics_TMaze_BBR.wbt.  

## ER Training Format
- Ensure the controller is `epuck_python - ER.py`
- Hyperparameters to keep track off:
    - epuck_python - ER.py:
        - self.number_hidden_layer
    - supervisorGA - ER.py:
        - self.num_generations
        - self.num_population
        - self.num_elite
        - weight_paper and weight_reward (Ideally would want reward to be higher, both should add up to 1)
    - ga.py:
        - cp
        - mp
- On completing training please store the Best.npy file (/controllers/supervisorGA - ER.py) with the weights and log the hyperparameters mentioned above within the training log folder.


