# Intelligent Robotics Playground
 A compilation of different tutorials and courseworks from the Intelligent Robotics course. Each directory is a simulation of robots in a space.

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
