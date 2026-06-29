# Different Simulation Environments.

<hr>

## 1   Simple  Simulation Environment: 

- possible simulation environment: OpenUAV or any other 
- number of UAVs: 1 
- mission scenario: Reach the target
- obstacle/target scenario: None
- sensing model: LiDAR
- perception errors to inject: None
- fusion conditions to test: None
- swarm-level outputs to measure: NIL
- expected tables/graphs: Distance vs Time or Energy Consumed vs Time Taken to reach the target.

<hr>

## 2   Simple  Simulation Environment with Obstacles: 

- possible simulation environment:  OpenUAV or any other
- number of UAVs: 1 
- mission scenario: Reach the target while also avoiding the obstacles.s
- obstacle/target scenario: 3  
- sensing model: LiDAR or some Ultrasonic Sensor 
- perception errors to inject: None
- fusion conditions to test: None
- swarm-level outputs to measure: NIL
- expected tables/graphs: Distance vs Time or Energy Consumed vs Time Taken to reach the target.


<hr>

## 3 Medium Simulation Environment with Obstacles: 

- possible simulation environment:  OpenUAV or any other
- number of UAVs: 2 
- mission scenario: The main objective of the two drones would be to reach the target while avoiding the obstacles in the path and also preventing collision with each other.
- obstacle/target scenario: 8  
- sensing model: LiDAR, radar sensor or Ultrasonic sensor 
- perception errors to inject: None
- fusion conditions to test: None
- swarm-level outputs to measure: NIL
- expected tables/graphs: Both Drones Time Taken to reach the target and compare both graph against each other.

<hr>

## 4 ML based Simulation Environment: 

- possible simulation environment: AirSim
- number of UAVs: 2 
- mission scenario: The main objective of the two drones would be to reach the target while avoiding the obstacles in the path and also preventing collision with each other, but we will try to integrate some AI model to make the decision making for our drone a lot easier obstacle/target scenario: 8  
- sensing model: LiDAR, radar sensor or Ultrasonic Sensor 
- perception errors to inject: None
- fusion conditions to test: None
- swarm-level outputs to measure: NIL
- expected tables/graphs: Loss Error graph of the ML model, how  much our ML model was able to correctly predict the path taken.
<hr>

