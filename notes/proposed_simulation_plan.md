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

## 5 Simple Simulation Environment with Perception Errors:

   - possible simulation environment: OpenUAV, ROS/Gazebo, or any other
   - number of UAVs: 1
   - mission scenario: Reach the target while experiencing simulated sensor noise to observe path correction behavior.
   - obstacle/target scenario: None
   - sensing model: LiDAR or GPS
   - perception errors to inject: Gaussian noise in distance/location measurements
   - fusion conditions to test: None
   - swarm-level outputs to measure: NIL
   - expected tables/graphs: Commanded Path vs. Actual Path Trajectory, or Deviation Error Magnitude vs. Time-.

<hr>

## 6 Simple Waypoint Navigation Environment:

   - possible simulation environment: OpenUAV, PX4 SITL, or any other
   - number of UAVs: 1
   - mission scenario: Navigate through a sequence of specific predefined waypoints before reaching the final target location.
   - obstacle/target scenario: 4 waypoints, 0 obstacles
   - sensing model: GPS and Odometry
   - perception errors to inject: None
   - fusion conditions to test: None
   - swarm-level outputs to measure: NIL
   - expected tables/graphs: 3D Trajectory Map (X, Y, Z coordinates) or Time Taken per Waypoint Segment.

<hr>

## 7 Simple Swarm Formation Environment:

   - possible simulation environment: OpenUAV, Webots, or AirSim
   - number of UAVs: 3
   - mission scenario: Drones take off and must establish and maintain a basic "V" formation or line-abreast formation while flying to a single target location.
   - obstacle/target scenario: None
   - sensing model: GPS and V2V (Vehicle-to-Vehicle) communication for position sharing
   - perception errors to inject: None
   - fusion conditions to test: None
   - swarm-level outputs to measure: Inter-agent distance, Formation error
   - expected tables/graphs: Inter-UAV Distance vs. Time (to ensure they maintain formation bounds without colliding).
