### Ground Truth Leakage Audit 

## - File
- simple_swarm_sim.py
## - Function
- This is base class for creating a 2D simulation. It creates the simulation required to simulate the whole world. All the drone entities , their movement , direction , radar detection , obstacle avoidence , goal reached or not and so on is done in this part.
## - Ground-truth access
- The Position of Goal is Known in Advance.
## - Allowed purpose
- To Move the drone toward the target position.
## - Runtime-decision access
- Yes, all the decision are made during runtime, however most of the values regarding the percentage rate at which we want to induce the errros into our UAV is imported from the config file.
## - PASS/FAIL
- Most of the test scenarios are passed, but some even end in failure as well.
## - Required correction
- The position of the goal must have to be decided during the runtime from the detections precieved from our sensors.

## - File
- radar_like_model.py
## - Function
- The function of the file is to induce a radar_like_model into our UAV. Where in Simple Swarm Sim class the range of UAV is static, radar_like_model allow us to control its range by setting the max, min values , maximum number of targets it can detect inside it's range MAX here is 4. Moreover different modes have been included in here such as normal mode , high resolution mode and degraded. Of course we can't use real radar in 2D simulation therefore we control the reliability of radar sensing information, by changing the values of the error we induce into our radar formation such as latency steps , max , min range , PD , PFA and so on.
## - Ground-truth access
- No, we avoid this by creating and recording  the values of the different instances true detection vs the precieved detections, i.e we overwrite the value of what is precieved by the UAV by inducing the error into the measurement. We create different instance of we is precieved before applying the error and we overwrite current values this inducing error. Also through this way we can include radial velocity and bearing angle.
## - Allowed purpose
- To achieve real world radar like behaviour.
## - Runtime-decision access
- Yes
## - PASS/FAIL
- Neutral some pass , some fail.
## - Required correction
- None

## - File
- vision_like_model.py
## - Function
- The main purpose is to induce the vision like effect into our model. However, how can we achieve this behaviour i understand about the radar having fixed or varying range to detect the objects. How do i achieve the same results using vision like model, of course i can't using some camera in an illusionary 2D simulated world. What we do to achieve this effect in 2D is by combining radar and vision. We know camera has a limited FOV, and in our our world its FOV is like a cone where the tip starts from our camera. The UAV can only sense and detect the objects within this FOV and induce the errors such as different env factors e.g fog , storm , lightning  we just remove the objects precieved by the drone to actully accumulate the behaviour of the Vision_Like_Model.py. Now instead of making things harder we make it easier to actually get the detections inside the FOV of vision. First we get the true detection of the obstacles. If the obstacle lies inside the FOV of camera append it to the actual true detections else remove it. Moreover we also add a dela(0.3 steps)y of the information recieved from the vision till it reaches the actual decision logic generator, since in real life it also takes some time in hz/s for camera to capture each frame and pass it to the UAV.
## - Ground-truth access
- None, but when receiving the pos if obstacle it is already know not computed at runtime, by using sensing waves to computed. Because this increase the computational cost and runtime of the code just to determine the position of the obstacles. 
## - Allowed purpose
- To achieve vision like behaviour.
## - Runtime-decision access
- Yes 
## - PASS/FAIL
- Neutral, some test cases pass, some fail.
## - Required correction
- To be able to detect the obstacle at runtime rather than knowing their actual positions, however implementing this while also considering the constraints is not possible right now. Because when we think about achieving the behaviour of the radar sensor we have to send out a wave everytime , everystep in the direction of the radar making the program more slower, if we try to achieve this effect using loops.


## - File
- lidar_like_model.py
## - Function
- Lidar uses the time of flight technology to get the detections of the obstacles. Unlike the vision model it is not good at telling what the object it detected actually is, but it is perfect at telling the percise location of the obstacles within the limited range, whereas the vision model lacks in this reagrd. The vision model has increased range wehere as the lidar has limited range, however the FOV of Lidar is greater than than the vision model(120* > 80*) moreover this FOV can be increased as well inlike vision model. Moreover where vision model gets effected by different environmental factors such as fog and lightning, the only error inducded in pd_eff in the lidar such that in real life clusttering and jammers can effects the  lidar detections measurements.
## Note: Since vision and lidar models mostly are alike therefore most of the information is same.
## - Ground-truth access
- None, but when receiving the pos if obstacle it is already know not computed at runtime, by using sensing waves to computed. Because this increase the computational cost and runtime of the code just to determine the position of the obstacles. 
## - Allowed purpose
- To achieve LiDar like behaviour.
## - Runtime-decision access
- Yes 
## - PASS/FAIL
- Neutral, some test cases pass, some fail.
## - Required correction
- To be able to detect the obstacle at runtime rather than knowing their actual positions, however implementing this while also considering the constraints is not possible right now. Because when we think about achieving the behaviour of the radar sensor we have to send out a wave everytime , everystep in the direction of the radar making the program more slower, if we try to achieve this effect using loops.

## - File
- radar_track_model.py
## - Function
- There are two main used here Kalman filter and the Nearest Neighbour Greedy Algorithm. The main purpose of the Kalman filter is to figure out the next step pos(x , y) based on the current pos and velocity. Wheras we know multipe tracks are maintained for the multiple targets, so we Nearest Neighbour Greedist Approach to chose the detections are closer to the contrained detections set by us the GATE_CHI value. 
## - Ground-truth access
- None 
## - Allowed purpose
- To build radar track using Kalman filter and nearest neighbour appraoch while also choosing the optimal path based on the prediction made.
## - Runtime-decision access
- Yes
## - PASS/FAIL
- Pass
## - Required correction
- NIL


## - File
- fusion_model.py
## - Function
## - Ground-truth access
## - Allowed purpose
## - Runtime-decision access
## - PASS/FAIL
## - Required correction
