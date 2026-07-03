# Few Research Paper Gaps
 
 
 The Following file contains the research gap from the different research papers we found.

## *Title* :  UAV Collision Avoidance in Unknown Scenarios with Causal Representation Disentanglement

*Research Gap* : One thing we could focus on through this research is that UAV tends to take the longest path possible to reach its destination, making it less energy efficient as well. We can focus on this aspect of the research making it more effective and better in this regard by improving the path planning of the UAV.


## *Swarm Level Outcomes* :

- collision risk : Evaluated via a negative collision reward (r-collision = -10) during training 

- formation error: NIL

- mission success:  Quantified using the "Fleet Success Rate" and  “Individual Success Rate”

- response time: Not Given

- unnecessary avoidance: Addressed and minimized by using an "Extra Distance" metric

- missed response: Prevented by implementing a safety-distance boundary threshold (d-safe= 5 meters)

- swarm stability: NIL


## *Title* : An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images

*Research Gap* : The research gap it has is that it fails at identifying the object correctly which is located at a very long distance from  the radar. We could  try to resolve this problem and also decrease the energy consumption by incorporating green code into the software.

## *Title* : Intelligent Multimodal Multi-Sensor Fusion-Based UAV Identification, Localization, and Countermeasures for Safeguarding Low-Altitude Economy

*Research Gap*: While this research only focuses on the area of fusion of radar , thermal and Optronic sensors to create an Intelligent UAV drone, we can expand our research by focusing on fusion of Image into our UAV of course it comes with cost , lower energy efficiency, but we can compromise one or two sensor for the image based recognition, and compare the results of scenarios by training and testing out model on different dataset and determining the best possible combination to achieve the effective results.


## *Title* : 3D UAV Trajectory Planning for IoT Data Collection via Matrix-Based Evolutionary Computation 

*Research Gap*: NIL I believe it has addressed most of the challenges and solved them in  an effective way.  Moreover this paper is more focused on the concept of optimizing different mathematical concepts for optimized tragectory planning of the UAVs.



## *Title* : Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey 

*Research Gap*: As a survey rather than an experimental paper, it does not itself test swarm-density, multi-UAV occlusion, or false-alarm-rate scenarios - confirming this remains an open gap for our work to address.


## *Title* : Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions

*Research Gap*: Automotive, single-target, and weather-only scope leaves multi-UAV swarm density, inter-UAV occlusion, and clutter-driven false alarms unaddressed.


## *Title* : An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images

*Research Gap*: As the first uncertainty-modeling approach in this domain, it is confined to a single sensor and single target, leaving multi-UAV, multi-sensor swarm uncertainty handling unexplored.



## *Title* : When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in Unmanned Aerial Vehicle Safety

*Research Gap*: Single-UAV, single-sensor (camera-based obstacle avoidance), correlational rather than algorithmic, and with no swarm-scale or multi-sensor fusion - leaving exactly the multi-UAV, multi-sensor dependability gap our work targets.


## Md file Summary: 
Most the gap we found from the above mentioned research papers are mostly related to UAV detection through radar, UAV swarm formation, sensors error, testing of UAVs through software simulation while leaving out different environmental factors which may effect the performance of the UAVs in real world scenario. Most of these research papers do not cover false positive, false negatives errors, sensor dropout, latency and fusion reliability. Moreover most of the perception errors are widely ignored, additionally also ignoring the swarm formation relationship directly with different perception level which induces the factor of uncertainty in the UAVs affecting the swarm flight as well while also increasing the risk of collision. Our main gap to study is regarding how perception level uncertainty affects swarm level safety and performance.

