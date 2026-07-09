‎# Radar Model Explanation
‎
‎### - Why radar sensing is added ?
‎- Radar sensing is added to the simulation to make it adaptable in real life scenarios. When we talk about real life we don't know the target actual coordinates unlike the simulation where we use ground truth. Therefore to determine the position of the target we often use radar based sensing models to help our UAV reach it's goal.
‎
‎### - What Range means ?
‎- Range means a particular distance from one point to another.
‎
‎### - What bearing means ?
‎- It means the angle we obtain between our UAV and the target position.
‎
‎### - What radial  velocity means ?
‎- It means rate of change of velocity of our obstacle wrt to the observer. We often use Doppler Effect to determine vx , vy.
‎
‎### - What P_D means ?
‎-  It is refered to radar detection probabiltity, it is high when target is detected else its value decreases with time.
‎
‎### - What P_FA means ?
‎- It refers to probability of radar false alarm. It is often high when erroneous target detection decision caused by background noise or interfering signals exceeding the radar's detection threshold. It makes UAV into thinking that target is present when it is not.
‎
‎### - What clutter means ?
‎- In UAV detection, clutter refers to unwanted, background radar or sensor echoes from environmental elements 
‎
‎### - How missed detection is generated ?
‎- Missed detection is generated when false negative is high, it is usually generated randomly in simulation.
‎
‎
‎### - How false alarm is generated ?
‎- False alarm is generated when the
‎
‎
‎### - How range bearing noise changes the position ?
‎- Range bearing noise changes the position by adding error value to the true detection of the radar causing UAV to divert from its path.
‎
‎
‎
‎### How radar fused outputs effects decision making ?
‎- By combining the detections of the all the UAVs and by using naive-fusion or trust-weighted-fusion technique we can determine the position of obstacle more accurately rather than relying on the result from some single UAV which could be false as well.
