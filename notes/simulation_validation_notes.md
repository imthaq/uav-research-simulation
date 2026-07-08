# Questions regarding the Simulation Validation:


- UAVs move toward the goal correctly.

A)  Yes  the UAV moves towards its goal correctly.

-  UAVs avoid obstacles when detection is correct.

A) Yes when the field of UAV obstacles detection detects an obtacles it a vector anti-parrallel to the direction of goal vector of the UAV, hence adding a repulsion factor to the movement.
-  False positives create unnecessary avoidance.

A) Yes false positives create unncessary acoidance, which in real life can be depicted as loss of energy to adding extra motion to the movement of UAV.
-  False negatives create missed response or collision risk.

A) Yes it does, however this factor is sometimes negated by averaging the confidence factor of all  the UAVs regarding the obstacle detection.
-  Sensor noise changes perceived obstacle position/distance.

A) Yes it does, however in this 2D scenario it is offen used by decreasing the confidence value of the UAV by a random factor.
-  Latency delays UAV response.

A) Yes , the UAV receives the obstcale detection after certain number of step i.e the number of delays.
-  Sensor dropout causes missing perception data.

A) Yes it does.
-  Mission success is calculated correctly.

A) Yes the missoon success is calculated by confirming that all the UAV reach the goal.
-  Collision risk / near miss logic is correct.

A) Yes, when collosion risk is high the force of repulsion is also high and when it cross the threshold value near_miss_logic is incremented by 1 to determine the count.
-  CSV values match actual simulation behavior

A) Yes it does.
