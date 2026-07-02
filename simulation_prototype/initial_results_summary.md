­# Initial Results Summary — 4 UAVs vs 2 UAVs

Six scenarios (`baseline`, `false_positive`, `false_negative`, `sensor_noise`, `latency`, `sensor_dropout`) run twice: once with the default swarm of **4 UAVs**, once with a smaller swarm of **2 UAVs**.

## 4 UAVs — every scenario fails

| Scenario | Reached goal | Collisions | Mission success |
|---|---|---|---|
| baseline | 1/4 | 0 | ❌ |
| false_positive | 1/4 | 0 | ❌ |
| false_negative | 1/4 | 0 | ❌ |
| sensor_noise | 1/4 | 67 | ❌ |
| latency | 1/4 | 0 | ❌ |
| sensor_dropout | 1/4 | 3 | ❌ |

## 2 UAVs — some scenarios succeed

| Scenario | Reached goal | Collisions | Mission success |
|---|---|---|---|
| baseline | 1/2 | 0 | ❌ |
| false_positive | 1/2 | 0 | ❌ |
| false_negative | 2/2 | 0 | ✅ |
| sensor_noise | 1/2 | 0 | ❌ |
| latency | 1/2 | 0 | ❌ |
| sensor_dropout | 2/2 | 0 | ✅ |

If you notice even in 2 UAVs Drone scenario it passed the testcases of “false_negative” and “sensor_dropout” only while the simple baseline is still failed meaning it still consider the other drones as an obstacle even if it reaches the goal hence avoiding the collosion and never reaching the goal.


## 2 Day Scenario Testing Results: 

What scenarios were tested? 
1. Baseline with no perception error
2. False positive scenario
3. False negative scenario
4. Sensor noise scenario
5. Latency scenario
6. Sensor dropout scenario

 How many runs were performed ?
- 18, 3 for each scenario.

 What metrics were collected ?
- The following mertics were collected and updated to the .csv files
"scenario",
"run_number",
"false_positive_rate",
"false_negative_rate",
"noise_level",
"latency_steps",
"dropout_probability",
"collision_risk_count",
"unnecessary_avoidance_count",
"missed_response_count",
"mission_success",
"avg_response_time_s",
"total_near_misses",
"avg_formation_error",

What changed compared to baseline?

- More obstacles were added, induced false positives and false negative as well as the sensor drop out , latency as well.


Which perception error caused the most problem?

- Sensor Noise caused the most problem.


What needs improvement in the simulation? 

- If we look at the results most the mission_success rate ends up being false meaning no drone ever reaches the result. This is the first thing that need to be improved.

- During sensor dropout the detection misses a lot of information which need to be retained to perform the best possible results. Therefore there should be somekind of memory factor which keep in storage most of the info.
