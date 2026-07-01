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
