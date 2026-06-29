# Variable-to-Metric Mapping

| Input Variable | Expected Effect | Output Metric |
|---|---|---|
| False Positive Rate (inc) | Unnecessary avoidance maneuvers | Response time (inc) |
| False Positive Rate (inc) | Wasted resources on false alarms | Mission success (dec) |
| False Negative Rate (inc) | Missed obstacles/threats | Collision risk (inc) |
| False Negative Rate (inc) | Incomplete spatial awareness | Formation error (inc) |
| Latency (inc) | Delayed reaction to environment | Response time (inc) |
| Latency (inc) | Stale information during avoidance | Collision risk (inc) |
| Latency (inc) | Asynchronous swarm coordination | Formation error (inc) |
| Sensor Dropout (inc) | Incomplete information | Formation error (inc) |
| Sensor Dropout (inc) | Lost situational awareness | Mission success (dec) |
| Sensor Dropout (inc) | Gaps in obstacle detection | Collision risk (inc) |
| Wrong Sensor Trust (inc) | Reliance on faulty data | Collision risk (inc) |
| Wrong Sensor Trust (inc) | Conflicting fusion decisions | Swarm stability (dec) |
| Wrong Sensor Trust (inc) | Incorrect avoidance logic | Response time (inc) |
| Detection Accuracy (dec) | Missed targets/obstacles | Collision risk (inc) |
| Detection Accuracy (dec) | Incomplete mission coverage | Mission success (dec) |
| Detection Accuracy (dec) | Degraded swarm awareness | Swarm stability (dec) |
| Sensor Noise (inc) | Uncertain measurements | Response time (inc) |
| Sensor Noise (inc) | Noisy fused state estimate | Formation error (inc) |
| Sensor Noise (inc) | Reduced confidence in decisions | Swarm stability (dec) |
| Occlusion Level (inc) | Partial visibility/blind zones | Collision risk (inc) |
| Occlusion Level (inc) | Loss of neighbor tracking | Formation error (inc) |
| Occlusion Level (inc) | Limited swarm coordination | Swarm stability (dec) |
| Confidence Calibration Error (inc) | Overconfident wrong decisions | Collision risk (inc) |
| Confidence Calibration Error (inc) | Misaligned fusion priorities | Swarm stability (dec) |
| Confidence Calibration Error (inc) | Unnecessary vs. missed reactions | Response time (inc) |
| Faulty Sensor Behavior (inc) | Corrupted input to fusion | Collision risk (inc) |
| Faulty Sensor Behavior (inc) | Unreliable state estimates | Formation error (inc) |
| Faulty Sensor Behavior (inc) | Degraded swarm consensus | Swarm stability (dec) |
