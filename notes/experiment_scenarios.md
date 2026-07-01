# Experiment Scenarios

## 1. Baseline (No Perception Error)

What will be changed: Nothing. All UAV sensors report ground truth exactly, with no delay, no noise, no dropout.

Why it matters: Establishes the reference behavior and reference metric values that all other scenarios are compared against.

Metric measured: All metrics (collision risk, unnecessary avoidance, missed response, mission success, response time, formation error).

Expected effect on swarm behavior: Smooth, direct path to goal, avoidance triggered only when truly needed, no unnecessary maneuvers, fastest response time, mission succeeds.

## 2. False Positive Scenario

What will be changed: One or more UAVs report a detected obstacle/drone that does not actually exist, at a chosen point in the run.

Why it matters: Tests how the swarm handles phantom threats and whether false alarms cause wasted effort or unsafe maneuvers.

Metric measured: Unnecessary avoidance count, mission success, response time.

Expected effect on swarm behavior: UAV(s) swerve or slow down unnecessarily, possible delay in reaching goal, possible disruption of formation.

## 3. False Negative Scenario

What will be changed: A real obstacle/drone is present but one or more UAVs fail to detect it.

Why it matters: Tests the worst-case safety risk - the swarm not reacting to a real hazard.

Metric measured: Collision risk / near-miss count, missed response count, mission success.

Expected effect on swarm behavior: UAV continues on path toward the undetected obstacle, higher chance of collision or near-miss, mission may fail.

## 4. High-Confidence Wrong Detection

What will be changed: A false detection (obstacle/drone that doesn't exist) is reported with high confidence, so the UAV's decision logic treats it as certain rather than questionable.

Why it matters: Tests whether high confidence in incorrect data leads to worse decisions than a low-confidence false positive would, and whether trust-based logic can be fooled.

Metric measured: Unnecessary avoidance count, response time, mission success.

Expected effect on swarm behavior: Strong, possibly extreme avoidance reaction (e.g. full stop or large detour) based on data that is wrong, larger deviation from planned path than a normal false positive.

## 5. Sensor Dropout

What will be changed: Detection capability is turned off for one or more UAVs for a set period of time (no detections reported at all during that window, regardless of ground truth).

Why it matters: Tests swarm resilience when a UAV effectively goes "blind" temporarily, which is a realistic hardware/communication failure mode.

Metric measured: Collision risk / near-miss count, missed response count, mission success.

Expected effect on swarm behavior: UAV continues moving without reacting to any obstacles during the dropout window, higher collision risk if a hazard appears during that time, possible recovery once dropout ends.

## 6. Delayed Detection (Latency)

What will be changed: Detections are still reported correctly but with a fixed delay (e.g. N time steps after the true event).

Why it matters: Tests how much lag the swarm's avoidance logic can tolerate before reaction becomes too late to be effective.

Metric measured: Response time, collision risk / near-miss count, mission success.

Expected effect on swarm behavior: UAV reacts later than in baseline, avoidance maneuver may be more abrupt or insufficient, higher near-miss count as delay increases.

## 7. Noisy Sensor Readings

What will be changed: Detected position/distance values are offset from the true values by random noise (magnitude varied across runs).

Why it matters: Tests robustness of avoidance logic to imprecise, realistic sensor data rather than perfect measurements.

Metric measured: Collision risk / near-miss count, unnecessary avoidance count, formation error.

Expected effect on swarm behavior: Avoidance maneuvers become less precise, UAV may steer around empty space or misjudge clearance, formation spacing becomes less consistent as noise increases.

## 8. Naive Fusion

What will be changed: When multiple UAVs share detection data, all reports are combined with equal weight regardless of source reliability (simple averaging or "any UAV reports it, so it's treated as true").

Why it matters: Establishes a baseline for how a simple, unweighted data-sharing approach performs when some UAVs have degraded perception (noise, dropout, false positives, etc.).

Metric measured: Unnecessary avoidance count, missed response count, mission success.

Expected effect on swarm behavior: Swarm is easily misled by a single faulty UAV's bad data; one noisy or false-positive-prone UAV can trigger unnecessary avoidance across the group, or a false negative can be masked improperly if fusion logic assumes all inputs are equally trustworthy.

## 9. Trust-Weighted Fusion

What will be changed: Shared detection data is combined using per-UAV trust/reliability weighting (e.g. a UAV known to have noisy or degraded sensors contributes less to the fused decision).

Why it matters: Tests whether weighting detections by source reliability reduces the negative impact of a single faulty UAV compared to naive fusion.

Metric measured: Unnecessary avoidance count, missed response count, mission success, collision risk / near-miss count.

Expected effect on swarm behavior: Swarm is more resistant to a single faulty UAV's bad data; fewer unnecessary avoidance events and fewer missed responses compared to naive fusion under the same injected errors.
