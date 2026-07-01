# Methodology Draft

## Proposed Simulation Approach

The study will use a 2D simulation of a small UAV swarm (3-5 drones) operating in a bounded area with at least one obstacle and one target/goal point. Each UAV follows simple goal-seeking behavior combined with collision avoidance logic (reactive, rule-based, not learning-based for the prototype stage).

Perception is simulated, not physically modeled. Each UAV has a simulated sensor that reports detections of obstacles/other drones. Perception errors are injected into this reported data by manipulating what the UAV "believes" it sees, separately from the ground truth state of the environment. This lets us compare UAV behavior under perfect perception vs. flawed perception using the same underlying scenario.

The simulation will be run scenario by scenario. Each scenario changes one perception condition at a time (controlled variable approach) so its individual effect on swarm behavior can be isolated.

## Perception Errors to be Injected

- False positive: UAV detects an obstacle/drone that does not exist
- False negative: UAV fails to detect an obstacle/drone that does exist
- Sensor noise: reported position/distance is offset from the true value
- Latency: detection is delayed by a fixed or variable number of time steps
- Sensor dropout: detection is unavailable for a period of time
- High-confidence wrong detection: a false detection reported with high confidence, so the UAV treats it as certain
- Fusion-related errors: differences in how multiple UAVs combine/share detections (naive vs trust-weighted)

## Swarm Behavior to be Observed

- Path taken by each UAV toward the goal
- Avoidance maneuvers (triggered correctly, unnecessarily, or not triggered at all)
- Collisions or near-misses between UAVs, and between UAVs and obstacles
- Formation spacing/consistency across the swarm (if formation logic is enabled)
- Time taken to reach the goal
- Whether the mission (reaching the goal without collision) succeeds or fails

## Input Variables to be Changed

- Perception error type (false positive, false negative, noise, latency, dropout, high-confidence wrong detection)
- Perception error rate/probability
- Sensor noise magnitude
- Latency duration (number of time steps delayed)
- Dropout duration (number of time steps missing)
- Fusion method (naive averaging vs trust-weighted)
- Number of UAVs (secondary variable, if time permits)

## Output Metrics to be Measured

- Collision risk / near-miss count
- Unnecessary avoidance count (avoidance triggered with no real obstacle present)
- Missed response count (no avoidance triggered despite real obstacle present)
- Mission success (goal reached without collision: yes/no)
- Response time (time steps between true detection event and UAV reaction)
- Formation error (deviation from intended spacing/formation, if applicable)

## How Results Will Be Compared

Each scenario will be run against the same baseline scenario (no perception error, identical map, identical start/goal positions). For each scenario:

- Metrics from the scenario run will be compared directly to the baseline run's metrics
- Differences will be reported as absolute change and, where meaningful, percentage change
- Behavior differences will also be described qualitatively (e.g. "swarm took a longer path", "one UAV stalled near the obstacle")
- Results will be logged to CSV per run so scenarios can later be compared side by side in a summary table
- No statistical significance testing is planned at the prototype stage since each scenario is run as a single deterministic or lightly randomized trial; repeated trials with averaging can be added in a later phase if needed
