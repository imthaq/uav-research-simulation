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
## Prototype Simulation Structure
The prototype is implemented as two components. The first, simple_swarm_sim.py, is the simulation engine: it contains a Perception class (one instance per UAV, responsible for corrupting that UAV's detections), a Simulation class (owns world state, steps time forward, applies each UAV's controller, counts collisions/near-misses, writes a log row per UAV per step), and a main() entry point that loads simulation_config.json, runs one or all named scenarios, and writes a per-step CSV log. The second, metrics_analysis.py, is the analysis layer: it imports the Simulation class directly, runs each scenario for several seeded trials, and writes a summary CSV with one row per scenario/run.
The simulation is fully config-driven: world size, obstacle placement, swarm size/speed/formation spacing, sensor range, controller gains, and per-scenario perception-error parameters all live in simulation_config.json. Each named scenario only overrides the specific perception-error parameters it's testing; any parameter it doesn't set falls back to a default.
## How UAV Movement is Modeled
Each UAV is modeled as a point mass with no inertia, using a reactive potential-field controller rather than real flight dynamics. Movement is computed from two combined vectors each step:
- A goal vector: the normalized direction from the UAV's current position to its assigned goal slot, scaled by a goal gain. UAVs do not all steer toward one identical shared point; each is assigned its own slot on a small circle around the target so multiple UAVs can settle at "the goal" without indefinitely jostling for the same spot.
- An avoidance vector: for every currently perceived threat (obstacle or other UAV) within sensor range, a radial repulsion term pointing away from the threat plus a tangential term perpendicular to it, so the UAV slides around a threat instead of stalling head-on against it.
These vectors are summed, renormalized, and scaled by the UAV's speed to produce a velocity; position is then updated by velocity times the simulation time step and clamped to the world boundaries. There is no acceleration limit, turn-rate limit, or momentum in the prototype, so velocity can change direction instantly from one step to the next.
## How Perception Errors are Injected
Ground truth detections are computed independently of perception, based purely on which obstacles/UAVs are within sensor range at that instant. This ground-truth set is then passed through each UAV's Perception object, which applies errors in a fixed order each step:
- Dropout is checked first: if a blackout is already in progress, or a new one probabilistically triggers, the UAV receives no detections at all (real or phantom) for a set number of steps, and no further error types are applied that step.
- False negatives: each true detection independently survives with a configured probability; the rest are silently dropped.
- Position noise: surviving detections have Gaussian jitter added to their reported position, with distance re-derived from the jittered position.
- False positives: with a configured probability, one phantom detection is fabricated at a random point within sensor range and added to the perceived set.
- Latency: independent of the steps above, each step's already-corrupted detection set is placed in a per-UAV buffer and only released to the controller a fixed number of steps later; zero latency behaves as if this step did not exist.
Each Perception instance also records which of these mechanisms fired on a given step, which is what allows the per-step log to report an error type (or combination of error types) alongside the UAV's position and action for that step.
## How Metrics are Calculated
- Collisions and near-misses: after all UAVs move, every UAV-obstacle and UAV-UAV pair is checked once per step; a distance at or below the collision threshold counts as a collision, and a distance at or below the near-miss threshold (but not a collision) counts as a near-miss.
- Collision-risk count: a finer-grained measure than near-miss count, tracked per UAV per step rather than per pair — whichever single entity is closest to a given UAV is checked, and if that distance is within the near-miss threshold the step is flagged as a risk event for that UAV.
- Unnecessary avoidance count: incremented on any step where a UAV received one or more detections but none of them were real (i.e. phantom-only), and the avoidance behavior still triggered.
- Missed response count: incremented on any step where a real threat was within the near-miss threshold in ground truth but did not appear in what the UAV perceived that step.
- Response time: for each real threat, the time between the step it first became detectable in ground truth and the step the UAV first perceived it (after any latency delay), averaged across all such events and converted to seconds.
- Formation error: the root-mean-square deviation of every pairwise UAV-to-UAV distance from the desired formation spacing, averaged over the run.
- Mission success: true only if every UAV reached its goal tolerance and zero collisions occurred at any point during the run.
- Aggregation across runs: metrics_analysis.py runs each scenario for multiple seeded trials (the random seed is incremented per run) and writes one row per scenario/run to the summary CSV, so results can be examined per-trial or averaged.
## Current Limitations
- The baseline scenario itself does not reliably succeed across repeated trials, before any perception error is introduced, which weakens clean comparisons against it until controller/formation tuning improves.
- There is no confidence or fusion model: every detection is treated as equally certain, and there is no cross-UAV sharing of detections, so the "high-confidence wrong detection" and fusion-related error types from the original proposal are not yet representable.
- The avoidance force scales inversely with distance without an upper bound, so a detection very close to a UAV, especially a noisy or phantom one, can produce a disproportionately large steering correction.
- UAV movement has no real flight dynamics: no inertia, acceleration limits, or turn-rate limits, so behavior resembles an idealized reactive agent more than a realistic drone.
- The environment is limited to a single static obstacle, a single target region, and two dimensions, with no multi-obstacle, dynamic-obstacle, or 3D scenarios.
- World boundaries are enforced by clamping position rather than by steering away from the edge, which can produce artificial "stuck" behavior near the boundary.
- Comparison of a scenario against baseline (absolute and percentage change) and statistical significance testing across repeated trials are not yet automated, even though the underlying per-run data already supports both.
## What Will be Improved in the Final Simulation
- Tune the goal-seeking and avoidance gains, and the formation logic, so the baseline scenario reliably succeeds before layering perception errors on top of it.
- Clamp or saturate the avoidance force so a single very-close noisy or phantom detection cannot dominate the steering vector.
- Add confidence-weighted detections and a basic fusion layer (naive averaging vs. trust-weighted) to support the two perception-error types from the original proposal that are not yet implemented.
- Add basic flight dynamics, such as acceleration and turn-rate limits, so avoidance maneuvers are more physically plausible.
- Automate baseline-vs-scenario delta computation and basic statistical summaries (e.g. mean and standard deviation, or confidence intervals) directly from the repeated-trial data already being collected.
- Add time-to-goal as an explicit, first-class output metric rather than something only recoverable indirectly from step counts.
- Extend the environment to multiple or dynamic obstacles, and optionally three dimensions, if the prototype's findings justify the added complexity.
