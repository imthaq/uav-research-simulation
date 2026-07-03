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
The prototype is implemented as two components. The first, simple_swarm_sim.py, is the simulation engine: it contains a Perception class (one instance per UAV, responsible for corrupting that UAV's detections), a Simulation class (owns world state, steps time forward, applies each UAV's controller, counts collisions/near-misses, writes a log row per UAV per step), and a main() entry point that loads simulation_config.json, runs one or all named scenarios, and writes a per-step CSV log. The second, metrics_analysis.py, is the analysis layer: it imports the Simulation class directly, runs each scenario for several seeded trials, and writes a summary CSV with one row per scenario/run. A third script, generate_plots.py, reads that summary CSV and renders the comparison charts.
The simulation is fully config-driven: world size, obstacle placement, swarm size/speed/formation spacing, sensor range, controller gains, and per-scenario perception-error parameters all live in simulation_config.json. Each named scenario only overrides the specific perception-error parameters it's testing; any parameter it doesn't set falls back to a default.

## Simulation Flow
A single scenario run proceeds as follows:
1. `Simulation.__init__` loads the world (bounds, obstacle), the swarm (start positions, speed, formation spacing), and the sensing/control parameters from the config, seeds a single shared random number generator from `sim.seed`, assigns each UAV its own goal slot on a small circle around the shared target, and creates one `Perception` instance per UAV.
2. `Simulation.run()` calls `step()` once per time increment, up to `max_steps`, and stops early the moment every UAV has reached its goal.
3. Inside each `step()` call, every UAV that hasn't yet reached its goal: gets its ground-truth detections, has those detections corrupted by its `Perception` object, buffers the corrupted detections for however many steps of latency are configured, steers using whatever detection set has actually "arrived" by that step, and moves.
4. After all UAVs have moved, the step checks every UAV-obstacle and UAV-UAV pair for collisions/near-misses, records each UAV's single nearest entity (for the per-step collision-risk flag), updates formation-error samples, updates each UAV's reached-goal flag, and appends one log row per UAV.
5. Once the run ends (goal reached by all, or `max_steps` exhausted), `_metrics()` aggregates the run into a single summary dict (mission success, collision/near-miss counts, unnecessary-avoidance/missed-response counts, average response time, average formation error).
`metrics_analysis.py` wraps this same flow in a loop: for each scenario it constructs a fresh `Simulation` with an incremented seed, runs it, and appends one row to the results-summary CSV per scenario/run.

## How UAV Movement is Modeled
Each UAV is modeled as a point mass with no inertia, using a reactive potential-field controller rather than real flight dynamics. Movement is computed from two combined vectors each step:
- A goal vector: the normalized direction from the UAV's current position to its assigned goal slot, scaled by a goal gain. UAVs do not all steer toward one identical shared point; each is assigned its own slot on a small circle around the target so multiple UAVs can settle at "the goal" without indefinitely jostling for the same spot.
- An avoidance vector: for every currently perceived threat (obstacle or other UAV) within sensor range, a radial repulsion term pointing away from the threat (strength = avoidance_gain / distance) plus a tangential term perpendicular to it (strength = tangential_gain / distance), so the UAV slides around a threat instead of stalling head-on against it. A detection only counts as triggering "real" avoidance if its repulsion strength exceeds a small threshold and it isn't a phantom.
These vectors are summed, renormalized, and scaled by the UAV's speed to produce a velocity; position is then updated by velocity times the simulation time step and clamped to the world boundaries. There is no acceleration limit, turn-rate limit, or momentum in the prototype, so velocity can change direction instantly from one step to the next.

## How Obstacle Detection is Simulated
Detection is purely geometric, not physically modeled (no line-of-sight occlusion, field-of-view cone, or signal model). Each step, for each UAV, ground truth is computed independently of perception:
- The obstacle is a "true" detection whenever the UAV's distance to the obstacle's edge (center distance minus radius) is within sensor range.
- Every other UAV is a "true" detection whenever it's within sensor range.
This ground-truth set is what the UAV *would* see with perfect sensing. It is passed to that UAV's `Perception` object, which is the only place errors are introduced (see below) — the ground-truth computation itself is never corrupted. The simulation separately tracks, per threat, the first step it became "real" (within the near-miss distance) and the first step the UAV actually perceived it, which is what lets response time and missed-response counts be computed later.

## How False Positives are Injected
With a configured probability, checked once per UAV per step, a single phantom detection is fabricated: a random angle and a random distance between 2.0 and the sensor range are chosen, a point is placed at that offset from the UAV's current position, and it's added to the perceived-detections list flagged `is_phantom`. A phantom has no corresponding real obstacle or UAV; because the false-positive check runs independently of the false-negative/noise steps, a step can contain a phantom detection alongside real ones. Whether an avoidance maneuver was triggered by a phantom-only detection set (no real detections that step) is what the "unnecessary avoidance" metric tracks.

## How False Negatives are Injected
Each true detection is evaluated independently: a random draw is compared against the configured false-negative rate, and the detection is silently dropped if the draw falls below that rate, otherwise it survives into the perceived set. Because the check is per-detection, a UAV can miss one nearby threat (e.g. the obstacle) while still perceiving another (e.g. a nearby UAV) in the same step. Every dropped detection's ID is recorded for that step, which is what lets the log report a false-negative error type and lets the missed-response count be incremented whenever a real threat within the near-miss distance failed to appear in what the UAV perceived.

## How Latency, Dropout, and Noise are Injected
- **Position noise**: if a noise standard deviation is configured, each surviving (non-dropped) detection has independent Gaussian jitter added to its reported x/y position, and its reported distance is recalculated from the jittered position rather than reused from ground truth.
- **Sensor dropout**: checked first, before any other error type, each step. If a blackout is already in progress, or a new one probabilistically triggers, the UAV receives no detections at all that step — real or phantom — and none of the other error mechanisms run for that step. A newly-triggered blackout lasts a configured number of steps (defaulting to 5 if unset).
- **Latency**: applied after all other corruption, independent of dropout/noise/false-positive/false-negative. Each step's already-corrupted detection set is pushed into a per-UAV buffer tagged with the step it was generated; the controller only receives a buffered entry once enough steps have passed to satisfy the configured latency, and consumed/stale buffer entries are discarded so the buffer stays bounded. Zero latency means the current step's detections are used immediately, as if no buffering happened.
Each `Perception` instance records which of these mechanisms fired on a given step (dropout, false positive, false negative, position noise, latency), which is what allows the per-step log to report an error type — or a combination, joined as e.g. `false_positive+latency` — alongside the UAV's position and action for that step.

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
