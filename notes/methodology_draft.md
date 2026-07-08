# Methodology Draft 
## Proposed Simulation Approach 
The study uses a 2D simulation of a small UAV swarm operating in a bounded area with one obstacle and one shared target/goal point. Each UAV follows simple goal-seeking behavior combined with collision avoidance logic (reactive, rule-based, not learning-based for the prototype stage). 
Perception is simulated, not physically modeled. Each UAV has a simulated sensor that reports detections of obstacles/other drones. Perception errors are injected into this reported data by manipulating what the UAV "believes" it sees, separately from the ground truth state of the environment. This lets us compare UAV behavior under perfect perception vs. flawed perception using the same underlying scenario. 
The simulation is run scenario by scenario. Each scenario changes one perception condition at a time relative to the baseline (controlled variable approach) so its individual effect on swarm behavior can be isolated - except the three fusion-comparison scenarios, which hold a single harsher combined error profile fixed across all three and vary only the fusion mode (see "Fusion Modes" and "Sensitivity / Experiment Design" below). 

## Simulation Environment, Swarm, and Goal/Obstacle Setup 
All values below are the prototype's actual `simulation_config.json` defaults (not just the original range considered in the proposal); every one of them is configurable. 
- **World**: a bounded 2D area, 100 x 100 units, with no other structure (no walls/rooms/terrain). 
- **Number of UAVs**: 4 (`swarm.num_uavs`), starting near the corners of a small 10x10 cluster at (5,5), (5,15), (15,5), (15,15). The original proposal considered 3-5 as a possible secondary variable; the prototype fixes it at 4 for all scenario comparisons so swarm size is not a confound, and treats varying it as future work (see "What Will be Improved"). 
- **Obstacle**: one static circular obstacle, center (50, 50), radius 5. 
- **Goal setup**: a single shared target point at (90, 90), but each UAV is assigned its own goal *slot* on a small circle around that shared target (rather than literally the same point), so multiple UAVs can all "reach the goal" without indefinitely jostling for one exact spot. 
- **UAV speed**: 2.0 units/step; **time step** `dt` = 0.2s; **max steps** 600 (**duration** up to 120s), whichever comes first, or until every UAV reaches its goal, whichever is sooner. 
- **Sensing**: sensor range 15 units, collision distance 1.5, near-miss distance 3.5, goal tolerance 2.0. 
- **Control gains**: goal-seeking gain 1.0, avoidance gain 6.0, tangential (slide-around) gain 4.0. 

## Perception Errors to be Injected 
- False positive: UAV detects an obstacle/drone that does not exist 
- False negative: UAV fails to detect an obstacle/drone that does exist 
- Sensor noise: reported position/distance is offset from the true value 
- Latency: detection is delayed by a fixed or variable number of time steps 
- Sensor dropout: detection is unavailable for a period of time 
- High-confidence wrong detection: a false detection reported with high confidence, so the UAV treats it as certain 
- Fusion-related errors: differences in how multiple UAVs combine/share detections (naive vs trust-weighted) - see "Fusion Modes" section below for how this is actually implemented 
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
- Fusion recovery count (a detection an individual UAV's own sensor missed, recovered via another UAV's fused detection) 
- Mission success (goal reached without collision: yes/no) 
- Response time (time steps between true detection event and UAV reaction) 
- Formation error (deviation from intended spacing/formation, if applicable) 
- Average confidence error (mean absolute gap between a detection's true and reported confidence, to confirm the miscalibration mechanism is working as configured) 

## Fusion Modes 
Three fusion modes are implemented in `simple_swarm_sim.py` (`fusion_mode` config value), used whenever more than one UAV currently has a real (non-phantom) detection of the same obstacle: 
- **no_fusion** - each UAV uses only its own perception; the legacy/default behavior, and what every non-fusion scenario (baseline, false_positive, false_negative, sensor_noise, latency, sensor_dropout, confidence_error) runs with. 
- **naive_fusion** - an unweighted average of every contributing UAV's detected obstacle position. 
- **trust_weighted_fusion** - the same average, but weighted by each detection's reported confidence value, so more-trusted detections count for more. 

Fusion can recover a detection an individual UAV's own sensor missed (false negative or dropout) as long as at least one other UAV still saw it and the fused position is within the recovering UAV's own sensor range - tracked as `fusion_recovery_count`. Phantom (false-positive) detections are never fused, since each is an uncorroborated, distinct ghost with nothing to average against. 

Fusion is evaluated with its own dedicated three-scenario comparison (`no_fusion_matched`, `naive_fusion`, `trust_weighted_fusion`), all three sharing the identical, harsher combined error profile (false-negative rate 0.2, position-noise std 1.5, confidence-error level 0.15) so that `fusion_mode` is the *only* variable that changes between them - `no_fusion_matched` exists purely as that fusion experiment's own no-fusion control, and should not be confused with, or compared directly against, the plain `baseline`/`confidence_error` scenarios above, which use a different (zero or single-factor) error profile. 

## Sensitivity / Experiment Design 
Two separate controlled-variable designs are used: 
1. **Single-factor perception-error sweep** - baseline plus six scenarios, each changing exactly one perception-error parameter away from baseline's all-zero defaults, at one tested level each: 

  | Scenario | Parameter changed | Level tested | 
  |---|---|---| 
  | baseline | (none) | - | 
  | false_positive | `false_positive_rate` | 0.08 | 
  | false_negative | `false_negative_rate` | 0.25 | 
  | sensor_noise | `position_noise_std` | 1.5 | 
  | latency | `latency_steps` | 5 | 
  | sensor_dropout | `dropout_prob` / `dropout_duration_steps` | 0.02 / 8 | 
  | confidence_error | `confidence_error_level` | 0.35 | 

  Only one level per parameter is tested in the current prototype (no low/medium/high sweep yet); expanding to multiple levels per parameter is listed under "What Will be Improved." 
2. **Fusion-mode comparison** - `no_fusion_matched`, `naive_fusion`, `trust_weighted_fusion` all run the same combined error profile (`false_negative_rate` 0.2, `position_noise_std` 1.5, `confidence_error_level` 0.15) and vary only `fusion_mode`, isolating fusion's effect from the perception-error sweep above. 

Every scenario (10 total, including the 3 fusion-mode scenarios) is run for 5 seeded trials each (seeds 42-46, `sim.seed` incremented by 1 per run), giving 50 total runs, so results reflect an average and a visible run-to-run spread rather than a single deterministic pass. 

## How Results Will Be Compared 
Each scenario is run against the same baseline scenario (no perception error, identical map, identical start/goal positions). For each scenario: 
- Metrics from the scenario's 5 runs (averaged) are compared against baseline's 5-run average 
- Differences can be read directly off `results/results_summary.csv`, which has one row per scenario/run, or off the `plots/` charts that already draw the baseline average as a reference line; automated absolute/percentage-change computation and formal statistical summaries (mean/stdev, confidence intervals) across the 5 seeds per scenario are not yet built on top of that data - see "Current Limitations" 
- Behavior differences are also described qualitatively, both from reading the per-step CSV logs and from watching the animated replays produced by `simulation_visualizer.py` (see "Visualizer and Replay Method" below) - e.g. "swarm took a longer path", "one UAV stalled near the obstacle" 
- Results are logged to CSV per run (`logs/<scenario>_run<n>.csv`, one row per UAV per step) so scenarios can be compared side by side, replayed, or re-aggregated later 
- Repeated trials with averaging (5 seeded runs per scenario, described above) are already implemented, superseding the original proposal's plan to defer this to a later phase; formal statistical significance testing across those 5 seeds is still future work 
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
This ground-truth set is what the UAV *would* see with perfect sensing. It is passed to that UAV's `Perception` object, which is the only place errors are introduced (see below) real or phantom or a combination, joined as e.g. `false_positive+latency` whichever single entity is closest to a given UAV is checked, and if that distance is within the near-miss threshold the step is flagged as a risk event for that UAV. 
- Unnecessary avoidance count: incremented on any step where a UAV received one or more detections but none of them were real (i.e. phantom-only), and the avoidance behavior still triggered. 
- Missed response count: incremented on any step where a real threat was within the near-miss threshold in ground truth but did not appear in what the UAV perceived that step. 
- Response time: for each real threat, the time between the step it first became detectable in ground truth and the step the UAV first perceived it (after any latency delay), averaged across all such events and converted to seconds. 
- Formation error: the root-mean-square deviation of every pairwise UAV-to-UAV distance from the desired formation spacing, averaged over the run. 
- Mission success: true only if every UAV reached its goal tolerance and zero collisions occurred at any point during the run. 
- Aggregation across runs: metrics_analysis.py runs each scenario for multiple seeded trials (the random seed is incremented per run) and writes one row per scenario/run to the summary CSV, so results can be examined per-trial or averaged. 
## Visualizer and Replay Method
`simulation_visualizer.py` provides both post-hoc replay and (API-level) live viewing, built on `matplotlib.animation`:
- **Replay from CSV**: loads any `logs/<scenario>_run<n>.csv` and steps through it frame by frame, drawing each UAV as a labeled dot, its accumulated trajectory, its goal slot, the real obstacle (solid), the perceived obstacle (dashed, when currently detected), and a collision-risk zone around any UAV whose `collision_risk_flag` is set. An info box overlays the current scenario, step, time, action taken, perception-error type, and mission status.
- **Mission status overlay**: rather than only reflecting the current row's `mission_completed_flag`, the visualizer scans the whole log up front to determine whether the run ever succeeds, so it can show a definitive **SUCCESS** (green) once that happens, or **FAILURE** (red) once the log's final step is reached without it ever happening - instead of an uninformative "in progress" for a run that is actually going to fail.
- **Interactive mode**: an on-screen window with Left/Right arrow-key stepping.
- **Video export**: any replay can be exported to MP4 or GIF; batch mode does this automatically for every scenario's `run1` log into `media/`.
- **Fusion comparison video**: a dedicated side-by-side render places `no_fusion_matched`, `naive_fusion`, and `trust_weighted_fusion` in synced panels in one video, for direct visual comparison of the fusion-mode experiment.
- **Live view (`LiveSimulationView`)**: an API meant to be called once per timestep from inside a running simulation's step loop (not just replaying an already-finished log), so movement can be watched as it happens rather than only afterward. This exists in `simulation_visualizer.py` but is not yet called from `simple_swarm_sim.py`'s own loop - wiring it in is listed under "What Will be Improved."

## Analysis Method
Quantitative and qualitative analysis are combined:
- **Quantitative**: `metrics_analysis.py` imports the `Simulation` class directly, runs every scenario for 5 seeded trials, and writes `results/results_summary.csv` (one row per scenario/run). `generate_plots.py` reads that CSV and renders comparison charts (per-scenario metric bars against the baseline average, and cross-scenario/cross-fusion-mode comparisons) into `plots/`.
- **Qualitative**: the animated replays and exported videos from `simulation_visualizer.py` are used to sanity-check and narrate what the numbers mean in practice - e.g. confirming visually that a high collision-risk count in `sensor_noise` corresponds to UAVs actually clustering incorrectly around a mis-perceived obstacle position, rather than treating the metric as a number in isolation.

## Current Limitations 
- ~~The baseline scenario itself does not reliably succeed across repeated trials...~~ **Resolved.** The original steering bug (every teammate treated as a full-range repulsion threat, including one already parked at its own goal) has been fixed; baseline now succeeds 5/5 seeded runs, and `mission_success` is driven purely by whether an actual collision occurred rather than by UAVs never reaching their goal slot. 
- ~~There is no confidence or fusion model...~~ **Resolved.** Confidence values and all three fusion modes (`no_fusion`, `naive_fusion`, `trust_weighted_fusion`) are implemented and covered by a dedicated 3-scenario comparison; see "Fusion Modes" above. Trust-weighted fusion's result is counterintuitive (worse hard-collision rate than naive fusion under confidence miscalibration) and is flagged as needing dedicated follow-up, not as unimplemented. 
- Hard collision count (`Simulation.collision_count`, the raw number of collision events per run) is computed internally and drives `mission_success`, but is not currently written to `results/results_summary.csv` or the per-step CSV logs - only the derived Yes/No `mission_success` is persisted. Getting the exact per-run hard-collision numbers currently requires re-running the scenario directly against `simple_swarm_sim.py`'s `Simulation` class rather than reading them off the saved CSVs. 
- The avoidance force scales inversely with distance without an upper bound, so a detection very close to a UAV, especially a noisy or phantom one, can produce a disproportionately large steering correction. 
- UAV movement has no real flight dynamics: no inertia, acceleration limits, or turn-rate limits, so behavior resembles an idealized reactive agent more than a realistic drone. 
- The environment is limited to a single static obstacle, a single target region, and two dimensions, with no multi-obstacle, dynamic-obstacle, or 3D scenarios. 
- World boundaries are enforced by clamping position rather than by steering away from the edge, which can produce artificial "stuck" behavior near the boundary. 
- Comparison of a scenario against baseline (absolute and percentage change) and statistical significance testing across repeated trials are not yet automated, even though the underlying per-run data already supports both. 
- Only one parameter level per perception-error type has been tested so far (see "Sensitivity / Experiment Design"); there is no low/medium/high sweep yet. 
- `LiveSimulationView` (in `simulation_visualizer.py`) is not yet called from `simple_swarm_sim.py`'s own step loop, so true live viewing of an in-progress run isn't wired up end-to-end yet - only CSV replay and the `--mode live-demo` API test path are. 
## What Will be Improved in the Final Simulation 
- Write `collision_count` (raw hard-collision events per run) into `results/results_summary.csv` directly, instead of only the derived Yes/No `mission_success`, so exact collision numbers don't require re-running the simulation. 
- Investigate and resolve the trust-weighted fusion result (worse hard-collision rate than naive fusion under confidence miscalibration) - either fix the weighting formula or confirm and document it as an expected failure mode. 
- Wire `LiveSimulationView` into `simple_swarm_sim.py`'s step loop so a run can be watched live, not just replayed afterward. 
- Clamp or saturate the avoidance force so a single very-close noisy or phantom detection cannot dominate the steering vector. 
- Add basic flight dynamics, such as acceleration and turn-rate limits, so avoidance maneuvers are more physically plausible. 
- Automate baseline-vs-scenario delta computation and basic statistical summaries (e.g. mean and standard deviation, or confidence intervals) directly from the repeated-trial data already being collected. 
- Expand the single-factor sensitivity sweep to multiple levels per parameter (e.g. low/medium/high false-positive rate) rather than one tested level each. 
- Add time-to-goal as an explicit, first-class output metric rather than something only recoverable indirectly from step counts. 
- Extend the environment to multiple or dynamic obstacles, and optionally three dimensions, and treat swarm size (currently fixed at 4) as a tested variable, if the prototype's findings justify the added complexity. 