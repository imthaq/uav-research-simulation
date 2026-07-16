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
---

## Methodology Update: Radar-Like Sensing, Multimodal Fusion, and Experiment Design

Everything above describes the original prototype: geometric detection, a
single fixed noise value, and three fusion modes. The simulation has since
grown a full probabilistic sensor stack, multiple fusion architectures, and
a proper experiment/analysis pipeline on top of it. This section documents
what was added, grounded in what's actually implemented today.

### Probabilistic radar measurement model
Every reported measurement (a real detection or a confirmed clutter false
alarm) now carries explicit measurement uncertainty instead of a single
fixed noise value applied blindly. `radar_like_model.py` computes, per
detection, a probability of detection and a probability of false alarm
that are *recomputed each time* from the target's SNR, the environmental
condition, and the radar's own reliability state - rather than reading one
static config number every time. This is what actually gates the
Bernoulli detection roll and the clutter-confirmation roll, so a distant
or degraded-condition target is genuinely harder to detect, not just
logged afterward as "should have been harder to detect."

### Range-dependent sensing behavior
Signal quality falls off with range following a radar-equation-style SNR
proxy (`_snr_db_for_range`, 4th-power falloff by default), squashed to a
`[0,1]` measurement-quality factor (`_quality_from_snr`). That quality
factor scales every measurement's variance up and its effective
probability of detection down as range increases - so a target near the
edge of `radar_max_range` is both noisier and more likely to be missed
than one close to the radar, instead of every in-range detection being
equally trustworthy. Two categorical knobs compound this:
`radar_environmental_condition` (clear/rain/fog/storm) and
`radar_reliability_state` (nominal/degraded/critical), each degrading
noise, PD, and PFA by its own multiplier.

### Measurement covariance
Each detection reports a 3x3 covariance matrix over
`(range, bearing, radial_velocity)`, serialized as `measurement_covariance`
in the CSV row. Channels are modeled independently (diagonal covariance)
- there's no cross-channel correlation model - but every diagonal entry
scales with range, environmental condition, and reliability state, so the
covariance a UAV reports is an honest reflection of how uncertain that
particular measurement actually is, not a constant.

### Clutter distribution
Radar false alarms are generated independently of the older
`false_positive_rate` "phantom" mechanism. Each step, the number of
clutter candidate returns is drawn from `clutter_distribution` -
`"poisson"` (default, rate `clutter_lambda`) or `"fixed"`
(`round(clutter_lambda)` every step) - positioned uniformly inside a
`[clutter_range_min, clutter_range_max]` annulus around the radar,
independent of the real-target detection range window. Each candidate is
then confirmed as a reported false detection with probability
`radar_false_alarm_probability` (itself scaled by the environmental/
reliability multipliers), and confirmed clutter is injected into the same
detection stream a UAV steers on, marked as phantom.

### Kalman tracking
`radar_track_model.py`'s `RadarTracker` replaced the earlier
exponential-smoothing tracker with a proper constant-velocity Kalman
filter per track (4-state: position + velocity), predicting forward every
step and updating on a matched detection. Each track carries a five-state
lifecycle - `tentative` (just created) → `confirmed` (matched
`CONFIRM_HITS`=3 times running) → `coasting` (predicted forward on a
miss) → `lost` (missed `MAX_MISSED`=3 times running) → `deleted` (final
row) - plus a recursive `existence_probability` that rises on hits and
decays on misses, giving a second, independent signal from raw track
status.

### Data association
Detection-to-track matching uses gated nearest-neighbor association: for
every (track, detection) pair, the Mahalanobis distance to the track's
predicted position is computed using the filter's own innovation
covariance, and only pairs inside the gate (`GATE_CHI2`=9.21, a
chi-squared threshold rather than a fixed Euclidean radius) are candidate
matches. Candidates are sorted by distance and claimed greedily, closest
first, so each track and each detection is used at most once per step.
The same idea is reused one level up in `fusion_model.py` for
track-to-track association across UAVs (greedy single-linkage clustering
by position, `CLUSTER_DISTANCE`=4.0), before those clustered tracks are
combined by whichever fusion mode is active.

### Multimodal sensor models
Three independent sensor models now exist, each with its own detection
probability, noise profile, field of view, and failure behavior:
**radar** (`radar_like_model.py` - range/bearing/Doppler-like, longest
range, the only modality currently fed into cross-UAV fusion), **vision**
(`vision_like_model.py` - strong classification confidence but a limited
FOV, sensitive to occlusion and lighting/environmental conditions), and
**LiDAR** (`lidar_like_model.py` - the most accurate position/range, but
the shortest range and prone to dropout in adverse weather). Vision and
LiDAR are generated and logged but not yet wired into `fusion_model.py`'s
cross-UAV combination (`_generate_vision_lidar_detections()` in
`simple_swarm_sim.py` is the marked extension point); today's fusion
results should be read as radar-only fusion.

### Asynchronous updates
Each sensor model has its own configurable update rate in Hz
(`radar_update_rate`, `vision_update_rate`, etc.), converted to an
`update_interval_steps` from the simulation's `dt` - radar and vision
default near 5 Hz, LiDAR slower. On steps that fall between a sensor's
actual updates, a per-UAV hold buffer re-serves the last row that sensor
actually generated rather than fabricating a fresh reading, and every row
carries `measurement_age_steps` / `is_stale` so downstream tracking and
fusion can tell an actually-fresh measurement from a held-over one.

### Centralized / distributed fusion
`fusion_model.py` separates *which weighting scheme* combines tracks
(naive/confidence/trust/covariance/CI - unchanged by this axis) from
*where* that combination happens. **Centralized** (the default, and what
`fuse_step` always did before this axis existed) has every UAV uplink its
track to one node - a ground station or lead UAV - which fuses once and
broadcasts a single shared estimate back out; one common answer, at the
cost of an uplink+downlink round trip before anything is usable, and a
single point of failure at the central node. **Distributed** has no
central node: each UAV broadcasts a lightweight summary of its own track
to its peers, and separately fuses locally over whatever peer summaries
actually arrived that step - since each peer-to-peer broadcast can
independently fail, different UAVs can end up with slightly different
local estimates of the same object in the same step.

### Communication uncertainty
`communication_model.py`'s `CommunicationChannel` models what a track
message actually has to survive to get where it's going: a per-message
`packet_loss_probability` (outright drop), `comm_range` (sender/receiver
pairs farther apart simply can't hear each other), `base_latency_steps`
(fixed extra delay on top of the sender's own sensor latency),
`max_staleness_steps` (a receiver hard-rejects a report older than this,
regardless of whether it was otherwise delivered), and
`corruption_probability` (the reported confidence/reliability value
arrives scaled by a random factor, modeling bit errors rather than
outright loss). This channel is what the distributed architecture's
peer-to-peer broadcasts - and the centralized architecture's uplink/
downlink - travel over; nothing here reads ground truth, so a receiver's
range gating and staleness checks use only the sender's own reported
position, exactly what a real inter-UAV link would have available.

### Dynamic trust
Everything in a track's instantaneous `reliability` score (confidence,
status, measurement age, latency, dropout state) is recomputed from
scratch every step, with no memory of what that source did previously - so
a sensor that self-reports high confidence every single step looks fine
even if it was wrong every one of those steps too. `TrustTracker` (in
`fusion_model.py`) adds a second, slow-moving `persistent_trust` score per
UAV/radar that *does* carry memory across a run: it decays when a source's
estimate repeatedly disagrees with its cluster-mates, is corroborated by
nobody while others nearby are seeing something, goes stale, or drops out
often; it recovers, more gradually than it decays, when the source starts
agreeing again, reports fresh data, or its confidence/covariance improve.
`persistent_trust` is folded directly into the same composite reliability
score every fusion mode already reads, so dynamic trust benefits
covariance-weighted fusion too, not only `trust_weighted_fusion`.

### Monte Carlo experiment design
`run_experiments.py` runs every scenario for a configurable number of
repeated trials (`--trials`, defaulting to `config["reproducibility"]
["trial_count"]`), each trial re-seeded so the run captures a scenario's
*distribution* of outcomes rather than one run's outcome - necessary
whenever a scenario has run-to-run variance from noise, dropout, or
fusion-mode comparisons. Two seed strategies are supported: `sequential`
(seed = base_seed + trial - 1, shared across scenarios, so trial N of
every scenario is comparable at matched noise draws) and `random`
(independent per-(scenario, trial) seeds from a single seeded RNG stream,
for a genuine unyoked Monte Carlo sweep). Guidance embedded in the runner:
>= 20 trials for exploratory analysis, 50-100 for paper-ready results
(with `--skip-step-logs` to drop the dominant per-trial-log disk/time cost
at that scale).

### Statistical analysis
`statistical_analysis.py` turns the raw per-run results into inferential
statistics rather than just descriptive averages: means/standard
deviations/confidence intervals per scenario and fusion mode, Cohen's d
effect size between fusion modes, correlation between perception
parameters (noise, dropout, latency, ...) and outcome metrics,
ANOVA/Kruskal-Wallis tests across fusion modes, paired comparisons (e.g.
naive vs. trust-weighted on the same seeds), and significance tests
specifically on mission success, collision risk, and response time - so a
claim like "trust-weighted fusion is better" can be backed by a
significance test on matched trials, not just a difference in sample
means.

### Ablation study
`ablation_experiments.py` isolates each fusion/tracking component's
individual contribution by disabling exactly one at a time and re-running
a subset of scenarios: no radar tracking (detection probability forced to
0), no confidence estimation (confidence forced to 1.0 for every source),
no trust weighting (forced to `naive_fusion`), no covariance weighting, no
sensor latency, no stale-data rejection (dropout/age decay disabled), no
inter-UAV communication/fusion at all (forced to `no_fusion`), and no
dynamic trust adaptation. Each ablated variant's fusion error, collision
risk, mission success, response time, and formation error are compared
against the full system, so a claim that a given component matters is
backed by "removing it made things measurably worse," not assumed from
the design alone.

### Stress testing
`stress_test_results.md` pushes the system to its failure boundaries
rather than only its typical operating range: 12 scenarios covering very
low probability-of-detection, very high false-alarm rate, heavy clutter,
high latency, high dropout, several simultaneous sensor failures at once,
a full communication outage, closely-crossing targets, a suddenly-
appearing hazard, a fast-approaching obstacle, an overconfident-but-wrong
("liar") sensor, and a deliberately wrong trust initialization. Each is
documented with expected vs. actual behavior, the specific failure point
(the parameter value or duration at which the system stops coping
gracefully), the recovery behavior once conditions improve, and a
qualitative safety-implication rating - so the system's resilience
boundaries are characterized explicitly instead of only reporting
average-case metrics that stay silent about how it fails.

### Reproducibility procedure
Every experiment run is traceable back to exactly the conditions that
produced it. `simulation_config.json`'s `reproducibility` block records
the trial count and output location a run defaults to (still overridable
per-invocation via CLI flags) alongside the tracker's lifecycle/gating
constants. `run_experiments.py` writes an `experiment_metadata.json`
alongside every results batch containing: the full resolved config used
(plus a SHA-256 fingerprint of it, for a cheap drift check without
re-diffing the whole file), the trial count, base seed and seed-mode, the
exact per-scenario seed list actually used, every output file path the
run wrote to, wall-clock duration, and the environment (Python version,
platform) it ran on. Any results CSV can therefore be traced back to a
metadata file recording precisely the config and seeds that generated it.