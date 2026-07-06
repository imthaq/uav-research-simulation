# UAV Swarm Perception-Error Simulation

## What this is

A 2D simulation of a UAV swarm flying from start positions to a shared target while avoiding one obstacle and each other. Each UAV's perception of the world can be corrupted (false positives, false negatives, position noise, latency, sensor dropout, miscalibrated confidence) to study how perception errors - and cross-UAV sensor fusion that tries to compensate for them - affect swarm safety and mission success.

## Folder layout

```
simulation_prototype/
  simple_swarm_sim.py        - core simulation (world, perception, fusion, control, logging)
  simulation_config.json     - world, swarm, sensing, control, and scenario settings
  run_experiments.py         - runs every scenario for N repeated trials, saves per-run CSVs
  metrics_analysis.py        - aggregates runs into results_summary.csv, prints summary stats
  generate_plots.py          - builds PNG charts from results_summary.csv
  initial_results_summary.md - narrative summary of findings
  logs/                      - one CSV per scenario per run, e.g. baseline_run1.csv, plus simulation_log.csv (combined)
  results/                   - results_summary.csv (aggregated per-run metrics, only copy)
  plots/                     - PNG charts generated from results_summary.csv
```

## How to run

Run everything (all scenarios x 3 repeated trials, one CSV per run, aggregated summary):

    python run_experiments.py --runs 3

Run just the simulation once per scenario (writes logs/simulation_log.csv):

    python simple_swarm_sim.py --config simulation_config.json --log logs/simulation_log.csv

Optional: `--scenario <name>` to run only one scenario instead of all.

Aggregate metrics across seeded runs on their own (writes results/results_summary.csv):

    python metrics_analysis.py --runs 5

Generate charts from the aggregated summary:

    python generate_plots.py

## World setup

* 100 x 100 world (`world.width` / `world.height`)
* Shared target/goal at (90, 90) (`world.target`)
* One circular obstacle at (50, 50), radius 5 (`world.obstacle`)
* 4 UAVs starting near (5-15, 5-15) (`swarm.num_uavs`, `swarm.start_positions`)
* Each UAV gets its own goal point arranged in a small circle around the
shared target (not the exact same point), so they don't all try to
crowd into one spot at the end

## Configurable parameters (simulation_config.json)

`swarm`: number of UAVs, start positions, UAV speed, formation spacing, safety distance
`world`: area size, goal/target position, obstacle position
`sim`: time step, max steps, simulation duration (seconds), random seed
`sensing`: sensor range, collision distance, near-miss distance, goal tolerance
`control`: goal-seeking gain, avoidance gain, tangential (slide-around) gain
`perception_errors` (also overridable per scenario):
* `false_positive_rate` - chance of a fake ("phantom") detection each step
* `false_negative_rate` - chance a real detection is missed
* `position_noise_std` - Gaussian noise added to detected x/y position
* `latency_steps` - delay between when a detection happens and when the controller receives it
* `dropout_prob` / `dropout_duration_steps` - chance of a temporary sensor blackout (no detections at all) lasting several steps
* `confidence_error_level` - how far a detection's *reported* confidence can drift from its true confidence (miscalibration)
* `fusion_mode` - `no_fusion`, `naive_fusion`, or `trust_weighted_fusion`

## Perception model

Each UAV senses obstacles and other UAVs within `sensor_range`. What it
actually perceives (vs. ground truth) can be corrupted per scenario using
the parameters above. Every real or phantom detection also carries a
**confidence value** (0-1): higher for close, clean, real detections;
moderate for phantoms (a false detection typically still looks clean to the
sensor that produced it, which is what makes it dangerous to trust);
optionally miscalibrated by `confidence_error_level`.

## Sensor fusion model

When more than one UAV currently has a real (non-phantom) detection of the
obstacle, the fusion step can combine those independent detections into one
shared belief instead of each UAV using only its own noisy view:

* **no_fusion** - each UAV uses only its own perception (default/legacy behavior).
* **naive_fusion** - unweighted average of every contributing UAV's detected obstacle position.
* **trust_weighted_fusion** - same average, but weighted by each detection's confidence value, so more-trusted detections count for more.

Fusion can *recover* a detection an individual UAV's own sensor missed
(false negative or dropout) as long as at least one other UAV still saw it
and the fused position is within the recovering UAV's own sensor range -
this is logged per run as `fusion_recovery_count`. Phantom (false-positive)
detections are never fused, since each is a distinct, uncorroborated ghost
with nothing to average against.

## Control model

Simple potential-field steering:

* Goal-seeking vector toward the UAV's target
* Repulsive vector away from each perceived threat (obstacle or other UAV, after fusion where applicable), strength ~ avoidance_gain / distance
* Tangential vector (perpendicular to repulsion) so the UAV slides around a threat instead of stalling head-on, strength ~ tangential_gain / distance

Combined vector is normalized and scaled by uav_speed.

## Scenarios (in simulation_config.json)

Each scenario runs a minimum of 3 repeated trials with different seeds:

1. `baseline` - no perception error, no fusion
2. `false_positive` - phantom detections only
3. `false_negative` - missed detections only
4. `sensor_noise` - noisy detected positions
5. `latency` - delayed detections
6. `sensor_dropout` - periodic sensor blackouts
7. `confidence_error` - miscalibrated confidence values, no fusion to compensate
8. `naive_fusion` - moderate noise + missed detections, fused by unweighted averaging
9. `trust_weighted_fusion` - same error conditions as naive_fusion, fused by confidence-weighted averaging

## Log output (one row per UAV per step)

Every run CSV (`logs/<scenario>_run<n>.csv`, `logs/simulation_log.csv`) includes:

* `scenario`, `step`, `time_s`, `uav_id`
* `uav_pos_x`, `uav_pos_y` - UAV position
* `goal_pos_x`, `goal_pos_y` - this UAV's goal position
* `actual_obstacle_x`, `actual_obstacle_y` - ground-truth obstacle position
* `perceived_obstacle_x`, `perceived_obstacle_y` - what the UAV (post-fusion, post-latency) believes the obstacle position is; blank if not currently detected
* `perception_error_type` - which corruption mechanism(s) fired this step (`none`, or a `+`-joined combo of `dropout`/`false_positive`/`false_negative`/`position_noise`/`latency`/`confidence_error`)
* `confidence_value` - confidence attached to the perceived obstacle detection (blank if none)
* `fusion_mode` - the fusion mode active for this run
* `action_taken` - one of: `move`, `avoidance`, `false_avoidance`, `at_goal`
* `num_perceived_detections`, `num_phantom_detections`
* `dist_to_goal`
* `distance_to_nearest_uav` - distance to the closest other UAV
* `distance_to_obstacle` - distance to the obstacle surface
* `nearest_entity_type`, `nearest_entity_distance` - closest entity of either kind (obstacle or UAV) and its distance
* `collision_risk_flag` - nearest entity within near_miss_distance
* `unnecessary_avoidance_flag` - avoidance triggered by a phantom only
* `missed_response_flag` - a real nearby threat that was not perceived
* `mission_completed_flag` - all UAVs reached goal AND zero collisions, as of this step
* `reached_goal` - whether this UAV has reached its own goal slot yet

## Aggregated metrics (results/results_summary.csv)

One row per scenario per run:

* scenario, run_number, fusion_mode
* false_positive_rate, false_negative_rate, noise_level, latency_steps, dropout_probability, confidence_error_level
* collision_risk_count, unnecessary_avoidance_count, missed_response_count, fusion_recovery_count
* mission_success, avg_response_time_s, total_near_misses, avg_formation_error

## Metrics (printed to stdout per scenario)

* steps_run
* uavs_reached_goal / num_uavs
* mission_success (all UAVs reached goal AND zero collisions)
* collision_count
* near_miss_count
* unnecessary_avoidance_count - avoidance triggered by a phantom only
* missed_response_count - a real nearby threat that was not perceived
* avoidance_action_count - avoidance triggered by a real detection
* fusion_recovery_count - steps where fusion supplied an obstacle detection a UAV had individually missed
* avg_response_time_s - average delay between a threat becoming real (within near_miss_distance) and the UAV actually perceiving it
* avg_formation_error - RMSE of inter-UAV distance vs. desired_formation_spacing