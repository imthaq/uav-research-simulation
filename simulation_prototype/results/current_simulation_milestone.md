# Current Simulation Milestone — Baseline Perception-Uncertainty Simulation

**Status:** Frozen baseline milestone, prior to radar-like sensor model rework.
**Date frozen:** 2026-07-09

This document is a snapshot of the perception-error swarm simulation as it
stands today, before the structure changes for the radar-like sensing
version. It exists so the current, working state can always be recovered
later, even after the codebase changes shape.

---

## 1. What has already been completed

- Core 2D swarm simulation (`simple_swarm_sim.py`): world, potential-field
  control (goal-seeking + repulsion + tangential slide-around), perception
  model, cross-UAV sensor fusion, and per-step CSV logging.
- 4-UAV swarm flying from start positions to a shared target, avoiding one
  static circular obstacle and each other, with per-UAV goal slots arranged
  around the shared target.
- Perception-error injection: false positives (phantom detections), false
  negatives (missed detections), Gaussian position noise, latency (delayed
  detections), sensor dropout (temporary blackouts), and confidence
  miscalibration.
- Cross-UAV sensor fusion with three modes: `no_fusion`, `naive_fusion`
  (unweighted average), `trust_weighted_fusion` (confidence-weighted
  average), including fusion-based recovery of individually-missed
  detections (`fusion_recovery_count`).
- Critical bug fix in `_steer`: teammates were repelling each other across
  the full 15-unit sensor range instead of only when genuinely close, which
  made `mission_success` false in 100% of runs (including baseline). Fixed
  by tying teammate avoidance to `uav_avoidance_range`/`safety_distance`,
  and treating a UAV that already reached its goal as a stationary waypoint
  with a tight safety margin instead of a standing threat. After the fix,
  `uavs_reached_goal` is 4/4 in every run, and `mission_success` reflects
  real collision outcomes.
- Experiment orchestration (`run_experiments.py`): runs every scenario for
  N repeated trials with different seeds, writes one CSV per run to `logs/`.
- Metrics aggregation (`metrics_analysis.py`): aggregates per-run CSVs into
  `results/results_summary.csv` (one row per scenario per run) and prints
  summary stats.
- Chart generation (`generate_plots.py`): builds PNG charts from
  `results_summary.csv` into `plots/`.
- Simulation visualizer (`simulation_visualizer.py`): replays any scenario's
  CSV log as an animated matplotlib view, exports MP4/GIF, batch mode that
  auto-generates one video per scenario plus a fusion-mode comparison video,
  and a `LiveSimulationView` class for watching a run live while it executes.
- **Live view is now wired in — entirely inside `simulation_visualizer.py`.**
  `simple_swarm_sim.py` is unchanged and still has zero knowledge of
  matplotlib or visualization; it only runs scenarios and produces log rows,
  exactly as before. All visualization — both the existing CSV replay path
  and the new live path — is owned by `simulation_visualizer.py`. A new
  `run_live_scenario()` function there imports `Simulation` from
  `simple_swarm_sim.py`, drives its step loop directly (mirroring
  `Simulation.run()`'s own loop), and feeds each step's rows straight to a
  `LiveSimulationView` as they're generated — no CSV round-trip needed.
  Exposed via a new `--mode live` CLI option (`--scenario`, `--out-log`,
  `--live-hold`), alongside the pre-existing `--mode live-demo` (which
  replays an *already-finished* CSV through the same live-view API, useful
  for testing the view itself without running a real simulation).
- Fixed a real bug found while building this: the mission status readout
  used to compute "is this the last step?" from `self.data.steps`, which
  live mode updates to `step + 1` on every single call — so it would flag
  **FAILURE** on nearly every frame until the mission actually succeeded.
  Fixed by adding a `SimulationData.is_live` flag: while live, status only
  ever reads SUCCESS or "In Progress"; `LiveSimulationView.close()` (called
  once the run loop ends) flips `is_live` off and re-renders the final
  frame so the true FAILURE/SUCCESS result is shown and held on screen
  briefly before the window closes. Verified with both a passing scenario
  (`baseline`) and a failing one (`sensor_noise`, 0/5 success) — status now
  reads correctly in both cases, and the CSV written by live mode matches
  the shape/values of a normal `simple_swarm_sim.py` run.
- Narrative results write-up (`initial_results_summary.md`) covering all 9
  scenarios, the mission-success bug fix, and the naive-vs-trust-weighted
  fusion finding.

## 2. Scenarios tested

All 9 scenarios from `simulation_config.json`, each run for 5 seeded trials
(seeds 42–46) via `metrics_analysis.py --runs 5` (45 total runs), with the
full per-step CSV for each individual run also saved via
`run_experiments.py`:

1. `baseline` — no perception error, no fusion
2. `false_positive` — phantom detections only (rate 0.08)
3. `false_negative` — missed detections only (rate 0.25)
4. `sensor_noise` — Gaussian position noise (std 1.5)
5. `latency` — detections delayed 5 steps
6. `sensor_dropout` — periodic blackouts (prob 0.02, duration 8 steps)
7. `confidence_error` — miscalibrated confidence (level 0.35), no fusion
8. `naive_fusion` — combined error profile (20% false negative, noise 1.5,
   confidence error 0.15), unweighted fusion
9. `trust_weighted_fusion` — same combined error profile as naive_fusion,
   confidence-weighted fusion

**Key results so far:**
- Baseline: 5/5 success, zero collisions.
- Sensor noise: 0/5 success — worst scenario (avg 20.8 hard collisions/run).
- False negative: 4/5 success, avg 17.0 missed-response events/run.
- Latency: 5/5 success, deterministic across seeds.
- Sensor dropout: 4/5 success, avg 10.8 missed-response events/run.
- Confidence error (no fusion): 5/5 success, no measurable behavioral effect.
- Naive fusion: 4/5 success, avg 0.4 hard collisions/run.
- Trust-weighted fusion: **0/5 success**, avg 3.6 hard collisions/run —
  consistently worse than naive fusion on every seed. Likely cause:
  confidence-weighting amplifies confidently-wrong detections when reported
  confidence is itself miscalibrated. Flagged as an open research question,
  not yet resolved.

## 3. Plots generated (`plots/`)

- `baseline_vs_error_scenarios.png`
- `dropout_vs_mission_success.png`
- `false_negative_vs_collision_risk.png`
- `false_positive_vs_unnecessary_avoidance.png`
- `fusion_mode_vs_safety_metrics.png`
- `latency_vs_response_time.png`

All generated from `results/results_summary.csv` via `generate_plots.py`.

## 4. Videos / images saved (`media/`)

One MP4 per scenario, plus a side-by-side fusion comparison video, generated
via `simulation_visualizer.py` (batch mode):

- `baseline_video.mp4`
- `confidence_error_video.mp4`
- `false_negative_video.mp4`
- `false_positive_video.mp4`
- `latency_video.mp4`
- `naive_fusion_video.mp4`
- `no_fusion_matched_video.mp4`
- `sensor_dropout_video.mp4`
- `sensor_noise_video.mp4`
- `trust_weighted_fusion_video.mp4`
- `fusion_comparison_video.mp4` — synced side-by-side panels of
  `naive_fusion`, `trust_weighted_fusion`, and `no_fusion_matched`

Each video draws UAV positions/trails, goal markers, ground-truth vs.
perceived obstacle position, collision-risk zones, and a live
SUCCESS/FAILURE/In Progress mission-status readout, straight from the CSV
log columns.

## 5. Metrics currently calculated

Per-step (logged per UAV per step in every `logs/*.csv`):
`uav_pos_x/y`, `goal_pos_x/y`, `actual_obstacle_x/y`,
`perceived_obstacle_x/y`, `perception_error_type`, `confidence_value`,
`fusion_mode`, `action_taken`, `num_perceived_detections`,
`num_phantom_detections`, `dist_to_goal`, `distance_to_nearest_uav`,
`distance_to_obstacle`, `nearest_entity_type`, `nearest_entity_distance`,
`collision_risk_flag`, `unnecessary_avoidance_flag`, `missed_response_flag`,
`mission_completed_flag`, `reached_goal`.

Per-run aggregate (in `results/results_summary.csv`, one row per
scenario/run):
`scenario`, `run_number`, `fusion_mode`, `false_positive_rate`,
`false_negative_rate`, `noise_level`, `latency_steps`,
`dropout_probability`, `confidence_error_level`, `collision_risk_count`,
`unnecessary_avoidance_count`, `missed_response_count`,
`fusion_recovery_count`, `mission_success`, `avg_response_time_s`,
`total_near_misses`, `avg_formation_error`, average confidence error.

Printed-to-stdout per scenario: `steps_run`, `uavs_reached_goal`/`num_uavs`,
`mission_success`, `collision_count`, `near_miss_count`,
`unnecessary_avoidance_count`, `missed_response_count`,
`avoidance_action_count`, `fusion_recovery_count`, `avg_response_time_s`,
`avg_formation_error`.

## 6. Current limitations

- Only 5 seeds per scenario — the naive vs. trust-weighted fusion gap is
  directionally consistent but would benefit from a larger sample (10+ runs)
  for tighter confidence intervals.
- Sensor dropout has no memory of the last known obstacle position once a
  blackout starts (no decaying-confidence carry-forward).
- Sensor noise has no filtering — each noisy reading is acted on directly;
  this is currently the single biggest safety gap (0/5 mission success).
- The trust-weighted fusion collision result is unresolved: unclear whether
  the confidence-weighting formula itself needs to discount unreliable
  confidence signals, or whether this is the expected failure mode of
  trust-based fusion under confidence miscalibration.
- Live mode runs one scenario window at a time (sequential, not
  side-by-side); a live multi-scenario comparison view like the offline
  `fusion_comparison_video.mp4` doesn't exist yet.
- Perception model is a simplified per-UAV "detection within sensor range"
  abstraction (position + confidence + error flags) rather than a modeled
  sensing process — this is exactly what the radar-like version is meant to
  replace.

## 7. What will be added in the radar-like version

- A radar-like sensing model in place of the current abstract
  within-range/confidence-value detection: field-of-view/beam constraints,
  range-dependent detection probability, and range/bearing-dependent noise,
  rather than a flat per-scenario error rate applied uniformly regardless of
  distance or geometry.
- Likely carries forward the existing error taxonomy (false positive/
  negative, noise, latency, dropout, confidence miscalibration) but derives
  them from the radar model's physics instead of injecting them directly.
- Filtering on the perceived obstacle position (moving average or simple
  Kalman filter) to address the sensor_noise collision gap.
- Resolution of the trust-weighted fusion investigation — either a corrected
  weighting formula or documented confirmation of the failure mode.
- Increased seed count for the scenarios with run-to-run variance
  (sensor_noise, sensor_dropout, fusion comparison).
- Possible live side-by-side comparison view (mirroring the offline
  `fusion_comparison_video.mp4`) once more than one scenario needs watching
  at once during a run.

---

## 8. How to run

### Replay mode (existing — visualize a finished CSV log after the fact)

Run the simulation first (writes `logs/<scenario>_run<n>.csv` or
`logs/simulation_log.csv`), then visualize:

    python run_experiments.py --runs 3
    python simulation_visualizer.py

Or replay a single existing log:

    python simulation_visualizer.py --log logs/baseline_run1.csv --mode interactive
    python simulation_visualizer.py --log logs/baseline_run1.csv --mode mp4 --output media/baseline_video.mp4

### Live mode (new — watch the swarm fly while the simulation runs)

Driven entirely by `simulation_visualizer.py` — `simple_swarm_sim.py` isn't
touched or run separately for this:

    python simulation_visualizer.py --mode live --scenario baseline

- Opens a matplotlib window and actually computes+watches that scenario
  live, step by step — not a replay, no CSV needed first.
- Runs headless-safe: if no display is available, it still runs to
  completion (metrics printed), it just skips the window.
- Mission status reads **In Progress** throughout the run, then resolves to
  **SUCCESS** or **FAILURE** once the run ends, held on screen for
  `--live-hold` seconds (default 2.0) before the window closes.
- Omit `--scenario` to run every scenario in `--config` live, one window at
  a time.
- Add `--out-log <path>` to also save the finished run's CSV once it
  completes (same format `simple_swarm_sim.py` would produce) — useful if
  you want to replay/export that exact live-watched run later. When
  `--scenario` is omitted, `--out-log` is treated as a directory and each
  scenario is saved as `<out-log>/<scenario>_live.csv`.

Example: watch every scenario live and save each one's CSV, holding each
final result for 3 seconds:

    python simulation_visualizer.py --mode live --out-log logs/ --live-hold 3
