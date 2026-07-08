# Research Tables

Reference tables summarizing the UAV swarm perception-error simulation prototype, for use in the writeup. All figures are pulled directly from `simulation_config.json` and `results/results_summary.csv` (10 scenarios x 5 seeded runs = 50 total runs, seeds 42-46). See `methodology_draft.md` and `initial_results_summary.md` for full narrative context.

## 1. Scenario settings

World: 100 x 100 units. Obstacle: circle at (50, 50), radius 5. Shared target: (90, 90) (each UAV gets its own goal slot on a small circle around it). Swarm: 4 UAVs, speed 2.0, `dt` 0.2s, max 600 steps / 120s. Sensor range 15, collision distance 1.5, near-miss distance 3.5, goal tolerance 2.0. Control gains: goal 1.0, avoidance 6.0, tangential 4.0.

| Scenario | Description | Fusion mode |
|---|---|---|
| baseline | No perception error, no fusion - reference case | no_fusion |
| false_positive | Phantom (ghost) detections only | no_fusion |
| false_negative | Missed detections only | no_fusion |
| sensor_noise | Gaussian position noise on real detections only | no_fusion |
| latency | Detections delayed before reaching the controller | no_fusion |
| sensor_dropout | Periodic total sensor blackouts | no_fusion |
| confidence_error | Miscalibrated reported confidence, no fusion to compensate | no_fusion |
| no_fusion_matched | Combined error profile (FN + noise + confidence error), fusion disabled - control for the fusion comparison | no_fusion |
| naive_fusion | Same combined error profile as no_fusion_matched, unweighted cross-UAV fusion | naive_fusion |
| trust_weighted_fusion | Same combined error profile as no_fusion_matched, confidence-weighted cross-UAV fusion | trust_weighted_fusion |

## 2. Input variables (parameter levels tested)

| Scenario | Parameter | Level |
|---|---|---|
| false_positive | `false_positive_rate` | 0.08 |
| false_negative | `false_negative_rate` | 0.25 |
| sensor_noise | `position_noise_std` | 1.5 |
| latency | `latency_steps` | 5 |
| sensor_dropout | `dropout_prob` / `dropout_duration_steps` | 0.02 / 8 |
| confidence_error | `confidence_error_level` | 0.35 |
| no_fusion_matched / naive_fusion / trust_weighted_fusion | `false_negative_rate` / `position_noise_std` / `confidence_error_level` | 0.2 / 1.5 / 0.15 (fusion_mode is the only variable that changes across these three) |

Only one level per parameter has been tested in the current prototype (no low/medium/high sweep yet).

## 3. Output metrics

| Metric | Meaning |
|---|---|
| `collision_risk_count` | Per-UAV-per-step count of steps where the nearest entity (obstacle or UAV) was within the near-miss threshold |
| `unnecessary_avoidance_count` | Avoidance triggered by a phantom-only detection set (no real detections that step) |
| `missed_response_count` | A real threat was within the near-miss threshold in ground truth but didn't appear in what the UAV perceived that step |
| `fusion_recovery_count` | A detection an individual UAV's own sensor missed, recovered via another UAV's fused detection |
| `mission_success` | Yes/No - every UAV reached its goal tolerance AND zero hard collisions occurred during the run |
| `avg_response_time_s` | Average delay (seconds) between a threat first becoming real and the UAV first perceiving it |
| `total_near_misses` | Per-pair (not per-UAV) count of UAV-obstacle/UAV-UAV distances at or below the near-miss threshold |
| `avg_formation_error` | RMSE of pairwise UAV-to-UAV distance vs. desired formation spacing |
| `avg_confidence_error` | Mean absolute gap between a detection's true and reported confidence |
| `collision_count` (hard collisions) | Raw count of collision-distance events; drives `mission_success` internally but is **not currently written** to `results_summary.csv` - see limitations in `methodology_draft.md` |

## 4. Results summary (averages across 5 seeded runs per scenario)

| Scenario | Mission success | Avg collision-risk count | Avg unnecessary avoidance | Avg missed response | Avg fusion recovery | Avg response time (s) | Avg near misses | Avg formation error | Avg confidence error |
|---|---|---|---|---|---|---|---|---|---|
| baseline | 5/5 | 79.0 | 0.0 | 0.0 | 0.0 | 0.000 | 47.0 | 3.682 | 0.000 |
| false_positive | 5/5 | 79.6 | 62.6 | 0.0 | 0.0 | 0.000 | 44.8 | 3.858 | 0.000 |
| false_negative | 4/5 | 89.6 | 0.0 | 17.0 | 0.0 | 0.095 | 47.8 | 4.003 | 0.000 |
| sensor_noise | 0/5 | 149.4 | 0.0 | 0.0 | 0.0 | 0.000 | 84.2 | 4.050 | 0.000 |
| latency | 5/5 | 96.0 | 0.0 | 0.0 | 0.0 | 1.000 | 54.0 | 4.059 | 0.000 |
| sensor_dropout | 4/5 | 90.4 | 0.0 | 10.8 | 0.0 | 0.078 | 52.2 | 4.008 | 0.000 |
| confidence_error | 5/5 | 79.0 | 0.0 | 0.0 | 0.0 | 0.000 | 47.0 | 3.682 | 0.243 |
| no_fusion_matched | 1/5 | 141.4 | 0.0 | 20.6 | 0.0 | 0.058 | 75.0 | 4.646 | 0.119 |
| naive_fusion | 4/5 | 79.2 | 0.0 | 7.4 | 78.2 | 0.055 | 43.6 | 4.154 | 0.121 |
| trust_weighted_fusion | 0/5 | 79.8 | 0.0 | 9.0 | 78.8 | 0.055 | 36.8 | 3.987 | 0.121 |

Hard collision counts (not in the CSV above; obtained by re-running each scenario directly against `simple_swarm_sim.py`'s `Simulation` class, see `methodology_draft.md` limitations):

| Scenario | Per-run hard collisions (seeds 42-46) | Average |
|---|---|---|
| baseline | 0, 0, 0, 0, 0 | 0.0 |
| false_positive | 0, 0, 0, 0, 0 | 0.0 |
| false_negative | 0, 0, 0, 0, 1 | 0.2 |
| sensor_noise | 24, 22, 30, 17, 11 | 20.8 |
| latency | 0, 0, 0, 0, 0 | 0.0 |
| sensor_dropout | 0, 3, 0, 0, 0 | 0.6 |
| confidence_error | 0, 0, 0, 0, 0 | 0.0 |
| no_fusion_matched | 10, 0, 13, 43, 63 | 25.8 |
| naive_fusion | 0, 0, 2, 0, 0 | 0.4 |
| trust_weighted_fusion | 1, 3, 2, 8, 4 | 3.6 |

## 5. Baseline vs. single-factor error comparison

Each row's values are the delta from baseline (baseline: collision-risk 79.0, hard collisions 0.0, missed response 0.0, unnecessary avoidance 0.0, mission success 5/5).

| Scenario | Δ collision-risk count | Δ hard collisions | Δ missed response | Δ unnecessary avoidance | Mission success |
|---|---|---|---|---|---|
| false_positive | +0.6 | 0.0 | 0.0 | +62.6 | 5/5 (no change) |
| false_negative | +10.6 | +0.2 | +17.0 | 0.0 | 4/5 (-1) |
| sensor_noise | +70.4 | +20.8 | 0.0 | 0.0 | 0/5 (-5, worst single-factor scenario) |
| latency | +17.0 | 0.0 | 0.0 | 0.0 | 5/5 (no change) |
| sensor_dropout | +11.4 | +0.6 | +10.8 | 0.0 | 4/5 (-1) |
| confidence_error | 0.0 | 0.0 | 0.0 | 0.0 | 5/5 (no change; error is present but behaviorally inert without fusion) |

## 6. Fusion comparison

All three rows share the identical combined error profile (FN 0.2, noise 1.5, confidence error 0.15); only `fusion_mode` changes.

| Scenario | Fusion mode | Mission success | Avg hard collisions | Avg missed response | Avg fusion recovery |
|---|---|---|---|---|---|
| no_fusion_matched | no_fusion | 1/5 | 25.8 | 20.6 | 0.0 |
| naive_fusion | naive_fusion | 4/5 | 0.4 | 7.4 | 78.2 |
| trust_weighted_fusion | trust_weighted_fusion | 0/5 | 3.6 | 9.0 | 78.8 |

Takeaway: both fusion modes sharply reduce missed response and hard collisions vs. no fusion at all, but trust-weighted fusion is worse than naive fusion on every one of the 5 seeds (likely because it amplifies confidently-wrong detections under confidence miscalibration) - see `initial_results_summary.md` for the full discussion.

## 7. Visual output / media list

| File | Type | Contents |
|---|---|---|
| `media/baseline_video.mp4` | MP4 replay | baseline scenario, run 1 |
| `media/false_positive_video.mp4` | MP4 replay | false_positive scenario, run 1 |
| `media/false_negative_video.mp4` | MP4 replay | false_negative scenario, run 1 |
| `media/sensor_noise_video.mp4` | MP4 replay | sensor_noise scenario, run 1 |
| `media/latency_video.mp4` | MP4 replay | latency scenario, run 1 |
| `media/sensor_dropout_video.mp4` | MP4 replay | sensor_dropout scenario, run 1 |
| `media/confidence_error_video.mp4` | MP4 replay | confidence_error scenario, run 1 |
| `media/no_fusion_matched_video.mp4` | MP4 replay | no_fusion_matched scenario, run 1 |
| `media/naive_fusion_video.mp4` | MP4 replay | naive_fusion scenario, run 1 |
| `media/trust_weighted_fusion_video.mp4` | MP4 replay | trust_weighted_fusion scenario, run 1 |
| `media/fusion_comparison_video.mp4` | MP4, 3-panel side-by-side | no_fusion_matched vs. naive_fusion vs. trust_weighted_fusion, synced frame-by-frame |
| `plots/baseline_vs_error_scenarios.png` | Static chart | Each single-factor scenario vs. baseline average |
| `plots/dropout_vs_mission_success.png` | Static chart | Sensor dropout's effect on mission success |
| `plots/false_negative_vs_collision_risk.png` | Static chart | False-negative rate's effect on collision-risk count |
| `plots/false_positive_vs_unnecessary_avoidance.png` | Static chart | False-positive rate's effect on unnecessary avoidance |
| `plots/fusion_mode_vs_safety_metrics.png` | Static chart | Fusion-mode comparison on safety metrics |
| `plots/latency_vs_response_time.png` | Static chart | Latency's effect on response time |

Generate/replay commands for the MP4s are documented in `simulation_readme.md` ("Visualizing runs"). Note the per-scenario videos are `run1` only; other seeded runs (`run2`-`run5`) can be replayed the same way with `--log logs/<scenario>_run<n>.csv` but are not pre-rendered to video.
