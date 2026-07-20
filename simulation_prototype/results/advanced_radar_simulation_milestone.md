# Advanced Radar-Like Multimodal Swarm Simulation — Milestone Freeze

**Project:** `simulation_prototype` (NCRA UAV Dependability Lab)
**Milestone status:** Complete — advanced radar/tracker/fusion/communication/trust pipeline validated and frozen
**Freeze date:** 2026-07-20

---

## 1. Final Source Files Completed

```
simple_swarm_sim.py                 Core kinematic swarm simulation (4-UAV, motion,
                                     obstacle/target avoidance, goal-seeking, formation keeping)
models/radar_like_model.py          Radar sensor model (P_D, P_FA, clutter, range/bearing
                                     noise, latency, dropout) — wraps Perception.process,
                                     Simulation._apply_fusion, Simulation.step
models/communication_model.py       CommunicationChannel model (packet loss, comm range,
                                     latency, staleness, confidence corruption)
models/vision_like_model.py         Comparison sensor stack (not part of final radar-centric package)
models/lidar_like_model.py          Comparison sensor stack (not part of final radar-centric package)
tracking/radar_track_model.py       Per-radar constant-velocity Kalman filter +
                                     Mahalanobis-gated nearest-neighbor association
fusion/fusion_model.py              Cross-UAV track-to-track fusion (6 modes) + TrustTracker
metrics_analysis.py                 Canonical metric definitions (SWARM/PERCEPTION/COMMUNICATION fields)
build_experiment_matrix.py          Scenario/architecture/trial-count matrix builder
experiments/run_experiments.py      Run/aggregate/save machinery
experiments/ablation_experiments.py Ablation sweeps
experiments/statistical_analysis.py Welch's-t-test statistical comparison machinery
run_final_simulations.py            Final reproducible Monte Carlo run (two-tier trial plan)
generate_final_result_package.py    Builds results/final/ package + result_sanity_check.md
generate_plots.py                   Static analysis plots
simulation_visualizer.py            Animated per-run visualizer
radar_model_validation.py           Standalone radar correctness/consistency check
tracker_validation.py               Standalone tracker correctness/consistency check
fusion_validation.py                Standalone fusion correctness/consistency check
metrics_validation.py               Standalone metrics correctness/consistency check
communication_model_validation.py   Standalone communication-model correctness check
dynamic_trust_validation.py         Standalone TrustTracker correctness check
simulation_config.json              Single source of truth for world/swarm/timing/scenario/
                                     sensor/communication parameters
```

Each pipeline stage is layered as a non-invasive wrapper (monkey-patched) around
`simple_swarm_sim.py` rather than editing it directly: radar model → tracker → fusion →
communication model, in that order.

---

## 2. Radar Parameters Implemented

- `radar_max_range` / `radar_min_range` — range gating
- `radar_field_of_view` — angular sector around goal-heading direction
- `radar_update_rate` — scan frequency (Hz); held/re-served between scans
- `radar_latency_steps` — delay beyond update rate before a scan reaches the controller
- `radar_dropout_probability` — per-scan chance of total radar blackout
- `radar_detection_probability` (P_D) — independent Bernoulli gate on real detections
- `radar_false_alarm_probability` (P_FA) — probability a clutter candidate is reported
- `radar_clutter_density` — Poisson rate for clutter candidates per scan
- `radar_range_noise_m` / `radar_bearing_noise_deg` / radial-velocity noise — per-channel
  measurement uncertainty, carried as an explicit 3×3 measurement covariance
- `radar_confidence_error` — Gaussian miscalibration of self-reported confidence

Baseline ("clean sensor") reference: 15-unit max range, 360° FOV, 5 Hz update rate, P_D=1.0,
P_FA=0.0, 0.3 m range-noise std, 0.1 unit/s radial-velocity-noise std, zero latency/dropout/
confidence error.

---

## 3. Sensor Models Implemented

- **Radar-like model** (final/primary): full measurement row per UAV per step — true and
  measured range/bearing/radial velocity, detected x/y, confidence, status flags
  (`false_alarm_flag`, `missed_detection_flag`, `clutter_flag`, `dropout_flag`,
  `radar_pd_miss_flag`)
- **Vision-like and lidar-like models**: exist as comparison sensor stacks in the codebase
  but are outside the radar-centric scenario matrix and final result package

---

## 4. Tracker Implemented

Per-radar (per-UAV) **constant-velocity Kalman filter**:
1. Predict step for every existing track
2. Mahalanobis-gated nearest-neighbor matching (gate = `GATE_CHI2`, tied to the track's own
   innovation covariance rather than a fixed Euclidean radius)
3. Kalman update on matched tracks; `missed_count` reset, `existence_probability` raised
4. Unmatched tracks coast at predicted state; `missed_count` increments,
   `existence_probability` decays; enough misses moves a track to `lost` → `deleted`
5. Unmatched detections spawn `tentative` tracks, confirmed after `CONFIRM_HITS` consecutive
   matches

Each track carries: filtered position/velocity, full 4×4 state covariance, confidence, age,
hit/miss counts, existence probability, and status (`tentative`/`confirmed`/`coasting`/
`lost`/`deleted`).

---

## 5. Data-Association Method

**Greedy nearest-neighbor with Mahalanobis gating**: candidate (track, detection) pairs are
gated by Mahalanobis distance-squared against `GATE_CHI2 = 9.21` (~99% confidence region,
2 DOF) using each track's own innovation covariance, then resolved nearest-first so each
track matches at most one detection and vice versa per step. Not a globally optimal
assignment — a known limitation in dense/crossing-target scenarios (see §11).

---

## 6. Fusion Modes Implemented

| Mode | Weighting |
|---|---|
| `no_fusion` | Each UAV's own track stands alone |
| `naive_fusion` | Unweighted average across UAVs agreeing on the same object |
| `confidence_weighted_fusion` | Weight = track's own reported confidence |
| `trust_weighted_fusion` | Weight = confidence × status reliability × composite reliability (age, latency, dropout, optional dynamic trust) |
| `covariance_weighted_fusion` | Information-filter (inverse-covariance) fusion; optimal only under independent source errors |
| `covariance_intersection_fusion` | Covariance Intersection; stays consistent under unknown cross-source error correlation |

Fusion never reads ground truth — all weighting derives from confidence, status, covariance,
staleness, and static config-known sensor characteristics.

---

## 7. Centralized / Distributed Modes

- **Centralized** — every UAV's track goes to one central node, which clusters + fuses once
  and broadcasts the single result back out (uplink-then-downlink round trip)
- **Distributed** — no central node; each UAV broadcasts a lightweight summary to peers and
  fuses locally over its own track plus whatever peer summaries arrived that step, so
  different UAVs can end up with slightly different local estimates

---

## 8. Communication Faults Implemented

`CommunicationChannel` (used by the distributed architecture in place of a flat
drop-probability scalar):
- `packet_loss_probability` — per-message chance of outright loss
- `comm_range` — max distance a message can travel (`None` = unlimited)
- `base_latency_steps` — fixed delay on top of the sender's own sensor latency
- `max_staleness_steps` — rejects messages older than this regardless of delivery
- `corruption_probability` — chance the confidence/reliability value arrives scaled by a
  random corruption factor (bit-error proxy)

Six dedicated scenarios sweep this model: `perfect_communication`, `low_packet_loss`,
`high_packet_loss`, `short_communication_range`, `delayed_track_sharing`,
`communication_outage`.

---

## 9. Trust Model Implemented

`TrustTracker` (opt-in via `trust_adaptation`) adds a slow-moving `persistent_trust` score per
UAV/radar that accumulates across steps within a run:
- **Decreases** on repeated disagreement with other UAVs tracking the same object, being the
  uncorroborated odd one out, climbing measurement age, or frequent recent dropout
- **Recovers**, more slowly than it decays, on renewed cluster agreement, tight/tightening
  covariance with high confidence, and fresh non-dropped data

Purpose-built as a fault-injection contrast against `faulty_sensor_*` scenarios (one UAV's
radar reports the obstacle 5 world units off true position at a forced 0.97 confidence),
comparing naive, confidence-weighted, fixed-trust, and dynamic-trust fusion against it.

---

## 10. Scenarios Completed

**46 scenarios total**, defined in `simulation_config.json`:
- Reference / single-factor perception errors (7): `baseline`, `false_positive`,
  `false_negative`, `sensor_noise`, `latency`, `sensor_dropout`, `confidence_error`
- Fusion-mode comparison (3): `no_fusion_matched`, `naive_fusion`, `trust_weighted_fusion`
- Multi-entity spatial scenarios (8): `one_uav_obstacle`, `multiple_obstacles`,
  `two_crossing_targets`, `moving_obstacle_approaching_swarm`, `target_temporarily_lost`,
  `target_reappearing_after_dropout`, `clutter_near_real_target`, `closely_spaced_targets`
- Environment presets (6): `env_clear`, `env_low_visibility`, `env_fog`,
  `env_heavy_clutter`, `env_communication_delay`, `env_partial_sensor_failure`
- Communication-channel sweeps (6): `perfect_communication`, `low_packet_loss`,
  `high_packet_loss`, `short_communication_range`, `delayed_track_sharing`,
  `communication_outage`
- Faulty/overconfident sensor vs. fusion mode (5): `faulty_sensor_naive_fusion`,
  `faulty_sensor_confidence_weighted_fusion`,
  `faulty_sensor_trust_weighted_fusion_fixed`,
  `faulty_sensor_trust_weighted_fusion_dynamic`,
  `faulty_sensor_covariance_weighted_fusion`
- Radar stress tests (6): `very_low_P_D`, `very_high_P_FA`, `high_clutter`, `high_latency`,
  `high_dropout`, `simultaneous_sensor_failures`
- Miscellaneous stress/trust cases (5): `target_crossing`, `sudden_target_appearance`,
  `rapidly_moving_obstacle`, `overconfident_faulty_sensor`, `wrong_trust_assignment`

---

## 11. Monte Carlo Runs Completed

- **920 total runs** = 46 scenarios × 20 trials (`CORE_TRIALS`), generated
  2026-07-20T07:16:04Z
- Fusion-comparison scenarios are bumped to 50 trials when a time-budget check permits;
  otherwise capped back to 20 with the fallback recorded in run metadata
- World: 100×100, 4 UAVs starting clustered at (5,5)–(15,15), rally target at (90,90),
  static obstacle at (50,50) radius 5
- Time base: `dt = 0.2 s`, `max_steps = 600` (120 s simulated per trial), fusion update
  rate 5 Hz, master seed 42 with per-trial derived seeds

---

## 12. Plots Generated

- `plots/final/` — **13 PNGs**: fusion mode vs. mission success / collision risk / position
  RMSE; P_D vs. collision risk / missed response; P_FA vs. false track count; radar range vs.
  position RMSE; comms latency vs. response time; packet loss vs. mission success; clutter
  vs. fusion error; centralized vs. distributed; static vs. dynamic trust; 95% CI major
  results
- `plots/advanced/` — **15 PNGs**: the same core set plus SNR-vs-measurement-error,
  tracking-method comparison, trust-error-vs-collision-risk, and P_D-vs-track-continuity

---

## 13. Videos Generated

**19 MP4s** across four folders in `media/`:
- `media/fusion/` (5): fusion comparison, centralized-vs-distributed, no-fusion-matched,
  trust-weighted, naive-fusion
- `media/stress_tests/` (9): target crossing, sensor dropout, latency, overconfident faulty
  sensor, communication outage, clutter stress test, sensor dropout recovery, confidence
  error, sensor noise
- `media/tracking/` (1): Kalman tracking example
- `media/basic/` (4): dynamic trust adaptation, false positive, baseline, false negative

---

## 14. Statistical Tests Completed

- Framework: **Welch's t-test with normal-approximation p-values**, comparing baseline
  against every other scenario, plus dedicated fusion-mode and faulty-sensor-fusion-mode
  group comparisons (`statistical_comparisons.csv`)
- QA status (`result_sanity_check.md`, 17/18 checks passed): every p-value produced falls
  validly in [0, 1], but the most recent run recorded **0 pairwise comparisons actually
  produced** against an expectation of at least one — this needs root-causing before
  significance claims from the current package can be trusted
- 95% confidence intervals (normal approximation, 1.96·sd/√n) reported per scenario for the
  headline metrics (mission success rate, collision risk, response time, position RMSE,
  track continuity, communication load)

---

## 15. Current Limitations

- **Statistical comparisons not currently produced** — `statistical_comparisons.csv`
  generation is failing silently (0 comparisons) despite baseline being present; root cause
  not yet identified
- **Possible unwired stress/communication parameters** — an earlier QA pass found that
  `very_low_P_D`, `very_high_P_FA`, `high_clutter`, and all six communication-degradation
  scenarios did not vary their intended parameter from baseline in previously generated
  data; whether this has been corrected in the current package needs explicit re-verification
  (e.g. spot-checking `raw_run_index.csv` scenario-parameter columns against
  `simulation_config.json`)
- **Small-N significance** — fusion-comparison scenarios only guaranteed 50 trials when
  runtime permits, otherwise 20; prior analysis at N=20 found naive-vs-trust-weighted-fusion
  differences did not reach significance
- **Fusion independence assumption** — `confidence_weighted`, `trust_weighted`, and
  `covariance_weighted` fusion are statistically optimal only under independent source
  errors; `covariance_intersection_fusion` is the only mode designed to stay consistent
  under unknown cross-source correlation
- **Dynamic trust is opt-in and slow-adapting by design** — disabled unless a scenario
  explicitly sets `trust_adaptation`; its disagreement-detection is an indirect
  uncorroborated-source proxy, not ground-truth-aware
- **Data association is greedy nearest-neighbor, not globally optimal** — can make a locally
  reasonable but globally suboptimal match in dense/crossing-target scenarios; only
  considers a single hypothesis per detection per step; no explicit clutter-density
  feedback into gate size
- **No vision/lidar results in the final package** — `vision_like_model.py` and
  `lidar_like_model.py` exist but sit outside the radar-centric scenario matrix
- **Numeric result tables not bundled with the code/docs** — `results/final/*.csv` files
  live on the original machine and were not part of this archive; regenerate via
  `generate_final_result_package.py` to get current numbers

---

## 16. New Dependability Upgrades Planned

1. **JPDA (Joint Probabilistic Data Association)** — update multiple in-gate tracks in
   proportion to association probability instead of a single greedy winner-take-all match;
   targeted at the crossing-target and closely-spaced-target scenarios where nearest-neighbor
   is weakest
2. **RFS (Random Finite Set) tracking, e.g. PHD/CPHD filter** — model the full multi-target
   state as one set-valued distribution, natively handling clutter, missed detections, and an
   unknown/time-varying target count without explicit track initiation/deletion heuristics
   (heavier computational/implementation lift than JPDA; flagged as a further-out upgrade)
3. **Root-cause and fix `statistical_comparisons.csv` generation** so significance testing is
   actually produced against baseline and group comparisons
4. **Verify/repair scenario-parameter wiring** for `very_low_P_D`, `very_high_P_FA`,
   `high_clutter`, and the six communication-degradation scenarios so they genuinely vary
   from baseline
5. **Increase trial counts** toward the "paper-ready" 50–100 range for fusion-mode
   comparisons to resolve currently non-significant descriptive differences