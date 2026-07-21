# Ground Truth Leakage Audit

Scope: `simple_swarm_sim.py`, `models/radar_like_model.py`,
`tracking/radar_track_model.py`, `fusion/fusion_model.py`,
`models/communication_model.py`, plus the swarm decision logic,
obstacle-avoidance logic, formation controller, and trust update logic
those files contain.

Method: every function that touches true state (`self.entities`,
`self.pos`, `true_dets`/`true_det`, `true_x/y`, `true_range`, etc.) or a
runtime-error quantity derived from it was traced to confirm *where its
output goes* — into a simulated measurement / evaluation log (expected),
or into a decision/weighting/trust computation (leakage).

## Checklist verdicts

- True states generate simulated sensor measurements — **PASS**
- True states are used for evaluation metrics only, never decisions — **PASS**
- UAV decisions use tracks or fused estimates, not ground truth — **PASS**
- Controller does not use true obstacle position — **PASS**
- Trust does not use true runtime measurement error — **PASS**
- Fusion weights do not select a sensor using ground truth — **PASS**
- Distributed UAVs use only locally available or communicated information — **PASS**

## Per-function results

### `simple_swarm_sim.py`

| Function | Verdict | Notes |
|---|---|---|
| `Perception.process` / `_apply_noise` / `_maybe_phantom` / `_confidence_for` | PASS | Consumes `true_detections` only to *corrupt* them (dropout, false negative, position noise, phantom injection, confidence miscalibration) into what the UAV perceives. This is the correct true→measurement direction. |
| `Simulation._true_detections_for` | PASS | Reads `self.entities`/`self.pos` (true state) to build the candidate detection list handed to `Perception.process`. Never called from steering/decision code directly. |
| `Simulation._advance_entities` / `_entity_active` / `_obstacle_view` | PASS | Pure true-world physics update (entity motion, appear/disappear). Not read by `_steer` or `decide_move`'s control logic. |
| `fuse_obstacle_detections` | PASS | Weights are each UAV's own reported `confidence`; combines `(x, y, confidence)` tuples that already came from `Perception.process`. No true position or true error term used. |
| `Simulation._apply_fusion` | PASS | Builds `contributions_by_id` strictly from `raw_percepts` (per-UAV perceived detections). Range-gates the fused result by the UAV's own sensor range, not by any ground-truth visibility check. |
| `Simulation._inject_external_estimates` | PASS | Consumes only the externally supplied `external_estimates` dict (fused track output from `fusion_model.py`), never `self.entities`. |
| `Simulation._steer` (obstacle-avoidance + UAV-UAV avoidance / formation logic) | PASS | Iterates `perceived` only — the post-perception, post-fusion, post-latency detection list. Both obstacle avoidance and inter-UAV avoidance (which is what produces formation-keeping behavior; there is no separate formation-controller function) key off `d["x"]`/`d["y"]`/`d["distance"]` from that list, never `self.entities` or another UAV's true `self.pos`. |
| `Simulation._get_delayed_perception` | PASS | Operates on the already-perceived/fused detection buffer; no true-state access. |
| `Simulation.sense` | PASS | Produces both `true_dets_all` (kept for logging/threat-timing metrics) and `raw_percepts` (perceived); does not feed `true_dets_all` into `_steer`. |
| `Simulation.decide_move` | PASS | Steering call `self._steer(i, perceived)` uses only the delayed/perceived list. `true_dets_all` in this function is used solely for: (a) `_threat_first_true_step` bookkeeping (a response-time *metric*), and (b) the `missed_response` flag (metric), never to alter `vx, vy`. |
| Collision/near-miss counting block in `decide_move` | PASS (evaluation) | Uses true `new_pos` vs. true `self.entities` positions to count collisions/near-misses. This is scoring the outcome of UAV decisions already made from perceived data, not an input to those decisions — correct use of ground truth for evaluation. |
| Formation-error RMSE (`decide_move`, using `formation_dists`) | PASS (evaluation) | Computed from UAVs' own true positions (`new_pos`), which is legitimate — a UAV's own position is not something it needs to "perceive," and this value is logged as a metric, not fed back into `_steer`. |
| `Simulation._log_row` | PASS (evaluation/logging) | Explicitly logs `actual_obstacle_x/y` alongside `perceived_obstacle_x/y` for diagnostic/plotting purposes; not consumed by any decision path. |
| `Simulation._metrics` | PASS (evaluation) | Aggregates counts/timers already computed above; no decision logic. |
| `run_radar_track_fusion_pipeline` | PASS | Decision call is `sim.decide_move(t, true_dets_all, raw_percepts, external_estimates=pending_estimates)`, where `pending_estimates` is the *previous step's* fused track estimate (`fused_by_uav`), never ground truth. All `true_range`/`true_target_x` fields computed in this function are written only into the returned CSV `rows` for logging. |

### `models/radar_like_model.py`

| Function | Verdict | Notes |
|---|---|---|
| `_range_bearing_radial`, `_snr_db_for_range`, `_quality_from_snr`, `_measurement_uncertainty` | PASS | These compute physics-correct, range-dependent sensor noise/PD/PFA. They are given `rng_`, which is the **true** range at the moment of measurement generation — this is the expected and required true→measurement direction (a real radar's SNR genuinely depends on true target range), not leakage into a decision. |
| `_apply_radar_noise` | PASS | Uses true `(base_x, base_y)` only to compute `true_range`/`true_bearing`, then overwrites `d["x"]/d["y"]` with the *noisy* range/bearing converted back to position. This noisy value is what `Simulation` subsequently reads for steering. |
| `_heading` | PASS | Uses the UAV's own goal slot (config-known, not sensed) as an orientation stand-in; not ground truth about another entity. |
| `_radar_dropout_fires`, `_apply_faulty_sensor`, `_apply_confidence_error` | PASS | Operate on the already-generated scan; faulty-sensor bias is a deliberately injected error, not a ground-truth read. |
| `_generate_clutter`, `_poisson_sample` | PASS | Clutter positions are drawn randomly within a configured annulus/FOV — no true target position involved at all. |
| `_patch_perception` (wrapped `Perception.process`) — P_D/FOV/min-max-range gating | PASS | The gate against `radar_max_range`/`radar_min_range`/FOV uses `true_range`/true bearing computed from the *matching true detection* purely to decide whether **this scan** contains/omits that detection (i.e., to generate the measurement stream correctly) — it does not hand true position to the UAV; the surviving detections still carry only noisy `x`/`y` after `_apply_radar_noise` runs on them. |
| `_patch_fusion` (wrapped `Simulation._apply_fusion`) | PASS | Only re-snapshots the (already ground-truth-free) post-fusion perceived list for logging; does not alter what fusion actually used. |
| `_patch_step` | PASS | Captures step index and UAV true positions purely to derive `observer_vel` (needed for Doppler/radial-velocity noise generation and for logging), not used by any decision code. |
| `_make_row`, `_finalize_step` | PASS (evaluation/logging) | Construct the CSV row: `true_target_x/y`, `true_range`, etc. are logged fields for offline evaluation, clearly separated from `measured_*`/`detected_x/y` fields that the UAV actually acts on. |

### `tracking/radar_track_model.py`

| Function | Verdict | Notes |
|---|---|---|
| `RadarTrack.__init__` / `.predict` / `._innovation` / `.mahalanobis_sq` / `.apply_match` / `.apply_miss` / `.as_row` | PASS | The Kalman filter operates purely on `det_x`/`det_y` (i.e., `detected_x`/`detected_y` from the radar model) passed in by the caller. No ground-truth field is read anywhere in this class. |
| `RadarTracker._match` / `.update` | PASS | Gating and association use only the filter's own innovation covariance and the detections passed in. |
| `build_tracks` | PASS | Explicitly filters to `row["detected_x"]`/`row["detected_y"]` (the measured fields) from `detection_rows`; `true_target_x/y` columns in those rows are never read here. |

### `fusion/fusion_model.py`

| Function | Verdict | Notes |
|---|---|---|
| `TrustTracker._agreement_score` | PASS | Compares a source's reported position only against its cluster-mates' reported positions (other sources' `x`/`y`, themselves derived from tracks/detections) — never against true target position. |
| `TrustTracker.update` | PASS | All five signals (`agreement`, `freshness` from `measurement_age_steps`, `dropout_score`, `confidence_score`, `covariance_score`) are derived from the track's own reported/self-assessed fields, not from any true-position residual or true detection/false-alarm label. |
| `_as_source` | PASS | Reliability composite uses `status`, `confidence`, `missed_count`, static config-known `sensor_latency_steps`/`sensor_dropout_probability`, and `persistent_trust` — no ground truth. |
| `_cluster` | PASS | Greedy spatial clustering on sources' own reported `x`/`y`. |
| `_weighted_average_xy`, `_information_fusion_xy`, `_covariance_intersection_pair`, `_covariance_intersection_xy`, `fuse_group` | PASS | All fusion-mode weighting uses only `confidence`, `status_weight`, `reliability`, and `covariance`/`eff_covariance`, all themselves ground-truth-free per `_as_source`. No fusion mode reads true position to pick or weight a source. |
| `_fuse_sources` | PASS | Delegates to `_cluster`/`fuse_group`; no additional ground-truth access. |
| `_sources_with_trust`, `_advance_trust` | PASS | Looks up `persistent_trust` from the (ground-truth-free) `TrustTracker`; advances it from this step's already-computed sources/clusters. |
| `fuse_centralized` | PASS | Comm/response-time bookkeeping only; fusion inputs are `radar_tracks` (already sensor-derived). |
| `fuse_distributed` | PASS | Each receiver fuses only its own track plus whatever peer tracks were actually delivered by `CommunicationChannel.transmit` (subject to packet loss / range / staleness) — this is the distributed-uses-only-locally-available-or-communicated-information requirement, directly verified. |
| `fuse_step`, `build_fused_log` | PASS | Orchestration only; no direct ground-truth access. |
| `estimation_error_against_ground_truth` | PASS (evaluation) | Explicitly documented and implemented as an evaluation-only helper — takes fused rows and a supplied ground-truth `(x, y)` to compute error metrics. Not called from any fusion/weighting/trust code path. |

### `models/communication_model.py`

| Function | Verdict | Notes |
|---|---|---|
| `CommunicationChannel.in_range` | PASS | Gates on `sender_pos`/`receiver_pos` as supplied by the caller — in `fuse_distributed` these are each UAV's own reported/track position, not true position (`fuse_distributed` doesn't even pass true positions in; range gating there is by track content, not called with sender/receiver positions at all in the current call site, and where it is exercised standalone/self-check it takes whatever positions the caller provides — the module never reaches into `self.entities`/`self.pos`). |
| `CommunicationChannel.is_stale`, `.transmit` | PASS | Packet loss, staleness, and confidence/reliability corruption are applied to the message dict only; no ground-truth read anywhere in the module. |
| `from_config` | PASS | Config parsing only. |

## Summary

No ground-truth leakage was found. True state (`self.entities`, `self.pos`,
`true_dets`, `true_range`, etc.) is used in exactly two legitimate ways
throughout the audited code:

1. **Measurement generation** — `_true_detections_for` → `Perception.process`
   → `radar_like_model`'s noise/PD/PFA pipeline, all of which consume true
   position/range only to *produce* a corrupted/noisy measurement, and then
   overwrite the detection's `x`/`y` with that noisy value before anything
   downstream reads it.
2. **Evaluation/logging** — collision/near-miss counts, formation-error
   RMSE, `_log_row`'s `actual_obstacle_x/y`, `_metrics`, and
   `estimation_error_against_ground_truth`, all of which score outcomes
   after decisions were already made from perceived/fused data, and are
   never read back into `_steer`, fusion weighting, or trust scoring.

All decision-making (`Simulation._steer`, `decide_move`'s steering call),
fusion weighting (`fuse_group` and every fusion mode in `fusion_model.py`,
`fuse_obstacle_detections` in `simple_swarm_sim.py`), and trust update
logic (`TrustTracker.update`) trace back only to perceived detections,
radar tracks, fused estimates, and each source's own self-reported
confidence/status/covariance/dropout/latency — never to `self.entities`,
`self.pos`, or any other true-state field. The distributed fusion
architecture additionally confirmed to route exclusively through
`CommunicationChannel.transmit`, so a UAV only ever fuses its own track
plus whatever peer tracks were actually delivered to it.
