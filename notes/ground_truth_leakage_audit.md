# Ground-Truth Leakage Audit

Files reviewed: `simple_swarm_sim.py`, `radar_like_model.py`, `radar_track_model.py`, `fusion_model.py`, UAV decision logic, obstacle-avoidance logic.

## Ground truth generates simulated measurements
`Simulation._true_detections_for()` builds true-range detections from actual positions, which `sense(t)` feeds into `Perception.process()`.
`radar_like_model.py`'s `_patch_perception()` wraps that call to derive the noisy scan (P_D roll, FOV/range gate, range/bearing noise, clutter/false alarms) from the true snapshot.

## Ground truth is used for evaluation
`true_dets` in `simple_swarm_sim.py` is only used for `missed_response_count` and `_threat_first_true_step` timing metrics, never passed to `_steer`.
`fusion_model.py`'s `estimation_error_against_ground_truth()` is explicitly evaluation-only and is not called by `fuse_step`, `fuse_centralized`, `fuse_distributed`, or `build_fused_log`.

## Runtime controller uses only detected, tracked, or fused positions
`decide_move()` steers using `perceived` (delayed sensor/fused/injected output from `_get_delayed_perception`), never `true_dets_all`.
`run_radar_track_fusion_pipeline()`'s docstring states ground truth is never used for decisions, per the `sense()`/`decide_move()` split and `_inject_external_estimates()`.

## Fusion weights do not use true error
`_as_source()` in `fusion_model.py` computes reliability from track-row fields (confidence, covariance trace, measurement age, dropout state) and static sensor config only.
No ground-truth comparison appears anywhere in the source-weighting math for `fuse_centralized` or `fuse_distributed`.

## Dynamic trust does not directly use ground-truth error during runtime
`TrustTracker.update()`'s docstring confirms every signal (cluster residual, freshness, dropout history, confidence, covariance) is something a real UAV would already have on hand.
The `_agreement_score` residual is computed against cluster-mates' estimates, not against the true target position.