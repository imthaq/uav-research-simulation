# Tracking Results: Radar Parameter Impact on Swarm Tracking Performance

This document analyzes which radar/perception parameters most affect tracking quality, using the single-factor perturbation scenarios (`baseline`, `false_positive`, `false_negative`, `sensor_noise`, `latency`, `sensor_dropout`, `confidence_error`) and the combined environmental scenarios (`env_clear` … `env_partial_sensor_failure`), all run in `no_fusion` mode with N=20 trials per scenario, seeds 42–61.

## 1. Single-Factor Perturbation Effects (vs. `baseline`)

| Parameter | Scenario | Formation Error (Δ vs baseline) | Collision Risk (Δ vs baseline) | Formation Error p | Collision Risk p |
|---|---|---|---|---|---|
| `noise_level` (1.5) | sensor_noise | 3.961 m (+0.279) | 146.5 (+67.5, **+85%**) | 6.9e-06 | 5.2e-23 |
| `latency_steps` (5) | latency | 4.059 m (+0.377) | 96.0 (+17.0) | <1e-16 | <1e-16 |
| `dropout_probability` (0.02) | sensor_dropout | 3.953 m (+0.271) | 87.8 (+8.8) | 4.1e-04 | 3.2e-04 |
| `false_negative_rate` (0.25) | false_negative | 3.945 m (+0.263) | 89.0 (+10.0) | 1.2e-06 | 1.9e-11 |
| `false_positive_rate` (0.08) | false_positive | 3.650 m (−0.032, n.s.) | 82.0 (+3.0) | 0.70 | 0.014 |
| `confidence_error_level` (0.35) | confidence_error | 3.682 m (+0.000) | 79.0 (+0.0) | n/a (no variance produced) |

## 2. Pooled Correlation Analysis (baseline + 6 perturbation scenarios + env_* scenarios)

| Parameter | r vs. Formation Error | r vs. Collision Risk |
|---|---|---|
| **noise_level** | +0.224 (p=2.7e-04) | **+0.954** (p=1.4e-136) |
| false_negative_rate | +0.234 (p=1.4e-04) | +0.825 (p=5.2e-66) |
| dropout_probability | +0.179 (p=3.8e-03) | +0.705 (p=1.9e-40) |
| confidence_error_level | +0.056 (n.s.) | +0.525 (p=8.7e-20) |
| latency_steps | +0.317 (p=1.8e-07) | −0.120 (n.s.) |
| false_positive_rate | −0.180 (p=3.6e-03) | −0.185 (p=2.8e-03) |

## 3. Key Finding

**`noise_level` (radar range/measurement noise) is the radar parameter with the single largest effect on tracking outcomes.** At `noise_level = 1.5`, the `sensor_noise` scenario is the only single-factor perturbation that drives mission success to **0%** (down from 100% at baseline), and it produces the strongest correlation with collision risk of any parameter tested (r = 0.954). Its effect on collision risk (+67.5, +85% over baseline) is roughly 4–7× larger than any other single perturbation.

Ranked by impact on tracking (collision risk correlation, then effect size):
1. **noise_level** — dominant driver of tracking failure
2. **false_negative_rate** — second-largest driver; missed detections directly translate into missed responses (avg. 16 missed responses per run)
3. **dropout_probability** — moderate but consistent degradation
4. **confidence_error_level** — moderately correlated with collision risk but *not* with formation error in the pooled sample, suggesting its damage is concentrated in fusion-stage trust miscalibration rather than raw tracking geometry (see `fusion_results.md`)
5. **latency_steps** — affects formation error and response time strongly, but not collision risk in this dataset (r = −0.120, n.s.)
6. **false_positive_rate** — smallest, and slightly *negative*, effect; false alarms mostly produce unnecessary avoidance maneuvers (66 per run in `false_positive`) rather than tracking-accuracy loss

## 4. Interpretation

Missed detections (false negatives), sensor noise, and dropout all degrade tracking by starving the estimator of usable measurements, which is consistent with a Kalman-style tracker whose covariance grows during coasting. False positives, in contrast, add spurious tracks that mostly trigger conservative avoidance behavior without corrupting the true-target estimate — a much milder failure mode. This ranking should guide where engineering effort on the perception stack (denoising, detection-probability improvements) yields the largest tracking-quality return.
