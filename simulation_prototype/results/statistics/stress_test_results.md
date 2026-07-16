# Stress Test and Failure Scenario Analysis

This document evaluates the multi-UAV fusion and control system under 11 designated stress scenarios (N=20 trials each, seeds 42–61). All values are taken directly from `results/scenario_summary.csv`.

## Stress Test Summary Matrix

| Scenario | Fusion Mode Used | Mission Success | Collision Risk (mean ± sd) | Formation Error (mean) | Notes |
|---|---|---|---|---|---|
| `very_low_P_D` | trust_weighted_fusion | 100% | 79.0 ± 0.0 | 3.682 m | **No perturbation applied** (identical to baseline) |
| `very_high_P_FA` | trust_weighted_fusion | 100% | 79.0 ± 0.0 | 3.682 m | **No perturbation applied** (identical to baseline) |
| `high_clutter` | trust_weighted_fusion | 100% | 79.0 ± 0.0 | 3.682 m | **No perturbation applied** (identical to baseline) |
| `high_latency` | trust_weighted_fusion | 100% | 72.0 ± 0.0 | 9.454 m | 20 latency steps injected; response time 4.0s; formation error worst of all scenarios |
| `high_dropout` | trust_weighted_fusion | **0%** | 236.5 ± 21.4 | 3.882 m | 70% dropout probability; heavy missed-response burden (mean 151.4/run) |
| `simultaneous_sensor_failures` | trust_weighted_fusion | **0%** | 236.0 ± 11.9 | 3.725 m | Combined FP/FN/noise/dropout/confidence-error stress; highest missed-response count (153.6/run) |
| `target_crossing` | trust_weighted_fusion | 100% | 75.2 ± 8.4 | 3.987 m | Fusion recovery events triggered (mean 6.6/run) |
| `sudden_target_appearance` | trust_weighted_fusion | 100% | 84.0 ± 10.6 | 4.096 m | Large unnecessary-avoidance count (218.5/run) from false positives during appearance |
| `rapidly_moving_obstacle` | trust_weighted_fusion | 40% | 104.9 ± 16.0 | 3.670 m | Missed responses (26.6/run), heavy reliance on fusion recovery (129.9/run) |
| `overconfident_faulty_sensor` | confidence_weighted_fusion | 40% | 95.7 ± 19.0 | 3.702 m | High unnecessary avoidance (66.6/run) from overconfident bad measurements |
| `wrong_trust_assignment` | trust_weighted_fusion | **10%** | 110.0 ± 19.6 | 3.771 m | Second-worst success rate; inverted trust assignment never fully recovers within run length |

## Data-Generation Limitation (Important)

`very_low_P_D`, `very_high_P_FA`, and `high_clutter` all produced results **byte-identical to `baseline`** (collision risk exactly 79.0 with zero standard deviation across all 20 trials, formation error exactly 3.682 m). Inspection of the underlying per-trial parameter columns confirms `false_positive_rate`, `false_negative_rate`, `noise_level`, `dropout_probability`, and `confidence_error_level` are all 0.0 in these three scenarios — i.e., the intended detection-probability, false-alarm, and clutter-density perturbations were **never actually applied**. These three rows should not be cited as evidence of robustness to low P_D, high P_FA, or clutter; they are effectively unperturbed baseline replicates, matching the same gap identified in `communication_results.md` for the communication-degradation scenarios.

## Genuine Failure Conditions

Of the eight scenarios with real perturbations, three drove mission success to a critical or catastrophic level:

- **`high_dropout` (0% success)**: 70% packet dropout overwhelms the fusion pipeline's ability to maintain shared tracks; missed-response count balloons to 151/run.
- **`simultaneous_sensor_failures` (0% success)**: combined multi-parameter degradation (FP 0.15, FN 0.3, noise 1.5, dropout 0.6, confidence error 0.4) is catastrophic even with fusion active — collision risk (236.0) matches `high_dropout`.
- **`wrong_trust_assignment` (10% success)**: an inverted trust initialization is not fully corrected within the run's duration, producing the second-worst outcome of the real perturbation scenarios.

Two moderate-severity conditions:
- **`rapidly_moving_obstacle` (40% success)**: fast-closing obstacles push collision risk to 104.9, above the `trust_weighted_fusion` reference (89.05, see `fusion_results.md`).
- **`overconfident_faulty_sensor` (40% success)**: a sensor reporting bad data with high confidence degrades performance, though the fusion pipeline (running in `confidence_weighted_fusion` mode here) contains it better than the dropout/simultaneous-failure cases.

## Interpretation

The genuinely-tested stress conditions show a clear severity ordering: **combined/simultaneous degradation and sustained high packet dropout are the most damaging failure modes** (0% success), followed by trust-initialization errors (10%) and fast-dynamics/overconfidence conditions (40%). Single, well-isolated stresses that the fusion and tracking logic can compensate for (`target_crossing`, `sudden_target_appearance`, `high_latency`) retain 100% mission success even though they impose measurable cost on formation error or response time. Before drawing conclusions about detection-probability, false-alarm, or clutter robustness specifically, the three no-op scenarios above need to be re-run with their intended parameters actually applied.
