# Advanced Results Summary and Discussion

This document synthesizes the updated results across `tracking_results.md`, `fusion_results.md`, `communication_results.md`, `statistical_results.md`, `ablation_results.md`, and `stress_test_results.md`. All figures are recomputed directly from `results/results_summary.csv`, `results/scenario_summary.csv`, `results/ablation_results.csv`, and `results/statistical_analysis.json` (920 total runs across 46 scenarios, N=20 trials each, seeds 42–61; 5-seed ablations at seeds 42–46).

## 1. Which radar parameter most affected tracking?

**Sensor/measurement noise (`noise_level`)** is by a wide margin the most damaging single parameter — r = 0.954 with collision risk (pooled, p ≈ 1.4×10⁻¹³⁶) and the only single-factor perturbation that drives mission success to 0%. Missed detections (`false_negative_rate`, r=0.825) and packet/track dropout (`dropout_probability`, r=0.705) are secondary but still strongly damaging; confidence miscalibration correlates with collision risk (r=0.525) but not formation error, suggesting its harm is concentrated at the fusion/trust stage. False positives are the mildest failure mode (weak, slightly negative correlation) — see `tracking_results.md`.

## 2. Which fusion approach produced the lowest estimation error?

**`trust_weighted_fusion`** had the lowest mean formation error (4.037 m vs. 4.082 m for `naive_fusion`), but the difference is not statistically significant (paired t(19)=1.50, p=0.150). See `fusion_results.md` and `statistical_results.md`.

## 3. Which approach produced the lowest collision risk?

**`naive_fusion`** had the lowest mean collision risk (84.70 vs. 89.05 for `trust_weighted_fusion`), also not statistically significant (paired t(19)=−1.44, p=0.166), and `naive_fusion` had the highest mission success rate (65% vs. 45%). Both fusion modes are dramatically and significantly better than no fusion (151.0 collision risk, 10% success; ANOVA p<1.5×10⁻¹⁴).

## 4. Did dynamic trust outperform fixed trust?

**No clear advantage was found.** Substituting a static, confidence-weighted trust scheme for the dynamic trust estimator (`no_dynamic_trust` ablation) produced nearly identical collision risk (79.2 vs. 79.8) and slightly better mission success/formation-error trade-offs than the full dynamic pipeline. Combined with the naive-vs-trust-weighted comparison above, this reinforces a standing project finding: trust-weighted fusion underperforms naive fusion under confidence miscalibration, and the added complexity of dynamic trust estimation is not yet paying for itself. See `fusion_results.md` §4 and `ablation_results.md`.

## 5. Did communication uncertainty change fusion performance?

**Not demonstrably — because it was never actually varied.** All six communication-degradation scenarios (`perfect_communication` through `communication_outage`) produced byte-identical results to the base `trust_weighted_fusion` run; the underlying packet-loss/latency/range parameters were not wired into these scenario configurations in this experiment. The one comparison that *is* valid — the `no_communication` ablation, which forces a total fallback to local sensing — shows a large, genuine effect (collision risk nearly doubling to 141.4), but this speaks to fusion-vs-no-fusion, not to graded communication uncertainty. See `communication_results.md`.

## 6. Failure Conditions

Ranked by severity among scenarios with genuine perturbations:
- **Catastrophic (0% success):** `high_dropout` (70% packet loss) and `simultaneous_sensor_failures` (combined FP/FN/noise/dropout/confidence-error stress) — both push collision risk above 235.
- **Severe (10% success):** `wrong_trust_assignment` — inverted trust initialization does not fully converge within the run.
- **Moderate (40% success):** `rapidly_moving_obstacle` and `overconfident_faulty_sensor`.
- **Stable (100% success despite cost):** `high_latency` (worst formation error of all tested scenarios, 9.454 m, but avoidance logic still succeeds), `target_crossing`, `sudden_target_appearance`.

See `stress_test_results.md` for the full matrix.

## 7. Limitations

- **Sample size:** 20 trials per scenario (the experiment metadata itself flags 50–100 trials as "paper-ready"); several fusion-mode comparisons show real but non-significant descriptive differences that a larger sample might resolve either direction.
- **Ablation sample size:** ablations were only run at 5 seeds, further limiting statistical power for those comparisons.
- **Unimplemented perturbations:** three stress-test scenarios (`very_low_P_D`, `very_high_P_FA`, `high_clutter`) and all six communication-degradation scenarios produced results identical to baseline/reference, indicating their intended parameters were never actually injected into the trial configuration. Conclusions about detection-probability, false-alarm, clutter, and communication-uncertainty robustness are not supported by the current data and require a corrected scenario generator.
- **Ablation no-ops:** four ablations (`no_radar_tracking`, `no_covariance`, `no_latency`, `no_stale_data`) produced no change relative to the reference run because the scenario they were tested against doesn't exercise those components; they need to be re-targeted at scenarios that do.
- **Fusion algorithm comparison underpowered:** none of the naive-vs-trust-weighted contrasts reached significance at current sample sizes, so "which fusion approach is best" remains a descriptive rather than confirmed statistical finding.

## 8. Research Contribution

This work delivers an end-to-end, reproducible simulation pipeline for multi-UAV swarm perception and fusion research: a configurable radar sensing and degradation model (`radar_like_model.py`, `radar_track_model.py`), three interchangeable fusion strategies (naive, confidence-weighted, trust-weighted) integrated non-invasively into an existing swarm simulator, and a unified CSV/JSON logging and statistical-analysis pipeline (`metrics_analysis.py`, `statistical_analysis.py`) supporting scenario sweeps, ablations, and stress tests at scale (920 runs across 46 scenarios in this run alone).

Beyond the tooling, the analysis yields a substantive, somewhat counter-intuitive empirical result: **sophistication in fusion (dynamic trust weighting) does not automatically outperform a simple naive-averaging baseline**, and in several respects (mission success rate, collision risk) trails it in this dataset. This is a useful negative/nuanced finding for the broader swarm-robotics literature, which often assumes trust-based weighting is strictly beneficial — it highlights that trust estimation needs either better-calibrated confidence inputs or scenarios with genuine inter-sensor trust divergence (e.g., one persistently faulty sensor among healthy ones) to demonstrate its intended advantage, and it flags concrete next steps (fixing the unimplemented perturbation parameters, increasing trial counts) needed before stronger claims can be made.
