# Statistical Analysis of Multi-UAV Swarm Fusion Performance

## 1. Experimental Design

Three fusion paradigms were compared using a matched-parameter scenario trio (`no_fusion_matched`, `naive_fusion`, `trust_weighted_fusion`), each run for N=20 trials (seeds 42–61) under identical perturbation (`false_negative_rate=0.2`, `noise_level=1.5`, `confidence_error_level=0.15`). Additional single-factor scenarios isolate individual radar/perception parameters. All statistics below are computed directly from `results/results_summary.csv` and `results/scenario_summary.csv`.

## 2. Descriptive Statistics (Matched Fusion-Mode Trio)

| Metric | `no_fusion_matched` | `naive_fusion` | `trust_weighted_fusion` |
|---|---|---|---|
| Collision Risk (mean) | 151.00 | **84.70** | 89.05 |
| Formation Error (mean) | 4.464 m | 4.082 m | **4.037 m** |
| Mission Success Rate | 10% | **65%** | 45% |
| Near Misses (mean) | 72.80 | 44.75 | 44.15 |

## 3. One-Way ANOVA Across Fusion Modes

$$H_0: \mu_{\text{no fusion}} = \mu_{\text{naive}} = \mu_{\text{trust-weighted}}$$

- **Collision Risk Count**: F(2, 57) = 58.997, p = 1.31 × 10⁻¹⁴ (significant, p < 0.001)
- **Formation Error**: F(2, 57) = 7.305, p = 1.50 × 10⁻³ (significant, p < 0.01)

**Conclusion:** Fusion mode has a statistically significant effect on both collision risk and formation error. Post-hoc inspection (see §4) shows this effect is driven almost entirely by the fused-vs-unfused contrast, not by differences between the two fusion algorithms.

## 4. Paired Comparison: Naive vs. Trust-Weighted Fusion

Paired t-tests across matched trial seeds (naive vs. trust-weighted, N=20 pairs):

- **Collision Risk:** t(19) = −1.442, p = 0.166 (not significant) — naive mean 84.70 vs. trust-weighted mean 89.05
- **Formation Error:** t(19) = 1.500, p = 0.150 (not significant) — naive mean 4.082 m vs. trust-weighted mean 4.037 m

**Conclusion:** Unlike the fused-vs-unfused contrast, the difference between naive and trust-weighted fusion is **not statistically significant** on either metric at N=20. Descriptively, naive fusion has lower collision risk and trust-weighted fusion has lower formation error, but neither edge clears significance — consistent with the project's earlier finding that trust-weighted fusion underperforms naive fusion under confidence miscalibration (see `/areas/uav-swarm-simulation.md` and `fusion_results.md`).

## 5. Mission Success Rate: Chi-Square Test

| Mode | Successes | Failures |
|---|---|---|
| `no_fusion_matched` | 2 | 18 |
| `naive_fusion` | 13 | 7 |
| `trust_weighted_fusion` | 9 | 11 |

χ²(2, N=60) = 12.917, p = 1.57 × 10⁻³ (significant at α = 0.01)

**Conclusion:** Mission success rate differs significantly across fusion modes. `naive_fusion` achieves the highest observed success rate (65%), ahead of `trust_weighted_fusion` (45%) and far ahead of `no_fusion_matched` (10%).

## 6. Correlation Analysis: Radar/Perception Parameters vs. Swarm Outcomes

Pooled across `baseline` and the six single-factor perturbation scenarios plus the `env_*` combined scenarios:

| Parameter | r (Formation Error) | r (Collision Risk) | Significance |
|---|---|---|---|
| **noise_level** | +0.224 (p=2.7e-04) | **+0.954** (p=1.4e-136) | *** |
| false_negative_rate | +0.234 (p=1.4e-04) | +0.825 (p=5.2e-66) | *** |
| dropout_probability | +0.179 (p=3.8e-03) | +0.705 (p=1.9e-40) | *** |
| confidence_error_level | +0.056 (n.s.) | +0.525 (p=8.7e-20) | *** (collision risk only) |
| latency_steps | +0.317 (p=1.8e-07) | −0.120 (n.s.) | * (formation error only) |
| false_positive_rate | −0.180 (p=3.6e-03) | −0.185 (p=2.8e-03) | ** (small, negative) |

### Key Takeaways
1. **Sensor noise (`noise_level`) is the dominant driver of tracking degradation** — its correlation with collision risk (r=0.954) is far stronger than any other parameter, and it is the only single-factor perturbation that drives mission success to 0%.
2. **Missed detections and dropout are the next most damaging perception failures** (r=0.825 and r=0.705 with collision risk respectively) — both starve the tracker of usable measurements.
3. **Confidence miscalibration correlates with collision risk but not formation error**, indicating its damage is concentrated at the fusion/trust stage rather than raw tracking geometry.
4. **False positives are the mildest failure mode** and are weakly *negatively* correlated with both outcomes in this pooled sample — ghost detections mostly cause unnecessary avoidance maneuvers rather than true tracking loss.

## 7. Conclusion

The data support two robust, statistically significant conclusions: (1) **having any cross-UAV fusion is far better than none** (large, significant effects on collision risk, formation error, and mission success), and (2) **sensor noise is the single most damaging radar parameter** for tracking performance. The data do **not** support a strong, statistically significant claim that trust-weighted fusion outperforms naive fusion in its current form — the observed differences between the two algorithms are small and not significant at N=20 trials per condition.
