# Fusion Results: Comparing Fusion Approaches

Comparison uses the matched-parameter trio `no_fusion_matched`, `naive_fusion`, `trust_weighted_fusion` (N=20 each, same underlying perturbation: `false_negative_rate=0.2`, `noise_level=1.5`, `confidence_error_level=0.15`, identical seeds 42–61), which isolates fusion-mode effects from scenario differences.

## 1. Summary Table

| Fusion Mode | Formation Error (mean) | Collision Risk (mean) | Mission Success | Near Misses (mean) |
|---|---|---|---|---|
| `no_fusion_matched` | 4.464 m | 151.00 | **10%** | 72.80 |
| `naive_fusion` | 4.082 m | **84.70** | **65%** | 44.75 |
| `trust_weighted_fusion` | **4.037 m** | 89.05 | 45% | 44.15 |

## 2. Which fusion approach produced the lowest estimation error?

**`trust_weighted_fusion`** has the lowest mean formation (estimation) error, 4.037 m vs. 4.082 m for `naive_fusion` — a small edge. A paired t-test across matched seeds shows this difference is **not statistically significant** (t(19) = 1.50, p = 0.150). Both fusion modes cut formation error by roughly 8–10% relative to `no_fusion_matched` (4.464 m), and an ANOVA across all three modes confirms fusion mode has a significant overall effect on formation error (F(2,57) = 7.31, p = 1.5e-03), driven mainly by the fused-vs-unfused contrast rather than naive-vs-trust differences.

## 3. Which approach produced the lowest collision risk?

**`naive_fusion`** has the lowest mean collision-risk count (84.70) — slightly lower than `trust_weighted_fusion` (89.05). This difference is also **not statistically significant** (paired t(19) = −1.44, p = 0.166). Both dramatically outperform `no_fusion_matched` (151.0); an ANOVA confirms a highly significant overall fusion-mode effect on collision risk (F(2,57) = 59.0, p = 1.3e-14).

A chi-square test on mission success rate across the three modes is significant (χ² = 12.92, df = 2, p = 1.6e-03): `naive_fusion` achieved the **highest** mission success rate (65%), ahead of `trust_weighted_fusion` (45%) and far ahead of `no_fusion_matched` (10%).

## 4. Does dynamic trust outperform fixed/simpler trust weighting?

Using the `no_dynamic_trust` ablation (5-seed subsample, seeds 42–46), which substitutes a static `confidence_weighted_fusion` mode in place of the dynamic trust estimator for the `trust_weighted_fusion` scenario, and comparing to the same 5 seeds under the full dynamic-trust pipeline:

| Condition | Collision Risk (mean) | Formation Error (mean) |
|---|---|---|
| Full dynamic trust (`trust_weighted_fusion`, seeds 42–46) | 79.8 | 3.987 m |
| Dynamic trust removed (`no_dynamic_trust` ablation → confidence-weighted fallback) | 79.2 | 4.154 m |

**Dynamic trust does not show a clear advantage over the fixed/confidence-weighted alternative** in this sample: collision risk is essentially unchanged (79.2 vs. 79.8), and formation error is slightly *worse* without dynamic trust (4.154 m vs. 3.987 m) but the sample is small (n=5) and not independently significance-tested. Combined with the full-sample result above — where `trust_weighted_fusion` did not significantly beat `naive_fusion` on either metric — this is consistent with the project's earlier finding that **`trust_weighted_fusion` underperforms `naive_fusion` under confidence miscalibration** (see `/areas/uav-swarm-simulation.md`): the added complexity of dynamic trust estimation is not paying for itself in the current implementation, particularly when sensor confidence reporting is unreliable.

## 5. Interpretation

Across every metric tested, going from **no fusion to any fusion is the dominant, statistically robust effect** (large, significant improvements in formation error, collision risk, and mission success). The *choice* between naive averaging and trust-weighted fusion, however, produces only small, non-significant differences in this dataset (N=20 per arm) — `naive_fusion` edges out on collision risk and mission success, `trust_weighted_fusion` edges out marginally on formation error. This suggests the trust-estimation logic needs either more trials to detect a real effect, or a scenario where sensor trust genuinely diverges (e.g., one faulty sensor among many healthy ones) to demonstrate a clear advantage over simple averaging.
