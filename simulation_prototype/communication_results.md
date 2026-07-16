# Communication Results: Effect of Communication Uncertainty on Fusion Performance

## 1. Scenario Set

Six scenarios were run to probe communication-layer degradation: `perfect_communication`, `low_packet_loss`, `high_packet_loss`, `short_communication_range`, `delayed_track_sharing`, and `communication_outage` (all `trust_weighted_fusion`, N=20, seeds 42–61).

## 2. Finding: No Measurable Effect of Graded Communication Degradation

All six scenarios produced **byte-identical outcome statistics**:

| Scenario | Collision Risk (mean) | Formation Error (mean) |
|---|---|---|
| perfect_communication | 89.05 | 4.0366 |
| low_packet_loss | 89.05 | 4.0366 |
| high_packet_loss | 89.05 | 4.0366 |
| short_communication_range | 89.05 | 4.0366 |
| delayed_track_sharing | 89.05 | 4.0366 |
| communication_outage | 89.05 | 4.0366 |

These values are identical to the baseline `trust_weighted_fusion` scenario. Checking the underlying per-trial parameter columns (`false_positive_rate`, `false_negative_rate`, `noise_level`, `latency_steps`, `dropout_probability`, `confidence_error_level`) confirms that **none of these six scenarios actually varied a communication-specific parameter** from the base `trust_weighted_fusion` configuration — they used identical seeds and identical perturbation values throughout. This indicates the communication-uncertainty axis (packet loss rate, comm range, delay) was **not wired into the simulation's parameter set for this experiment run**, so the graded degradation these scenario names imply was never actually applied.

**Conclusion: communication uncertainty could not be shown to change fusion performance in this dataset, because it was never actually varied.** This is a data-generation gap, not evidence that communication quality is irrelevant to fusion, and it should not be reported as a robustness finding.

## 3. What the `no_communication` Ablation Does Show

The `no_communication` ablation, which forces every scenario to fall back to `no_fusion` (representing total loss of the inter-UAV channel), gives a genuinely different result:

| Condition | Collision Risk (mean, 5-seed) | Formation Error (mean, 5-seed) |
|---|---|---|
| Full communication (`trust_weighted_fusion`, matched seeds) | 79.8 | 3.987 m |
| `no_communication` ablation (forced `no_fusion` fallback) | 141.4 | 4.646 m |

This is a large, real effect: losing the ability to share tracks at all (collapsing to purely local sensing) nearly doubles collision risk and meaningfully worsens formation error. This confirms that **cooperative fusion itself (any fusion vs. none) is what protects the swarm** — consistent with `fusion_results.md` — but it speaks to an all-or-nothing loss of communication, not to graded uncertainty (packet loss %, latency, range) within a working link.

## 4. Recommendation

To actually answer "does communication uncertainty change fusion performance," the simulation's scenario generator needs to route `packet_loss_probability`, `comm_range`, and `comm_delay`-style parameters into the trial configuration for these six scenario names (they currently pass through unperturbed). Until that is fixed, only the binary fusion-vs-no-fusion comparison above is evidentially supported.
