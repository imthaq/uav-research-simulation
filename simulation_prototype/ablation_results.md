# Ablation Results

Eight ablations were run (5 seeds each, 42–46) by disabling one pipeline component at a time and re-running the `naive_fusion` and `trust_weighted_fusion` scenarios. Values shown are (Collision Risk mean / Formation Error mean / Mission Success %).

| Ablation | `naive_fusion` scenario | `trust_weighted_fusion` scenario | Fusion mode actually used (trust scenario) |
|---|---|---|---|
| Full pipeline (reference, 5-seed) | 79.20 / 4.154 / 80% | 79.80 / 3.987 / 0% | trust_weighted_fusion |
| `no_radar_tracking` | 79.20 / 4.154 / 80% | 79.80 / 3.987 / 0% | trust_weighted_fusion |
| `no_confidence` | 80.60 / 4.127 / 80% | **87.00 / 4.127 / 20%** | trust_weighted_fusion |
| `no_trust_weighting` | 79.20 / 4.154 / 80% | **79.20 / 4.154 / 80%** | naive_fusion (substituted) |
| `no_covariance` | 79.20 / 4.154 / 80% | 79.80 / 3.987 / 0% | trust_weighted_fusion |
| `no_latency` | 79.20 / 4.154 / 80% | 79.80 / 3.987 / 0% | trust_weighted_fusion |
| `no_stale_data` | 79.20 / 4.154 / 80% | 79.80 / 3.987 / 0% | trust_weighted_fusion |
| `no_communication` | **141.40 / 4.646 / 20%** | **141.40 / 4.646 / 20%** | no_fusion (substituted) |
| `no_dynamic_trust` | 79.20 / 4.154 / 80% | **79.20 / 4.154 / 80%** | confidence_weighted_fusion (substituted) |

## Findings by Ablation

- **`no_radar_tracking`, `no_covariance`, `no_latency`, `no_stale_data`**: produced numbers **identical** to the full-pipeline reference for both scenarios. These components were not exercised by the specific `naive_fusion`/`trust_weighted_fusion` scenario perturbation profile used in this run (which doesn't include extra latency, stale-data, or radar-tracking-specific stress), so these four ablations are effectively no-ops here rather than evidence the components don't matter. They would need to be re-run against scenarios that actually stress those components (e.g. `latency`, `sensor_dropout`) to be informative.
- **`no_confidence`** (removing confidence-error injection): `trust_weighted_fusion` collision risk *increased* slightly (87.0 vs 79.8) but mission success improved (20% vs 0%), and formation error became identical between naive and trust modes (4.127 both) — consistent with confidence miscalibration being a primary source of the trust-weighted mode's fragility (see `fusion_results.md`).
- **`no_trust_weighting`**: as expected, substitutes `naive_fusion` behavior in place of `trust_weighted_fusion`, producing identical numbers to the naive-fusion row. This is a sanity-check ablation confirming the trust-weighting code path is what differentiates the two modes when active.
- **`no_communication`**: the only ablation producing a **large, decisive** effect — both scenarios collapse to `no_fusion` behavior, collision risk nearly doubling (141.4 vs ~79.5) and formation error rising ~0.6 m. See `communication_results.md` for full discussion.
- **`no_dynamic_trust`**: substitutes a static `confidence_weighted_fusion` mode; results are identical to the `naive_fusion` row (79.2 / 4.154 / 80%), and slightly *better* than the full dynamic-trust pipeline (79.8 / 3.987 / 0%) on collision risk and mission success, though slightly worse on formation error. See `fusion_results.md` §4.

## Overall Takeaway

The most consequential ablation by far is **removing communication/fusion entirely** — this is the one manipulation that reliably and substantially degrades every metric. Ablations targeting the *sophistication* of fusion (confidence handling, dynamic trust) produce smaller, mixed-direction effects, reinforcing that in this dataset, having *some* fusion matters far more than which specific fusion algorithm is used. Several ablations (`no_radar_tracking`, `no_covariance`, `no_latency`, `no_stale_data`) need to be re-targeted at scenarios that actually stress the ablated component before they can be interpreted.
