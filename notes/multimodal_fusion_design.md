# Multimodal Fusion Design

### What fusion combines
- `fusion_model.py` combines each UAV's own radar *track* (output of `radar_track_model.py`, itself built from `radar_like_model.py` detections) into one shared estimate per real-world object, per time step. Fusion never sees ground truth - only radar tracks/detections, matching the project's core rule that UAVs decide based on sensed, not true, positions. Vision-like and LiDAR-like models were left optional (Task 10, not implemented in this milestone), so today fusion only combines radar tracks across UAVs; `_generate_vision_lidar_detections()` in `simple_swarm_sim.py` is the extension point ready for them.

### The three fusion modes
- **no_fusion** - every UAV's track stands alone; nothing is combined across UAVs.
- **naive_fusion** - an unweighted average of the x/y estimates from every UAV's track that refers to the same object. Every UAV counts equally, confidence is ignored entirely.
- **trust_weighted_fusion** - a weighted average, where each UAV's contribution is weighted by `confidence * status_weight` (a confirmed track counts fully, a tentative one at 0.6x, since it hasn't proven itself over several hits yet).

### Deciding which tracks belong to the same object
- Before averaging, tracks from different UAVs have to be grouped by which real object they refer to. This is done with greedy single-linkage clustering by position (`CLUSTER_DISTANCE = 4.0`): any two tracks within that distance are put in the same cluster. This mirrors the same nearest-neighbor idea `radar_track_model.py` already uses one level down (detection-to-track), just applied one level up (track-to-track).

### Why confidence and status matter for trust-weighting
- A UAV's reported detection confidence already reflects range and (if `radar_confidence_error` is nonzero) sensor miscalibration - it's a proxy for "how much should this reading be trusted right now." Track status adds a second, independent signal: a `tentative` track hasn't been confirmed by several consecutive hits yet, so even a high-confidence tentative detection is discounted relative to a `confirmed` one. Combining both means trust-weighted fusion downweights exactly the readings that are least likely to be correct - phantom-adjacent detections, coasting tracks, and freshly spawned tracks - while naive fusion treats all of them the same as a solid, long-confirmed track.

### What "fusion error" measures
- `metrics_analysis.py` computes `fusion_error` as the average distance between each step's fused x/y estimate and the true object position, whenever fusion produced an estimate. This is the direct, per-run measure of how good a fusion mode's shared belief actually is - see `radar_results_discussion.md` for the naive vs. trust-weighted comparison.

### Design tradeoff
- Naive fusion is simpler and fully deterministic given the same tracks, but it lets one bad (noisy, phantom, coasting) track pull the fused estimate off just as much as a good one. Trust-weighted fusion needs each source to carry a meaningful confidence/status, but it's more robust to exactly the kind of degraded, partial radar picture this project's scenarios are built to stress-test (missed detections, clutter, dropout).
