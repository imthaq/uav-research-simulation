# Fusion Validation Results (Task 5)

Controlled checks of the multi-source weighting math in `fusion/fusion_model.py` (`_as_source`, `fuse_group`, `fuse_centralized`) - hand-built radar-track-shaped rows fed straight into fusion, not the full radar+tracker+fusion pipeline.

**Result: 38/38 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| identical_measurements | 11/11 |
| radar_more_accurate | 5/5 |
| vision_more_accurate | 4/4 |
| one_stale_measurement | 5/5 |
| high_confidence_incorrect | 3/3 |
| one_sensor_dropout | 5/5 |
| covariance_intersection_correlated | 5/5 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | identical_measurements | [naive_fusion] fusing two identical measurements returns that exact position | got (10.0000,20.0000) |
| PASS | identical_measurements | [naive_fusion] fusing identical sources reports both as contributors | got 2 |
| PASS | identical_measurements | [confidence_weighted_fusion] fusing two identical measurements returns that exact position | got (10.0000,20.0000) |
| PASS | identical_measurements | [confidence_weighted_fusion] fusing identical sources reports both as contributors | got 2 |
| PASS | identical_measurements | [trust_weighted_fusion] fusing two identical measurements returns that exact position | got (10.0000,20.0000) |
| PASS | identical_measurements | [trust_weighted_fusion] fusing identical sources reports both as contributors | got 2 |
| PASS | identical_measurements | [covariance_weighted_fusion] fusing two identical measurements returns that exact position | got (10.0000,20.0000) |
| PASS | identical_measurements | [covariance_weighted_fusion] fusing identical sources reports both as contributors | got 2 |
| PASS | identical_measurements | [covariance_intersection_fusion] fusing two identical measurements returns that exact position | got (10.0000,20.0000) |
| PASS | identical_measurements | [covariance_intersection_fusion] fusing identical sources reports both as contributors | got 2 |
| PASS | identical_measurements | agreeing sources raise fused confidence above either individual confidence | got 0.78 |
| PASS | radar_more_accurate | [confidence_weighted_fusion] fused x sits closer to the accurate (radar) source than the midpoint | got x=3.4483 |
| PASS | radar_more_accurate | [trust_weighted_fusion] fused x sits closer to the accurate (radar) source than the midpoint | got x=2.1692 |
| PASS | radar_more_accurate | [covariance_weighted_fusion] fused x sits closer to the tighter-covariance (radar) source | got x=0.1441 |
| PASS | radar_more_accurate | [covariance_intersection_fusion] fused x sits closer to the tighter-covariance (radar) source | got x=0.0001 |
| PASS | radar_more_accurate | naive_fusion ignores accuracy and lands at the unweighted midpoint | got x=5.0000 |
| PASS | vision_more_accurate | [confidence_weighted_fusion] fused x sits closer to the accurate (vision) source | got x=7.0370 |
| PASS | vision_more_accurate | [trust_weighted_fusion] fused x sits closer to the accurate (vision) source | got x=8.4941 |
| PASS | vision_more_accurate | [covariance_weighted_fusion] fused x sits closer to the accurate (vision) source | got x=9.8844 |
| PASS | vision_more_accurate | [covariance_intersection_fusion] fused x sits closer to the accurate (vision) source | got x=9.9999 |
| PASS | one_stale_measurement | a stale source's reliability is discounted below a fresh source's | fresh_reliability=0.9 stale_reliability=0.0529 |
| PASS | one_stale_measurement | [trust_weighted_fusion] fused position is pulled toward the fresh source, away from the stale one | got x=0.2297 |
| PASS | one_stale_measurement | [covariance_weighted_fusion] fused position is pulled toward the fresh source, away from the stale one | got x=0.5556 |
| PASS | one_stale_measurement | [covariance_intersection_fusion] fused position is pulled toward the fresh source, away from the stale one | got x=0.0006 |
| PASS | one_stale_measurement | max_staleness_steps hard-rejects a too-old source before fusing | got [(0.0, 1)] |
| PASS | high_confidence_incorrect | [confidence_weighted_fusion] a single high-confidence outlier can pull the fused estimate far off | got x=45.3425 |
| PASS | high_confidence_incorrect | trust_weighted_fusion is also pulled toward a high-confidence, high-status outlier | got x=57.7554 |
| PASS | high_confidence_incorrect | clustering upstream keeps the far outlier in its own cluster (never reaches fuse_group) | cluster sizes=[2, 1] |
| PASS | one_sensor_dropout | dropout_state is flagged for a lost/missed track | active=False dropped=True |
| PASS | one_sensor_dropout | a dropped-out source's reliability is discounted below an active source's | active=0.9 dropped=0.05 |
| PASS | one_sensor_dropout | [trust_weighted_fusion] fused position favors the active source over the dropped-out one | got x=0.0552 |
| PASS | one_sensor_dropout | [covariance_weighted_fusion] fused position favors the active source over the dropped-out one | got x=0.5263 |
| PASS | one_sensor_dropout | with only one source reporting (full dropout of the other), fusion returns it unchanged | got {'x': 0.0, 'y': 0.0, 'confidence': 0.9, 'num_sources': 1, 'source_ids': ['active'], 'position_variance': 2.0, 'avg_persistent_trust': 1.0} |
| PASS | covariance_intersection_correlated | CI on two equal-covariance sources lands at the midpoint (symmetric case) | got x=5.0000 |
| PASS | covariance_intersection_correlated | CI's fused position_variance does not overclaim precision the way naive information fusion does | CI_var=8.0 info_var=4.0 |
| PASS | covariance_intersection_correlated | CI's fused variance is no larger than either individual source's (effective) variance - still informative | CI_var=8.0 individual_trace=8.0 |
| PASS | covariance_intersection_correlated | naive information fusion (assumes independence) halves the trace for two equal sources | got 4.0 vs expected 4.0 |
| PASS | covariance_intersection_correlated | CI's fused variance for several identical, fully-correlated sources stays close to the single-source variance | got 8.0 |
