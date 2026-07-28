# Fusion Validation Results (Task 5)

Validated multi-source fusion math in `fusion/fusion_model.py`.

**Result: 29/29 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| identical_measurements | 6/6 |
| radar_more_accurate | 4/4 |
| vision_more_accurate | 4/4 |
| lidar_more_accurate | 1/1 |
| stale_sensor | 1/1 |
| stale_rejection | 1/1 |
| dropped_sensor | 1/1 |
| missing_sensor_handling | 1/1 |
| high_conf_wrong_sensor | 1/1 |
| low_conf_correct | 1/1 |
| sensor_disagreement | 1/1 |
| covariance_intersection | 1/1 |
| correlated_estimates | 1/1 |
| centralized_fusion | 1/1 |
| distributed_fusion | 2/2 |
| delayed_track | 1/1 |
| trust_behavior | 1/1 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | identical_measurements | [naive_fusion] Fusing identical radar and vision returns exact pos | got (15.0000,25.0000) |
| PASS | identical_measurements | [confidence_weighted_fusion] Fusing identical radar and vision returns exact pos | got (15.0000,25.0000) |
| PASS | identical_measurements | [trust_weighted_fusion] Fusing identical radar and vision returns exact pos | got (15.0000,25.0000) |
| PASS | identical_measurements | [covariance_weighted_fusion] Fusing identical radar and vision returns exact pos | got (15.0000,25.0000) |
| PASS | identical_measurements | [covariance_intersection_fusion] Fusing identical radar and vision returns exact pos | got (15.0000,25.0000) |
| PASS | identical_measurements | Ground truth is not used in identical measurements test |  |
| PASS | radar_more_accurate | [confidence_weighted_fusion] Fused x favors accurate radar source | x=3.4483 |
| PASS | radar_more_accurate | [trust_weighted_fusion] Fused x favors accurate radar source | x=2.1692 |
| PASS | radar_more_accurate | [covariance_weighted_fusion] Fused x favors accurate radar source | x=0.1441 |
| PASS | radar_more_accurate | [covariance_intersection_fusion] Fused x favors accurate radar source | x=0.0001 |
| PASS | vision_more_accurate | [confidence_weighted_fusion] Fused x favors accurate vision source | x=7.0370 |
| PASS | vision_more_accurate | [trust_weighted_fusion] Fused x favors accurate vision source | x=8.4941 |
| PASS | vision_more_accurate | [covariance_weighted_fusion] Fused x favors accurate vision source | x=9.8844 |
| PASS | vision_more_accurate | [covariance_intersection_fusion] Fused x favors accurate vision source | x=9.9999 |
| PASS | lidar_more_accurate | Fused x favors tight-covariance LiDAR near target | x=0.0001 |
| PASS | stale_sensor | Fused position pulled toward fresh source, discarding stale | x=0.2297 |
| PASS | stale_rejection | Max staleness hard-rejects old data |  |
| PASS | dropped_sensor | Dropout state discounts reliability against active source | x=0.0552 |
| PASS | missing_sensor_handling | Missing sensor degrades to single active source seamlessly |  |
| PASS | high_conf_wrong_sensor | Clustering rejects far-off wrong sensor entirely | clusters=2 |
| PASS | low_conf_correct | Low conf correctly overshadowed by higher conf despite being 'true' | x=18.8235 |
| PASS | sensor_disagreement | Sensors disagreeing widely form separate clusters |  |
| PASS | covariance_intersection | CI handles correlated estimates without overclaiming precision | CI=8.0 Info=4.0 |
| PASS | correlated_estimates | CI remains bounded by individual source variance |  |
| PASS | centralized_fusion | Centralized creates exactly 1 shared world estimate with comm delays |  |
| PASS | distributed_fusion | Distributed uses available messages and creates one view per UAV |  |
| PASS | distributed_fusion | Distributed fusion uses ONLY available (undropped) messages | xs=[0.0, 2.0] |
| PASS | delayed_track | Delayed communicated track increases response time fields | resp=5 |
| PASS | trust_behavior | Persistent trust degrades influence of untrusted source | x=9.0909 |
