# Metrics Validation Results

Validates each metric calculation against a small, hand-computable example. The goal is to ensure a human can check the expected number on paper, ensuring the math is strictly correct rather than just that the simulation "looks reasonable".

Tests cover Tracking, Fusion, Swarm, and Communication metrics.

**Total checks:** 32  |  **PASS:** 32  |  **FAIL:** 0

| # | Category | Metric | Actual Result | Expected Result | Result |
|---|---|---|---|---|---|
| 1 | Tracking | position_RMSE | 3.5355 | 3.5355 | PASS |
| 2 | Tracking | velocity_RMSE | 2.0 | 2.0 | PASS |
| 3 | Tracking | missed_detections | 2 | 2 | PASS |
| 4 | Tracking | false_detections | 1 | 1 | PASS |
| 5 | Tracking | false_tracks | 1 | 1 | PASS |
| 6 | Tracking | missed_tracks | 1 | 1 | PASS |
| 7 | Tracking | track_continuity | 0.8 | 0.8 | PASS |
| 8 | Tracking | track_fragmentation | 2 | 2 | PASS |
| 9 | Tracking | association_errors | 1 | 1 | PASS |
| 10 | Tracking | track_lifetime | 33 | 33 | PASS |
| 11 | Fusion | fused_position_RMSE | 2.5819 | 2.5819 | PASS |
| 12 | Fusion | covariance_consistency | 3.0 | 3.0 | PASS |
| 13 | Fusion | sensor_contribution_primary | 0.8 | 0.8 | PASS |
| 14 | Fusion | stale_data_count | 2 | 2 | PASS |
| 15 | Fusion | faulty_sensor_influence | 4.5 | 4.5 | PASS |
| 16 | Swarm | collision_count | 1 | 1 | PASS |
| 17 | Swarm | near_miss_count | 1 | 1 | PASS |
| 18 | Swarm | collision_risk_count | 2 | 2 | PASS |
| 19 | Swarm | minimum_separation | 0.5 | 0.5 | PASS |
| 20 | Swarm | response_time | 1.2 | 1.2 | PASS |
| 21 | Swarm | mission_completion_time | 15.0 | 15.0 | PASS |
| 22 | Swarm | mission_success | False | False | PASS |
| 23 | Swarm | formation_error | 1.4142 | 1.4142 | PASS |
| 24 | Swarm | unnecessary_avoidance | 1 | 1 | PASS |
| 25 | Swarm | hold_duration | 15 | 15 | PASS |
| 26 | Communication | messages_sent | 100 | 100 | PASS |
| 27 | Communication | messages_received | 80 | 80 | PASS |
| 28 | Communication | messages_dropped | 20 | 20 | PASS |
| 29 | Communication | stale_messages | 5 | 5 | PASS |
| 30 | Communication | communication_load | 500.0 | 500.0 | PASS |
| 31 | Communication | outage_duration | 3 | 3 | PASS |
| 32 | Communication | recovery_time | 2 | 2 | PASS |
