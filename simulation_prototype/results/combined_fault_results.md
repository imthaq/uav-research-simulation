# Combined-fault scenario results

Controller: `5_dynamic_trust_handoff`, 2 seeds/scenario, classified against the same baseline run and thresholds as `swarm_failure_envelope.csv` (Task 18) - see `experiments/combined_fault_scenarios.py`.

| Scenario | Mission success | Collisions (mean) | Near-misses (mean) | Formation error (mean) | Classification |
|---|---|---|---|---|---|
| low_P_D + high_clutter | 0.00 | 0.00 | 18.5 | 5.58 | **MISSION FAILURE** |
| radar_dropout + communication_loss | 1.00 | 0.00 | 51.5 | 4.33 | **DEGRADED BUT FUNCTIONAL** |
| registration_error + overconfident_vision | 1.00 | 0.00 | 63.5 | 3.78 | **SAFE** |
| radar_ghost_returns + target_crossing | 0.00 | 0.00 | 0.5 | 13.03 | **MISSION FAILURE** |
| latency + rapidly_moving_obstacle | 0.00 | 19.00 | 74.5 | 3.18 | **SAFETY FAILURE** |
| sensor_failure + centralized_fusion_unavailable | 1.00 | 0.00 | 56.0 | 3.62 | **SAFE** |
| corrupted_confidence + packet_delay | 0.00 | 0.00 | 0.0 | 4.41 | **MISSION FAILURE** |
| two_faulty_UAV_perception_sources | 1.00 | 0.00 | 56.0 | 3.62 | **SAFE** |

Baseline reference: near-miss count 56.0, formation error 3.780 (the 1.5x-of-baseline thresholds Task 18's `classify()` uses).

