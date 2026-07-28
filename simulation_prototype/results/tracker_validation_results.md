# Tracker Validation Results (Task 4)

Deterministic checks of the Kalman-filter + gated nearest-neighbor tracker in `tracking/radar_track_model.py` - detections are fed straight into `RadarTracker`, not through the full radar model.

**Result: 40/40 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| constant_velocity_target | 4/4 |
| no_measurement_noise | 2/2 |
| prediction_correctness | 3/3 |
| low_measurement_noise | 3/3 |
| one_missed_detection | 7/7 |
| several_missed_detections | 2/2 |
| track_deletion | 2/2 |
| target_reappearance | 4/4 |
| clutter_near_target | 5/5 |
| two_crossing_targets | 4/4 |
| high_measurement_noise | 1/1 |
| parallel_targets | 2/2 |
| multiple_targets_in_close_proximity | 1/1 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | constant_velocity_target | exactly one track survives a clean constant-velocity run | got 1 |
| PASS | constant_velocity_target | filter converges to the true position | got (38.000,19.000) vs true (38.0,19.0) |
| PASS | constant_velocity_target | filter converges to the true velocity | got (2.000,1.000) vs true (2.0,1.0) |
| PASS | constant_velocity_target | track becomes confirmed after CONFIRM_HITS consecutive matches | status=confirmed |
| PASS | no_measurement_noise | stationary, noise-free target converges to exact position | got (5.00000,-3.00000) |
| PASS | no_measurement_noise | stationary, noise-free target converges to zero velocity | got (0.00000,0.00000) |
| PASS | prediction_correctness | predict() advances position by velocity*dt | got (2.8349,0.0000) |
| PASS | prediction_correctness | predict() leaves velocity unchanged (no measurement yet) | got 0.9358 |
| PASS | prediction_correctness | predict() grows position covariance (process noise added) | P[0][0]=17.2569 |
| PASS | low_measurement_noise | low-noise track stays within a few sigma of true position | got (59.428,29.108) vs true (59.0,29.5) |
| PASS | low_measurement_noise | low-noise track's velocity estimate is the right order of magnitude and sign | got (0.991,0.408) |
| PASS | low_measurement_noise | track remains confirmed under low noise | status=confirmed |
| PASS | one_missed_detection | a single miss keeps the track alive (not deleted) | got 1 |
| PASS | one_missed_detection | track id is stable across a single miss | before=rt5_t1 after=rt5_t1 |
| PASS | one_missed_detection | missed_count increments to 1 on a miss | got 1 |
| PASS | one_missed_detection | status becomes 'coasting' after one miss (below MAX_MISSED) | status=coasting |
| PASS | one_missed_detection | coasting position is the pure constant-velocity prediction | got 5.001 vs expected 5.0 |
| PASS | one_missed_detection | missed_count resets to 0 once a detection matches again | got 0 |
| PASS | one_missed_detection | track id is still stable after recovering from the miss | before=rt5_t1 after=rt5_t1 |
| PASS | several_missed_detections | after MAX_MISSED=3 consecutive misses, status is 'lost' | status=lost |
| PASS | several_missed_detections | track is not yet dropped from the tracker on the 'lost' step | got 1 |
| PASS | track_deletion | a final 'deleted' row is emitted the step after 'lost' | got 1 deleted rows for rt7_t1 |
| PASS | track_deletion | the deleted track is dropped from the tracker's live track list | live ids=[] |
| PASS | target_reappearance | track list is empty right after deletion | got 0 |
| PASS | target_reappearance | a reappearing target spawns exactly one new track | got 1 |
| PASS | target_reappearance | the new track gets a fresh id, not the deleted track's old id | old=rt8_t1 new=rt8_t2 |
| PASS | target_reappearance | the new track starts 'tentative' | status=tentative |
| PASS | clutter_near_target | still exactly one confirmed/tentative track for the real object |  |
| PASS | clutter_near_target | the real track's estimate stays near the real detection, not the clutter | got 5.000 |
| PASS | clutter_near_target | nearby clutter spawns a separate new tentative track, doesn't hijack the real one | tracks=[('rt9_t1', 'confirmed'), ('rt9_t2', 'tentative')] |
| PASS | clutter_near_target | gating rejects distant clutter - the real track is unaffected by a far outlier | got (5.000,0.000) |
| PASS | clutter_near_target | a far clutter point spawns its own tentative track instead of matching |  |
| PASS | two_crossing_targets | two distinct tracks are maintained approaching the crossing | ids over time=[('rt10_t1', 'rt10_t2'), ('rt10_t1', 'rt10_t2'), ('rt10_t1', 'rt10_t2'), ('rt10_t1', 'rt10_t2')] |
| PASS | two_crossing_targets | left/right track identity stays stable before the crossing point | distinct id-orderings seen before crossing={('rt10_t1', 'rt10_t2')} |
| PASS | two_crossing_targets | exactly two tracks still exist after passing through the crossing | got 2 |
| PASS | two_crossing_targets | both post-crossing tracks are confirmed (neither was lost/respawned) | statuses=['confirmed', 'confirmed'] |
| PASS | high_measurement_noise | high-noise track estimate remains bounded roughly near true position | got (59.692,31.393) vs true (59.0,29.5) |
| PASS | parallel_targets | parallel targets maintain exactly two tracks | got 2 |
| PASS | parallel_targets | both parallel targets are confirmed |  |
| PASS | multiple_targets_in_close_proximity | multiple targets in close proximity maintain distinct tracks | got 3 |
