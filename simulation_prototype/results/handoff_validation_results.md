# Handoff Validation Results

Deterministic checks for trigger evaluation, mode selection, episode lifecycle, and API safety in perception_handoff_model.py.

**Result: 57/57 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| T01 | 2/2 |
| T02 | 1/1 |
| T03 | 2/2 |
| T04 | 3/3 |
| T05 | 2/2 |
| T06 | 3/3 |
| T07 | 2/2 |
| T08 | 2/2 |
| T09 | 2/2 |
| T10 | 2/2 |
| T11 | 1/1 |
| T12 | 1/1 |
| T13 | 1/1 |
| T14 | 2/2 |
| T15 | 2/2 |
| T16 | 2/2 |
| T17 | 1/1 |
| T18 | 6/6 |
| T19 | 5/5 |
| T20 | 4/4 |
| T21 | 5/5 |
| T22 | 6/6 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | T01 | empty signals -> no triggers | got [] |
| PASS | T01 | level is None when perception_quality_level not supplied |  |
| PASS | T02 | sensor_failed=True fires SENSOR_FAILURE |  |
| PASS | T03 | dropout_rate=CEILING fires SENSOR_FAILURE |  |
| PASS | T03 | dropout_rate just below CEILING does NOT fire SENSOR_FAILURE |  |
| PASS | T04 | perception_quality_level=CRITICAL fires CRITICAL_QUALITY |  |
| PASS | T04 | level returned matches the supplied value |  |
| PASS | T04 | perception_quality_level=GOOD does NOT fire CRITICAL_QUALITY |  |
| PASS | T05 | disagreement=threshold fires SENSOR_DISAGREEMENT |  |
| PASS | T05 | disagreement just below threshold does NOT fire |  |
| PASS | T06 | trace=EXCESSIVE_COVARIANCE_TRACE fires EXCESSIVE_COVARIANCE |  |
| PASS | T06 | trace just below threshold does NOT fire |  |
| PASS | T06 | matrix covariance with matching trace also fires |  |
| PASS | T07 | missed=CEILING fires REPEATED_MISSED_DETECTIONS |  |
| PASS | T07 | missed=CEILING-1 does NOT fire |  |
| PASS | T08 | comm_age=STALE_THRESHOLD fires STALE_DISTRIBUTED_TRACK |  |
| PASS | T08 | comm_age just below threshold does NOT fire |  |
| PASS | T09 | communication_recovered=True fires COMMUNICATION_RECOVERY |  |
| PASS | T09 | communication_recovered=False does NOT fire |  |
| PASS | T10 | all three triggers fire | ['sensor_failure', 'critical_quality_status', 'strong_sensor_disagreement'] |
| PASS | T10 | SENSOR_FAILURE precedes CRITICAL_QUALITY in active_triggers list | ['sensor_failure', 'critical_quality_status', 'strong_sensor_disagreement'] |
| PASS | T11 | sensor failure + no resources -> SAFE_HOLD | got 'safe_hold_no_reliable_source' |
| PASS | T12 | sensor failure + radar available -> RADAR_ONLY_FALLBACK | got 'local_radar_only_fallback' |
| PASS | T13 | sensor disagreement + peer available -> REQUEST_PEER_TRACK (over local sensors) | got 'request_neighbouring_uav_track' |
| PASS | T14 | stale distributed track + radar+peer -> RADAR_ONLY (peer excluded) | got 'local_radar_only_fallback' |
| PASS | T14 | stale distributed track + lidar+peer -> LIDAR_ONLY (peer excluded) | got 'local_lidar_only_fallback' |
| PASS | T15 | healthy track -> NO_HANDOFF | got 'no_handoff' |
| PASS | T15 | primary_trigger is None for NO_HANDOFF |  |
| PASS | T16 | decide: failed sensor + no resources -> SAFE_HOLD | got 'safe_hold_no_reliable_source' |
| PASS | T16 | primary_trigger is SENSOR_FAILURE | got 'sensor_failure' |
| PASS | T17 | decide: failed sensor + radar -> RADAR_ONLY_FALLBACK | got 'local_radar_only_fallback' |
| PASS | T18 | handing_off=True right after trigger |  |
| PASS | T18 | handing_off=False after healthy step |  |
| PASS | T18 | EVENT_TRIGGERED in log | ['handoff_triggered', 'handoff_mode_selected', 'handoff_resolved'] |
| PASS | T18 | EVENT_RESOLVED in log | ['handoff_triggered', 'handoff_mode_selected', 'handoff_resolved'] |
| PASS | T18 | resolved entry final_outcome = OUTCOME_RECOVERED | got 'handoff_no_longer_needed' |
| PASS | T18 | resolved episode duration_steps = 10 - 5 = 5 | got 5 |
| PASS | T19 | step 3 duration_steps = 1 | got 1 |
| PASS | T19 | step 4 duration_steps = 2 | got 2 |
| PASS | T19 | step 5 duration_steps = 3 | got 3 |
| PASS | T19 | step 6 duration_steps = 4 | got 4 |
| PASS | T19 | step 7 duration_steps = 5 | got 5 |
| PASS | T20 | both uav_ids are handing_off after trigger |  |
| PASS | T20 | close_all returns 2 entries | got 2 |
| PASS | T20 | neither uav_id is handing_off after close_all |  |
| PASS | T20 | final_outcome is OUTCOME_UNRESOLVED_AT_END after force-close | {'unresolved_at_simulation_end'} |
| PASS | T21 | 2 handoff episodes triggered | got 2 |
| PASS | T21 | SENSOR_FAILURE is the only primary_trigger seen | {'sensor_failure': 2} |
| PASS | T21 | OUTCOME_RECOVERED counted once | {'handoff_no_longer_needed': 1, 'unresolved_at_simulation_end': 1} |
| PASS | T21 | OUTCOME_UNRESOLVED_AT_END counted once | {'handoff_no_longer_needed': 1, 'unresolved_at_simulation_end': 1} |
| PASS | T21 | avg_resolved_duration_steps = 5 (UAV 0 resolved at step5-0=5) | got 5.0 |
| PASS | T22 | decide() has no ground-truth parameter | found: [] |
| PASS | T22 | decide_for_track_row() has no ground-truth parameter | found: [] |
| PASS | T22 | handing_off() has no ground-truth parameter | found: [] |
| PASS | T22 | close_all() has no ground-truth parameter | found: [] |
| PASS | T22 | summary() has no ground-truth parameter | found: [] |
| PASS | T22 | evaluate_triggers() has no ground-truth parameter | found: [] |
