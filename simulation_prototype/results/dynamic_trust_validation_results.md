# Dynamic Trust Validation Results (Task 7)

Validates TrustTracker behavior regarding sensor trust penalties and recovery.

**Result: 13/13 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| disagreement_decreases_trust | 1/1 |
| dropout_decreases_trust | 1/1 |
| staleness_decreases_trust | 1/1 |
| false_alarms_decreases_trust | 1/1 |
| inconsistent_updates_decreases_trust | 1/1 |
| trust_never_negative | 1/1 |
| trust_never_exceeds_max | 1/1 |
| trust_gradually_recovers | 1/1 |
| one_good_update_insufficient | 1/1 |
| hysteresis | 1/1 |
| ground_truth_not_used | 1/1 |
| trust_updates_are_logged | 2/2 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | disagreement_decreases_trust | trust strictly decreases every step under repeated hard disagreement | history=[1.0, 0.9385365853658537, 0.8955121951219512, 0.8653951219512195, 0.8443131707317073, 0.8295558048780488, 0.8192256487804879] |
| PASS | dropout_decreases_trust | trust strictly decreases every step under repeated dropout | history=[1.0, 0.9145365853658537, 0.8547121951219512, 0.8128351219512194, 0.7835211707317072, 0.7630014048780487, 0.7486375687804877] |
| PASS | staleness_decreases_trust | trust strictly decreases every step under repeated staleness | history=[1.0, 0.932183644189383, 0.8847121951219512, 0.8514821807747489, 0.8282211707317073, 0.8119384637015782, 0.8005405687804878] |
| PASS | false_alarms_decreases_trust | trust strictly decreases after repeated false alarms (low confidence, isolated) | history=[1.0, 0.8766666666666667, 0.7903333333333333, 0.7299, 0.6875966666666666, 0.6579843333333333, 0.6372557] |
| PASS | inconsistent_updates_decreases_trust | trust decreases overall when updates are wildly inconsistent and covariance spikes | history=[1.0, 0.9445365853658536, 0.8643184668989546, 0.8510229268292683, 0.7988589059233449, 0.8005501934494773, 0.7635279925574913] |
| PASS | trust_never_negative | trust never drops below 0 | final=0.0500 |
| PASS | trust_never_exceeds_max | trust never exceeds TRUST_MAX | max seen=1.0000 |
| PASS | trust_gradually_recovers | trust rises monotonically once good signals resume |  |
| PASS | one_good_update_insufficient | a single perfect update does not restore trust to TRUST_MAX | after one good update=0.8220 |
| PASS | hysteresis | trust recovery contains hysteresis (alpha_up < alpha_down) to drop fast but recover slowly | up=0.08, down=0.3 |
| PASS | ground_truth_not_used | trust does not directly use ground-truth error in sources |  |
| PASS | trust_updates_are_logged | trust snapshot provides per-radar scores for logging |  |
| PASS | trust_updates_are_logged | last_signals provides signal breakdown for logging |  |
