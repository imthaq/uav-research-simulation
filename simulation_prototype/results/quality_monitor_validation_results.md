# Quality Monitor Validation Results

Deterministic checks for every scoring function and the composite evaluator in perception_quality_monitor.py.

**Result: 71/71 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| T01 | 6/6 |
| T02 | 5/5 |
| T03 | 5/5 |
| T04 | 6/6 |
| T05 | 6/6 |
| T06 | 6/6 |
| T07 | 4/4 |
| T08 | 5/5 |
| T09 | 6/6 |
| T10 | 2/2 |
| T11 | 2/2 |
| T12 | 2/2 |
| T13 | 2/2 |
| T14 | 2/2 |
| T15 | 1/1 |
| T16 | 4/4 |
| T17 | 1/1 |
| T18 | 2/2 |
| T19 | 1/1 |
| T20 | 3/3 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | T01 | trace=0 -> score=1.0 |  |
| PASS | T01 | trace=REF -> score=0.5 |  |
| PASS | T01 | trace=3*REF -> score=0.25 |  |
| PASS | T01 | negative trace -> None |  |
| PASS | T01 | None trace -> None |  |
| PASS | T01 | matrix form: [[1,0],[0,1]] -> same as trace=2 |  |
| PASS | T02 | age=0 -> score=0.0 |  |
| PASS | T02 | age=MATURITY -> score=1.0 |  |
| PASS | T02 | age=MATURITY//2 -> score=0.5 |  |
| PASS | T02 | age > MATURITY -> clamped to 1.0 |  |
| PASS | T02 | None age -> None |  |
| PASS | T03 | missed=0 -> score=1.0 |  |
| PASS | T03 | missed=CEILING -> score=0.0 |  |
| PASS | T03 | missed > CEILING -> clamped to 0.0 |  |
| PASS | T03 | missed=1 -> correct partial score |  |
| PASS | T03 | None missed -> None |  |
| PASS | T04 | agreement=1.0 -> 1.0 |  |
| PASS | T04 | agreement=0.0 -> 0.0 |  |
| PASS | T04 | agreement=0.7 -> 0.7 |  |
| PASS | T04 | agreement > 1 -> clamped to 1.0 |  |
| PASS | T04 | agreement < 0 -> clamped to 0.0 |  |
| PASS | T04 | None agreement -> None |  |
| PASS | T05 | innovation=0 -> score=1.0 |  |
| PASS | T05 | innovation=GATE -> score=0.0 |  |
| PASS | T05 | innovation > GATE -> clamped to 0.0 |  |
| PASS | T05 | innovation=GATE/2 -> score=0.5 |  |
| PASS | T05 | None -> None |  |
| PASS | T05 | negative -> None |  |
| PASS | T06 | error=0.0 -> score=1.0 |  |
| PASS | T06 | error=1.0 -> score=0.0 |  |
| PASS | T06 | error=0.3 -> score=0.7 |  |
| PASS | T06 | error > 1 -> clamped to 0.0 |  |
| PASS | T06 | None -> None |  |
| PASS | T06 | negative -> None |  |
| PASS | T07 | age=0 -> score=1.0 |  |
| PASS | T07 | age=HALF_LIFE -> score=0.5 |  |
| PASS | T07 | age=2*HALF_LIFE -> score=1/3 |  |
| PASS | T07 | None -> None |  |
| PASS | T08 | rate=0.0 -> score=1.0 |  |
| PASS | T08 | rate=CEILING -> score=0.0 |  |
| PASS | T08 | rate > CEILING -> clamped to 0.0 |  |
| PASS | T08 | rate=0.25 -> score=0.5 (half of ceiling) |  |
| PASS | T08 | None -> None |  |
| PASS | T09 | trust=1.0 -> 1.0 |  |
| PASS | T09 | trust=0.0 -> 0.0 |  |
| PASS | T09 | trust=0.75 -> 0.75 |  |
| PASS | T09 | trust > 1 -> clamped to 1.0 |  |
| PASS | T09 | trust < 0 -> clamped to 0.0 |  |
| PASS | T09 | None -> None |  |
| PASS | T10 | all-None signals -> composite=None |  |
| PASS | T10 | every per-signal score is None | {} |
| PASS | T11 | 2-signal composite = 1.0 when both signals are perfect | got 1.0 |
| PASS | T11 | trust=1 age=0 composite = 2/3 per SIGNAL_WEIGHTS | got 0.6666666666666666, expected 0.6666666666666666 |
| PASS | T12 | all-perfect signals -> GOOD | got 'GOOD', score=0.9996977253835109 |
| PASS | T12 | all-perfect score >= GOOD_THRESHOLD | score=0.9996977253835109 |
| PASS | T13 | all-bad signals -> CRITICAL | got 'CRITICAL', score=0.0016835016835016834 |
| PASS | T13 | all-bad score < CRITICAL_THRESHOLD | score=0.0016835016835016834 |
| PASS | T14 | no signals -> CRITICAL (fail-safe, not GOOD) | got 'CRITICAL' |
| PASS | T14 | no signals -> score is None | got None |
| PASS | T15 | single perfect signal -> composite=1.0, not diluted by absent signals | got 1.0 |
| PASS | T16 | evaluate_track_row with scalar covariance returns a level | got 'GOOD' |
| PASS | T16 | covariance signal is not None |  |
| PASS | T16 | age signal is not None |  |
| PASS | T16 | missed_updates signal is not None |  |
| PASS | T17 | JSON-string covariance parsed: cov_score matches manual trace | got 0.6451612903225806, expected 0.6451612903225806 |
| PASS | T18 | healthy_confirmed_track has highest composite score | {'healthy_confirmed_track': 0.9852, 'new_tentative_track': 0.8053, 'coasting_after_misses': 0.564, 'stale_relayed_track': 0.7878, 'unreliable_sensor': 0.0625} |
| PASS | T18 | unreliable_sensor has lowest composite score | {'healthy_confirmed_track': 0.9852, 'new_tentative_track': 0.8053, 'coasting_after_misses': 0.564, 'stale_relayed_track': 0.7878, 'unreliable_sensor': 0.0625} |
| PASS | T19 | custom weights (trust-only) -> composite == trust_score | got 0.65, expected 0.65 |
| PASS | T20 | evaluate() has no ground-truth parameter | found: [] |
| PASS | T20 | score() has no ground-truth parameter | found: [] |
| PASS | T20 | evaluate_track_row() has no ground-truth parameter | found: [] |
