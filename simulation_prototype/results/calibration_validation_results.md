# Calibration Validation Results

Deterministic checks for confidence-calibration math (metrics_analysis.confidence_calibration_metrics and radar_like_model.calibration_pairs).

**Result: 24/24 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| T01 | 3/3 |
| T02 | 1/1 |
| T03 | 1/1 |
| T04 | 2/2 |
| T05 | 2/2 |
| T06 | 3/3 |
| T07 | 2/2 |
| T08 | 1/1 |
| T09 | 4/4 |
| T10 | 3/3 |
| T11 | 2/2 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | T01 | detected+missed included; bad rows excluded | got 2 |
| PASS | T01 | first pair is (0.9, True) |  |
| PASS | T01 | second pair is (0.7, False) |  |
| PASS | T02 | Brier score = 0.10 for known 2-pair case | got 0.1 |
| PASS | T03 | NLL matches hand computation | got 0.366985, expected 0.366985 |
| PASS | T04 | ECE = 0.05 for symmetric 2-bin case | got 0.05 |
| PASS | T04 | MCE = 0.05 for symmetric 2-bin case | got 0.05 |
| PASS | T05 | ECE ~= 0 for perfectly-calibrated bin | got 0.0 |
| PASS | T05 | Brier score = 0.25 at conf=0.5, 50/50 outcomes | got 0.25 |
| PASS | T06 | ECE = 0.95 for maximally overconfident sensor | got 0.95 |
| PASS | T06 | overconfidence_rate = 1.0 | got 1.0 |
| PASS | T06 | underconfidence_rate = 0.0 | got 0.0 |
| PASS | T07 | overconfidence_rate = 0.5 | got 0.5 |
| PASS | T07 | underconfidence_rate = 0.5 | got 0.5 |
| PASS | T08 | n_samples = 2 (2 filtered-in pairs) | got 2 |
| PASS | T09 | n_samples = 0 for empty input |  |
| PASS | T09 | ECE is None for empty input |  |
| PASS | T09 | Brier score is None for empty input |  |
| PASS | T09 | reliability_bins is empty list |  |
| PASS | T10 | 2 pairs extracted (None rows excluded) | got 2 |
| PASS | T10 | first pair: (0.9, True) |  |
| PASS | T10 | second pair: (0.4, False) |  |
| PASS | T11 | conf=1.0 lands in last bin (no IndexError) | bin9 has 2 entries |
| PASS | T11 | conf=0.0 lands in first bin | bin0 has 1 entries |
