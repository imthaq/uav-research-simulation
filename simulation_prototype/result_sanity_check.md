# Final Result Package — Sanity Check

Automated QA pass over the final result package built by `generate_final_result_package.py` into `results/final/` (46 scenarios x 20 trials = 920 runs, generated 2026-07-20T07:16:04.389702+00:00).

**Result: 17/18 checks passed** — 1 FAILED

## Summary by task

| Task | Passed |
|---|---|
| file_presence | 7/7 |
| raw_run_index | 1/1 |
| scenario_summary | 4/4 |
| aggregated_metrics | 1/1 |
| failed_run_report | 1/1 |
| run_metadata | 2/2 |
| statistical_comparisons | 1/2 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | file_presence | raw_run_index.csv exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/raw_run_index.csv |
| PASS | file_presence | aggregated_metrics.csv exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/aggregated_metrics.csv |
| PASS | file_presence | scenario_summary.csv exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/scenario_summary.csv |
| PASS | file_presence | statistical_comparisons.csv exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/statistical_comparisons.csv |
| PASS | file_presence | failed_run_report.csv exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/failed_run_report.csv |
| PASS | file_presence | run_metadata.json exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/run_metadata.json |
| PASS | file_presence | README.md exists in results/final/ | /home/bot47/GitProjects/uav-research/simulation_prototype/results/final/README.md |
| PASS | raw_run_index | raw_run_index.csv row count == num_scenarios * trials | 920 rows vs 920 expected |
| PASS | scenario_summary | every requested scenario appears exactly once in scenario_summary.csv | 46 summary rows for 46 scenarios |
| PASS | scenario_summary | num_passed + num_failed == num_trials for every scenario |  |
| PASS | scenario_summary | mission_success_rate is within [0, 1] for every scenario |  |
| PASS | scenario_summary | ci95_lower <= mean <= ci95_upper for every headline metric |  |
| PASS | aggregated_metrics | spot-checked (scenario, metric) means recompute identically from raw_run_index.csv | baseline/collision_risk_count: raw=None vs agg=None; baseline/mission_success: raw=None vs agg=None; false_positive/collision_risk_count: raw=None vs agg=None; false_positive/mission_success: raw=None vs agg=None; false_negative/collision_risk_count: raw=None vs agg=None; false_negative/mission_success: raw=None vs agg=None |
| PASS | failed_run_report | failed_run_report.csv row count == total FAIL rows in raw_run_index.csv |  |
| PASS | run_metadata | run_metadata.json total_trials_run matches raw_run_index.csv row count |  |
| PASS | run_metadata | run_metadata.json overall_status matches presence of any FAILed trial |  |
| **FAIL** | statistical_comparisons | at least one statistical comparison was produced (when baseline is present) | 0 comparisons |
| PASS | statistical_comparisons | every reported p-value falls within [0, 1] |  |
