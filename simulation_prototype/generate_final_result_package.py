"""
generate_final_result_package.py

Task 13: builds the project's final result package into results/final/,
and a top-level result_sanity_check.md validating that package.

Reuses the same run/metric machinery every other results-facing script in
this project already relies on - metrics_analysis.run_once() (itself
metrics_analysis.py's own thin wrapper around
simple_swarm_sim.run_radar_track_fusion_pipeline) and its
SWARM_FIELDS/PERCEPTION_FIELDS/COMMUNICATION_FIELDS metric lists - so the
metrics here are guaranteed to mean exactly what they mean everywhere else
in the project (results_summary.csv, the plots, the ablation study), not a
second, drifting definition.

results/final/ contents
------------------------
  raw_run_index.csv          - one row per (scenario, trial): seed, status
                                (PASS/FAIL), runtime, scenario parameters,
                                and every metric run_once() produced (or
                                blank metrics for a FAILed trial).
  aggregated_metrics.csv     - long format: one row per (scenario, metric)
                                with n, mean, median, stdev, and a 95% CI
                                (normal approximation, matching the
                                1.96*sd/sqrt(n) convention generate_plots.py
                                already uses elsewhere in this project).
  scenario_summary.csv       - one row per scenario: trial/pass/fail
                                counts and mean/median/stdev/CI for a
                                headline metric subset (mission success
                                rate, collision risk, response time,
                                position RMSE, track continuity,
                                communication load).
  statistical_comparisons.csv - pairwise comparisons (baseline vs. every
                                other scenario, plus the fusion-mode and
                                faulty-sensor-fusion-mode groups) via a
                                Welch's t-test with a normal-approximation
                                p-value (no scipy dependency, consistent
                                with the rest of this project's stats).
  failed_run_report.csv      - every FAILed trial with its error, plus a
                                per-scenario failure count.
  run_metadata.json          - seeds, trial counts, config hash, wall
                                clock, overall PASS/FAIL.
  README.md                  - short manifest describing the above.

result_sanity_check.md
-----------------------
An automated QA pass over the package just built (reusing
validation_common.Checker, the same PASS/FAIL-table report format every
other *_validation.py script in this project uses): file presence, row-
count consistency, recomputation spot-checks against the raw data, and
sane value ranges. Written at the project root next to the other
top-level docs.

Usage:
    python generate_final_result_package.py
    python generate_final_result_package.py --trials 20
    python generate_final_result_package.py --scenarios baseline naive_fusion trust_weighted_fusion --trials 5
"""

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import statistics
import sys
import time
import traceback
from datetime import datetime, timezone

from metrics_analysis import run_once, scenario_params, SWARM_FIELDS, PERCEPTION_FIELDS, COMMUNICATION_FIELDS
from validation.validation_common import Checker

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

# Every numeric metric run_once() can produce, pulled straight from
# metrics_analysis.py's own field lists so this can never silently drift
# out of sync with what run_once() actually returns. mission_success is a
# bool in run_once()'s output; it's coerced to 0/1 below so it aggregates
# (mean = success rate) the same way every other metric does.
METRIC_FIELDS = list(dict.fromkeys(SWARM_FIELDS + PERCEPTION_FIELDS + COMMUNICATION_FIELDS))

# Headline subset promoted into the wide scenario_summary.csv (every
# metric still gets its own row in the long-format aggregated_metrics.csv
# regardless).
HEADLINE_METRICS = [
    "mission_success", "collision_risk_count", "avg_response_time_s",
    "rmse_position_error", "track_continuity", "communication_load",
]

# Metrics checked in every statistical comparison.
COMPARISON_METRICS = ["collision_risk_count", "mission_success", "avg_response_time_s", "rmse_position_error"]

# Fusion-mode comparison groups: scenarios that only differ in fusion_mode
# (or a faulty-sensor variant of it), so a pairwise comparison within a
# group isolates the effect of fusion strategy specifically.
FUSION_COMPARISON_GROUPS = {
    "core_fusion_modes": ["no_fusion_matched", "naive_fusion", "trust_weighted_fusion"],
    "faulty_sensor_fusion_modes": [
        "faulty_sensor_naive_fusion", "faulty_sensor_confidence_weighted_fusion",
        "faulty_sensor_trust_weighted_fusion_fixed", "faulty_sensor_trust_weighted_fusion_dynamic",
        "faulty_sensor_covariance_weighted_fusion",
    ],
}


# ---------------------------------------------------------------------
# Stats helpers (stdlib only - statistics + math, no scipy dependency;
# the 1.96*sd/sqrt(n) 95% CI matches generate_plots.py's mean_ci()).
# ---------------------------------------------------------------------

def _config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]


def compute_stats(vals):
    """mean/median/stdev/95% CI (normal approximation) for a list of
    numbers, skipping Nones. n<2 gets a defined mean/median but no spread
    (stdev/CI reported as 0.0, not fabricated from a single sample)."""
    vals = [v for v in vals if v is not None]
    n = len(vals)
    if n == 0:
        return dict(n=0, mean=None, median=None, stdev=None,
                    ci95_halfwidth=None, ci95_lower=None, ci95_upper=None)
    mean = statistics.mean(vals)
    median = statistics.median(vals)
    sd = statistics.stdev(vals) if n > 1 else 0.0
    half = (1.96 * sd / math.sqrt(n)) if n > 1 else 0.0
    return dict(n=n, mean=round(mean, 4), median=round(median, 4), stdev=round(sd, 4),
                ci95_halfwidth=round(half, 4), ci95_lower=round(mean - half, 4),
                ci95_upper=round(mean + half, 4))


def _norm_cdf(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2)))


def welch_t_test(a, b):
    """Welch's t-test for two independent samples with unequal variance,
    with a normal-approximation p-value (math.erf-based, no scipy) - the
    same trade-off generate_plots.py already makes for its CIs. Flagged
    explicitly as an approximation in the output field name so a reader
    knows to reach for scipy.stats.ttest_ind(equal_var=False) if an exact
    Student's-t p-value is needed."""
    a = [v for v in a if v is not None]
    b = [v for v in b if v is not None]
    na, nb = len(a), len(b)
    result = dict(n_a=na, n_b=nb, mean_a=None, mean_b=None, mean_diff=None,
                  t_stat=None, p_value_normal_approx=None, significant_95=None)
    if na < 2 or nb < 2:
        return result
    mean_a, mean_b = statistics.mean(a), statistics.mean(b)
    var_a, var_b = statistics.variance(a), statistics.variance(b)
    se = math.sqrt(var_a / na + var_b / nb)
    t_stat = 0.0 if se == 0 else (mean_a - mean_b) / se
    p = 2 * (1 - _norm_cdf(abs(t_stat)))
    result.update(mean_a=round(mean_a, 4), mean_b=round(mean_b, 4),
                  mean_diff=round(mean_a - mean_b, 4), t_stat=round(t_stat, 4),
                  p_value_normal_approx=round(p, 4), significant_95=p < 0.05)
    return result


# ---------------------------------------------------------------------
# Run + aggregate
# ---------------------------------------------------------------------

def run_all(config, scenario_names, trials, base_seed):
    """Runs `trials` seeded trials of every scenario via
    metrics_analysis.run_once(), wrapping each in try/except so one
    failing trial is recorded as a FAIL row rather than aborting the
    whole package build. Returns (raw_rows, fail_rows)."""
    raw_rows, fail_rows = [], []
    total = len(scenario_names) * trials
    done = 0

    for scenario_name in scenario_names:
        params = scenario_params(config["scenarios"][scenario_name])
        for trial in range(1, trials + 1):
            seed = base_seed + (trial - 1)
            t0 = time.monotonic()
            status, error, m = "PASS", None, None
            try:
                m = run_once(copy.deepcopy(config), scenario_name, seed)
            except Exception:
                status = "FAIL"
                error = traceback.format_exc(limit=3)
            runtime_s = round(time.monotonic() - t0, 4)

            row = {
                "scenario": scenario_name, "trial": trial, "seed": seed,
                "status": status, "runtime_seconds": runtime_s, **params,
            }
            for key in METRIC_FIELDS:
                if m is None:
                    row[key] = None
                elif key == "mission_success":
                    row[key] = 1 if m["mission_success"] else 0
                else:
                    row[key] = m.get(key)

            if status == "FAIL":
                row["error"] = error.strip().splitlines()[-1]
                fail_rows.append({**row, "error_detail": error})
            else:
                row["error"] = None

            raw_rows.append(row)
            done += 1
            if done % 25 == 0 or done == total:
                print(f"  [{done}/{total}] {scenario_name} trial {trial} (seed={seed}): {status}")

    return raw_rows, fail_rows


def aggregate(raw_rows, scenario_names):
    """Long-format per-(scenario, metric) stats + a wide per-scenario
    summary over HEADLINE_METRICS. Only PASS rows feed the numbers - a
    FAILed trial has no metrics to aggregate."""
    agg_rows = []
    summary_rows = []

    for scenario in scenario_names:
        srows = [r for r in raw_rows if r["scenario"] == scenario]
        passed = [r for r in srows if r["status"] == "PASS"]
        failed = [r for r in srows if r["status"] == "FAIL"]

        for metric in METRIC_FIELDS:
            stats = compute_stats([r[metric] for r in passed])
            agg_rows.append({"scenario": scenario, "metric": metric, **stats})

        summary = {
            "scenario": scenario,
            "num_trials": len(srows),
            "num_passed": len(passed),
            "num_failed": len(failed),
            "status": "PASS" if not failed else "FAIL",
        }
        for metric in HEADLINE_METRICS:
            stats = compute_stats([r[metric] for r in passed])
            for stat_key in ("mean", "median", "stdev", "ci95_lower", "ci95_upper"):
                summary[f"{metric}_{stat_key}"] = stats[stat_key]
        summary["mission_success_rate"] = summary.get("mission_success_mean")
        summary_rows.append(summary)

    return agg_rows, summary_rows


def build_comparisons(raw_rows, scenario_names):
    """baseline-vs-every-other-scenario, plus pairwise comparisons within
    each fusion-mode group in FUSION_COMPARISON_GROUPS, on
    COMPARISON_METRICS, via welch_t_test."""
    comparisons = []

    def passed_values(scenario, metric):
        return [r[metric] for r in raw_rows if r["scenario"] == scenario and r["status"] == "PASS"]

    if "baseline" in scenario_names:
        for scenario in scenario_names:
            if scenario == "baseline":
                continue
            for metric in COMPARISON_METRICS:
                a, b = passed_values("baseline", metric), passed_values(scenario, metric)
                if not a or not b:
                    continue
                comparisons.append({
                    "comparison_group": "baseline_vs_scenario",
                    "group_a": "baseline", "group_b": scenario, "metric": metric,
                    **welch_t_test(a, b),
                })

    for group_name, members in FUSION_COMPARISON_GROUPS.items():
        present = [m for m in members if m in scenario_names]
        for i in range(len(present)):
            for j in range(i + 1, len(present)):
                a_name, b_name = present[i], present[j]
                for metric in COMPARISON_METRICS:
                    a, b = passed_values(a_name, metric), passed_values(b_name, metric)
                    if not a or not b:
                        continue
                    comparisons.append({
                        "comparison_group": group_name,
                        "group_a": a_name, "group_b": b_name, "metric": metric,
                        **welch_t_test(a, b),
                    })

    return comparisons


# ---------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------

def _write_csv(path, rows, fieldnames=None):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not rows:
        # Still write a header-only file so downstream tooling/sanity
        # checks always find the file, even for an edge case with zero
        # rows (e.g. --scenarios with nothing that produced a comparison).
        with open(path, "w", newline="") as f:
            if fieldnames:
                csv.DictWriter(f, fieldnames=fieldnames).writeheader()
        return
    fieldnames = fieldnames or list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


README_TEMPLATE = """# Final Result Package

Generated by `generate_final_result_package.py` on {generated_at}.

| File | Contents |
|---|---|
| `raw_run_index.csv` | One row per (scenario, trial): seed, PASS/FAIL status, runtime, scenario parameters, and every run-level metric. The raw data everything else here is computed from. |
| `aggregated_metrics.csv` | Long format: one row per (scenario, metric) with n, mean, median, stdev, and a 95% confidence interval. |
| `scenario_summary.csv` | One row per scenario: trial/pass/fail counts and mean/median/stdev/CI for the headline metrics ({headline}). |
| `statistical_comparisons.csv` | Pairwise Welch's t-test comparisons (baseline vs. every other scenario, plus the fusion-mode comparison groups). |
| `failed_run_report.csv` | Every FAILed trial, with its error. |
| `run_metadata.json` | Seeds, trial counts, config hash, wall-clock time, overall PASS/FAIL. |

See `../../result_sanity_check.md` (project root) for an automated QA pass over this package.
"""


def write_package(out_dir, raw_rows, fail_rows, agg_rows, summary_rows, comparisons, metadata):
    os.makedirs(out_dir, exist_ok=True)

    raw_fieldnames = (["scenario", "trial", "seed", "status", "runtime_seconds"]
                       + list(scenario_params({}).keys()) + METRIC_FIELDS + ["error"])
    _write_csv(os.path.join(out_dir, "raw_run_index.csv"), raw_rows, raw_fieldnames)

    agg_fieldnames = ["scenario", "metric", "n", "mean", "median", "stdev",
                       "ci95_halfwidth", "ci95_lower", "ci95_upper"]
    _write_csv(os.path.join(out_dir, "aggregated_metrics.csv"), agg_rows, agg_fieldnames)

    summary_fieldnames = ["scenario", "num_trials", "num_passed", "num_failed", "status",
                           "mission_success_rate"]
    for metric in HEADLINE_METRICS:
        for stat_key in ("mean", "median", "stdev", "ci95_lower", "ci95_upper"):
            summary_fieldnames.append(f"{metric}_{stat_key}")
    _write_csv(os.path.join(out_dir, "scenario_summary.csv"), summary_rows, summary_fieldnames)

    comparison_fieldnames = ["comparison_group", "group_a", "group_b", "metric",
                              "n_a", "n_b", "mean_a", "mean_b", "mean_diff",
                              "t_stat", "p_value_normal_approx", "significant_95"]
    _write_csv(os.path.join(out_dir, "statistical_comparisons.csv"), comparisons, comparison_fieldnames)

    fail_fieldnames = raw_fieldnames + ["error_detail"]
    _write_csv(os.path.join(out_dir, "failed_run_report.csv"), fail_rows, fail_fieldnames)

    with open(os.path.join(out_dir, "run_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    with open(os.path.join(out_dir, "README.md"), "w") as f:
        f.write(README_TEMPLATE.format(
            generated_at=metadata["generated_at"], headline=", ".join(HEADLINE_METRICS)))


# ---------------------------------------------------------------------
# Sanity check (reuses validation_common.Checker, same report style as
# every other *_validation.py script in this project)
# ---------------------------------------------------------------------

def sanity_check(out_dir, raw_rows, fail_rows, agg_rows, summary_rows, comparisons,
                  metadata, scenario_names, trials):
    c = Checker()

    expected_files = ["raw_run_index.csv", "aggregated_metrics.csv", "scenario_summary.csv",
                       "statistical_comparisons.csv", "failed_run_report.csv",
                       "run_metadata.json", "README.md"]
    for fname in expected_files:
        path = os.path.join(out_dir, fname)
        c.check("file_presence", f"{fname} exists in results/final/", os.path.exists(path), path)

    expected_raw_rows = len(scenario_names) * trials
    c.check("raw_run_index", "raw_run_index.csv row count == num_scenarios * trials",
            len(raw_rows) == expected_raw_rows,
            f"{len(raw_rows)} rows vs {expected_raw_rows} expected")

    summary_scenarios = [r["scenario"] for r in summary_rows]
    c.check("scenario_summary", "every requested scenario appears exactly once in scenario_summary.csv",
            sorted(summary_scenarios) == sorted(scenario_names),
            f"{len(summary_scenarios)} summary rows for {len(scenario_names)} scenarios")

    counts_consistent = all(r["num_passed"] + r["num_failed"] == r["num_trials"] for r in summary_rows)
    c.check("scenario_summary", "num_passed + num_failed == num_trials for every scenario",
            counts_consistent)

    rates_in_range = all(
        r["mission_success_rate"] is None or 0.0 <= r["mission_success_rate"] <= 1.0
        for r in summary_rows)
    c.check("scenario_summary", "mission_success_rate is within [0, 1] for every scenario",
            rates_in_range)

    ci_sane = all(
        r[f"{m}_ci95_lower"] is None or r[f"{m}_ci95_upper"] is None
        or (r[f"{m}_ci95_lower"] - 1e-9) <= r[f"{m}_mean"] <= (r[f"{m}_ci95_upper"] + 1e-9)
        for r in summary_rows for m in HEADLINE_METRICS
    )
    c.check("scenario_summary", "ci95_lower <= mean <= ci95_upper for every headline metric",
            ci_sane)

    # Recomputation spot-check: for a sample scenario/metric, recompute
    # mean directly from raw_run_index rows and compare to what
    # aggregated_metrics.csv reports for that same (scenario, metric).
    spot_checks_ok = True
    spot_detail = []
    sample_scenarios = scenario_names[:3]
    for scenario in sample_scenarios:
        for metric in ("collision_risk_count", "mission_success"):
            raw_vals = [r[metric] for r in raw_rows if r["scenario"] == scenario and r["status"] == "PASS"]
            expected_mean = round(statistics.mean(raw_vals), 4) if raw_vals else None
            agg_row = next((r for r in agg_rows if r["scenario"] == scenario and r["metric"] == metric), None)
            reported_mean = agg_row["mean"] if agg_row else None
            ok = (expected_mean == reported_mean) or (
                expected_mean is not None and reported_mean is not None
                and abs(expected_mean - reported_mean) < 1e-6)
            spot_checks_ok = spot_checks_ok and ok
            spot_detail.append(f"{scenario}/{metric}: raw={expected_mean} vs agg={reported_mean}")
    c.check("aggregated_metrics", "spot-checked (scenario, metric) means recompute identically from raw_run_index.csv",
            spot_checks_ok, "; ".join(spot_detail))

    c.check("failed_run_report", "failed_run_report.csv row count == total FAIL rows in raw_run_index.csv",
            len(fail_rows) == sum(1 for r in raw_rows if r["status"] == "FAIL"))

    c.check("run_metadata", "run_metadata.json total_trials_run matches raw_run_index.csv row count",
            metadata.get("total_trials_run") == len(raw_rows))

    c.check("run_metadata", "run_metadata.json overall_status matches presence of any FAILed trial",
            metadata.get("overall_status") == ("FAIL" if fail_rows else "PASS"))

    comparisons_present = len(comparisons) > 0 if "baseline" in scenario_names else True
    c.check("statistical_comparisons", "at least one statistical comparison was produced (when baseline is present)",
            comparisons_present, f"{len(comparisons)} comparisons")

    valid_p = all(
        cmp["p_value_normal_approx"] is None or 0.0 <= cmp["p_value_normal_approx"] <= 1.0
        for cmp in comparisons)
    c.check("statistical_comparisons", "every reported p-value falls within [0, 1]", valid_p)

    intro = (
        f"Automated QA pass over the final result package built by "
        f"`generate_final_result_package.py` into `results/final/` "
        f"({len(scenario_names)} scenarios x {trials} trials = {expected_raw_rows} runs, "
        f"generated {metadata['generated_at']})."
    )
    report_path = os.path.join(_ROOT_DIR, "result_sanity_check.md")
    c.write_markdown(report_path, "Final Result Package — Sanity Check", intro)
    failed = c.print_summary()
    print(f"\nSanity-check report -> {report_path}")
    return failed


def main():
    parser = argparse.ArgumentParser(description="Task 13: build the final result package")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--scenarios", nargs="+", default=None,
                         help="Scenarios to include (default: every scenario in --config)")
    parser.add_argument("--trials", type=int, default=20, help="Trials per scenario (default: 20)")
    parser.add_argument("--base-seed", type=int, default=None,
                         help="Master seed (default: config['sim']['seed'])")
    parser.add_argument("--output-dir", default=os.path.join(_ROOT_DIR, "results", "final"))
    args = parser.parse_args()

    if args.trials < 1:
        sys.exit(f"--trials must be >= 1 (got {args.trials})")

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = args.scenarios or list(config["scenarios"].keys())
    missing = [s for s in scenario_names if s not in config["scenarios"]]
    if missing:
        sys.exit(f"Unknown scenario(s): {missing}. Available: {list(config['scenarios'].keys())}")

    base_seed = args.base_seed if args.base_seed is not None else config["sim"]["seed"]

    print(f"=== Task 13: final result package ===")
    print(f"{len(scenario_names)} scenarios x {args.trials} trials "
          f"({len(scenario_names) * args.trials} total runs), base_seed={base_seed}\n")

    started_at = datetime.now(timezone.utc).isoformat()
    t0 = time.monotonic()

    raw_rows, fail_rows = run_all(config, scenario_names, args.trials, base_seed)
    agg_rows, summary_rows = aggregate(raw_rows, scenario_names)
    comparisons = build_comparisons(raw_rows, scenario_names)

    wall_clock = round(time.monotonic() - t0, 3)
    overall_status = "FAIL" if fail_rows else "PASS"

    metadata = {
        "generated_at": started_at,
        "wall_clock_seconds": wall_clock,
        "overall_status": overall_status,
        "num_scenarios": len(scenario_names),
        "trials_per_scenario": args.trials,
        "total_trials_run": len(raw_rows),
        "total_trials_failed": len(fail_rows),
        "base_seed": base_seed,
        "config_path": os.path.abspath(args.config),
        "config_sha256_16": _config_hash(config),
        "scenarios": scenario_names,
        "seeds_used": sorted({r["seed"] for r in raw_rows}),
    }

    write_package(args.output_dir, raw_rows, fail_rows, agg_rows, summary_rows, comparisons, metadata)
    print(f"\nFinal result package -> {args.output_dir}/")
    print(f"  raw_run_index.csv         ({len(raw_rows)} rows)")
    print(f"  aggregated_metrics.csv    ({len(agg_rows)} rows)")
    print(f"  scenario_summary.csv      ({len(summary_rows)} rows)")
    print(f"  statistical_comparisons.csv ({len(comparisons)} rows)")
    print(f"  failed_run_report.csv     ({len(fail_rows)} rows)")
    print(f"  run_metadata.json, README.md")
    print(f"\nWall clock: {wall_clock}s | overall_status={overall_status} "
          f"({len(fail_rows)}/{len(raw_rows)} trials failed)")

    failed_checks = sanity_check(args.output_dir, raw_rows, fail_rows, agg_rows, summary_rows,
                                  comparisons, metadata, scenario_names, args.trials)

    return 1 if (overall_status == "FAIL" or failed_checks) else 0


if __name__ == "__main__":
    sys.exit(main())
