"""
run_experiments.py

Monte Carlo experiment runner: executes every scenario in the config for a
configurable number of repeated trials, saving three things per invocation:

  1. Run-level results  - one row per (scenario, trial), unchanged schema
     from before this task so `generate_plots.py` / `metrics_analysis.py`
     keep working against `results/results_summary.csv` without any
     changes on their end.
  2. Scenario-level summary - one row per scenario, aggregating every
     numeric metric across its trials into mean/median/stdev/min/max/95%
     confidence interval, plus the mission-success rate.
  3. Experiment metadata - a JSON record of exactly how this run was
     produced (config used, trial count, seed strategy, per-scenario
     seeds, timing, environment) so a result can always be traced back to
     the conditions that generated it.

Monte Carlo trial count
------------------------
Each trial re-runs a scenario with a different seed, so results
aggregate a scenario's *distribution* of outcomes (collision counts,
response times, ...) rather than a single run's outcome, which matters
whenever a scenario has any run-to-run variance (noise, dropout,
fusion-mode comparisons, ...). Recommended trial counts:
  - >= 20 trials for initial/exploratory analysis - enough for the
    95% CI to start being meaningfully tighter than a handful of runs.
  - 50-100 trials for paper-ready final results, if computationally
    feasible - use --skip-step-logs to drop the per-trial CSV logs (the
    dominant cost at that scale) while still keeping every aggregated
    number.

Random seed control
--------------------
--base-seed sets the master seed the whole experiment is derived from
(defaults to config["sim"]["seed"], matching pre-Task-16 behavior).
--seed-mode picks how per-(scenario, trial) seeds are derived from it:
  - "sequential" (default, backward-compatible) - seed = base_seed +
    (trial - 1), the same seed sequence shared across every scenario, so
    trial N of every scenario is comparable at matched noise draws.
  - "random" - seeds are instead drawn from a single random.Random
    seeded with base_seed, independently per (scenario, trial), so
    different scenarios don't share the same per-trial seed - the usual
    choice for a "genuine" Monte Carlo sweep where scenarios shouldn't be
    yoked together.
Either mode is fully reproducible: the same --base-seed and --seed-mode
always regenerate the same run matrix and results.

Automatic scenario generation
------------------------------
`build_run_matrix()` derives the complete (scenario, trial, seed) list to
execute purely from `config["scenarios"]` and the requested trial count -
nothing about which scenarios exist or how many trials to run is
hardcoded here, so adding a scenario to the config or bumping --trials is
all that's needed to change what gets run.
"""

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import platform
import random
import statistics
import sys
import time
from datetime import datetime, timezone

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from simple_swarm_sim import Simulation

# --- run-level (one row per scenario per trial) output schema -------------
# Unchanged from pre-Task-16 (plus "seed" appended at the end) so
# generate_plots.py / metrics_analysis.py keep reading this file's
# existing columns without modification.
RUN_FIELDNAMES = [
    "scenario",
    "run_number",
    "fusion_mode",
    "false_positive_rate",
    "false_negative_rate",
    "noise_level",
    "latency_steps",
    "dropout_probability",
    "confidence_error_level",
    "collision_risk_count",
    "unnecessary_avoidance_count",
    "missed_response_count",
    "fusion_recovery_count",
    "mission_success",
    "avg_response_time_s",
    "total_near_misses",
    "avg_formation_error",
    "seed",
]

# Numeric metrics aggregated into the scenario-level summary. Each gets a
# mean/median/stdev/min/max/ci95_lower/ci95_upper column group.
AGGREGATE_METRICS = [
    "collision_risk_count",
    "unnecessary_avoidance_count",
    "missed_response_count",
    "fusion_recovery_count",
    "avg_response_time_s",
    "total_near_misses",
    "avg_formation_error",
]

STAT_SUFFIXES = ("mean", "median", "stdev", "min", "max", "ci95_lower", "ci95_upper")

# Two-tailed 95% critical t-values, indexed by degrees of freedom (n - 1).
# Beyond df=30 the t-distribution is already within ~1% of the normal
# z=1.960 critical value, so larger df all fall back to that - accurate
# enough at the >= 20-trial sample sizes this framework recommends.
_T95_TABLE = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
    6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
    16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
    21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060,
    26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
    40: 2.021, 60: 2.000, 120: 1.980,
}
_T95_Z_FALLBACK = 1.960


def _t95(df):
    """95% two-tailed t critical value for `df` degrees of freedom - exact
    lookup for df<=30 (and the 40/60/120 landmarks), normal-approximation
    fallback beyond that."""
    if df in _T95_TABLE:
        return _T95_TABLE[df]
    if df < 1:
        return None
    if df > 120:
        return _T95_Z_FALLBACK
    # Between unlisted df (e.g. 31-39): nearest listed df at or below,
    # which is always a slightly more conservative (larger) t-value than
    # the true one - fine for a reported CI, never overstates precision.
    below = max(k for k in _T95_TABLE if k <= df)
    return _T95_TABLE[below]


def mean_ci95(values):
    """Returns (mean, ci95_lower, ci95_upper) for a list of numeric
    values using the t-distribution (appropriate for the trial counts
    this framework works with, rather than a large-sample z-interval).
    Needs at least 2 values to report an interval; with exactly 1 value
    the interval collapses to that single point; with 0, everything is
    None."""
    n = len(values)
    if n == 0:
        return None, None, None
    mean = statistics.mean(values)
    if n == 1:
        return mean, mean, mean
    stdev = statistics.stdev(values)
    if stdev == 0:
        return mean, mean, mean
    margin = _t95(n - 1) * (stdev / math.sqrt(n))
    return mean, mean - margin, mean + margin


def aggregate_metric(values):
    """Aggregates one metric's per-trial values (None entries dropped -
    e.g. avg_response_time_s/avg_formation_error are None on trials with
    no qualifying samples) into mean/median/stdev/min/max/95% CI."""
    clean = [v for v in values if v is not None]
    n = len(clean)
    if n == 0:
        return {"n": 0, **{s: None for s in STAT_SUFFIXES}}
    mean, lo, hi = mean_ci95(clean)
    return {
        "n": n,
        "mean": round(mean, 4),
        "median": round(statistics.median(clean), 4),
        "stdev": round(statistics.stdev(clean), 4) if n >= 2 else 0.0,
        "min": round(min(clean), 4),
        "max": round(max(clean), 4),
        "ci95_lower": round(lo, 4) if lo is not None else None,
        "ci95_upper": round(hi, 4) if hi is not None else None,
    }


def build_run_matrix(scenario_names, num_trials, base_seed, seed_mode="sequential"):
    """Automatic scenario generation: derives the full list of
    {scenario, trial, seed} runs to execute from just the scenario names
    (read straight off the config) and the requested trial count - no
    per-scenario or per-trial listing is ever written out by hand.

    "sequential" reproduces pre-Task-16 behavior: seed = base_seed +
    (trial - 1), shared across every scenario, so trial N is the same
    seed everywhere. "random" draws an independent seed per (scenario,
    trial) from a single random.Random(base_seed) master stream, so nothing
    is shared between scenarios - both modes are fully determined by
    (scenario_names, num_trials, base_seed, seed_mode), so the same inputs
    always regenerate the same matrix.
    """
    matrix = []
    if seed_mode == "sequential":
        for scenario_name in scenario_names:
            for trial in range(1, num_trials + 1):
                matrix.append({"scenario": scenario_name, "trial": trial,
                                "seed": base_seed + (trial - 1)})
    elif seed_mode == "random":
        master_rng = random.Random(base_seed)
        for scenario_name in scenario_names:
            for trial in range(1, num_trials + 1):
                matrix.append({"scenario": scenario_name, "trial": trial,
                                "seed": master_rng.randint(0, 2**31 - 1)})
    else:
        raise ValueError(f"Unknown seed_mode: {seed_mode!r} (expected 'sequential' or 'random')")
    return matrix


def run_and_save(config, scenario_name, run_number, seed, logs_dir, save_step_log=True):
    run_config = copy.deepcopy(config)
    run_config["sim"]["seed"] = seed

    sim = Simulation(run_config, scenario_name)
    metrics = sim.run()

    out_path = None
    if save_step_log:
        out_path = os.path.join(logs_dir, f"{scenario_name}_run{run_number}.csv")
        if sim.log_rows:
            os.makedirs(logs_dir, exist_ok=True)
            fieldnames = list(sim.log_rows[0].keys())
            with open(out_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(sim.log_rows)

    return sim, metrics, out_path


def run_level_row(config, scenario_name, run_number, seed, sim, metrics):
    scn = config["scenarios"][scenario_name]
    collision_risk_count = sum(1 for r in sim.log_rows if r.get("collision_risk_flag"))
    return {
        "scenario": scenario_name,
        "run_number": run_number,
        "fusion_mode": metrics["fusion_mode"],
        "false_positive_rate": scn.get("false_positive_rate", 0.0),
        "false_negative_rate": scn.get("false_negative_rate", 0.0),
        "noise_level": scn.get("position_noise_std", 0.0),
        "latency_steps": scn.get("latency_steps", 0),
        "dropout_probability": scn.get("dropout_prob", 0.0),
        "confidence_error_level": scn.get("confidence_error_level", 0.0),
        "collision_risk_count": collision_risk_count,
        "unnecessary_avoidance_count": metrics["unnecessary_avoidance_count"],
        "missed_response_count": metrics["missed_response_count"],
        "fusion_recovery_count": metrics["fusion_recovery_count"],
        "mission_success": "Yes" if metrics["mission_success"] else "No",
        "avg_response_time_s": metrics["avg_response_time_s"],
        "total_near_misses": metrics["near_miss_count"],
        "avg_formation_error": metrics["avg_formation_error"],
        "seed": seed,
    }


# Metrics that must always be a real number (never None/NaN) - unlike
# avg_response_time_s/avg_formation_error, which are legitimately None
# when a trial has no qualifying samples.
_CRITICAL_INT_METRICS = [
    "collision_risk_count", "unnecessary_avoidance_count",
    "missed_response_count", "fusion_recovery_count", "total_near_misses",
]


def _is_nan(v):
    return isinstance(v, float) and math.isnan(v)


def check_run_integrity(config_before_hash, config, scenario_name, run_number, seed,
                         sim, metrics, row, out_path, save_step_log, in_run_collision):
    """Task 10: per-run integrity checks. Returns a list of short failure
    codes (empty list = run is clean)."""
    failures = []

    if save_step_log:
        if not out_path or not os.path.exists(out_path):
            failures.append(("output_missing", "Step log output file was not created"))
        elif os.path.getsize(out_path) == 0:
            failures.append(("output_empty", "Step log output file is empty"))
    if not sim.log_rows:
        failures.append(("output_empty", "Simulation generated no log rows"))

    # no output file is overwritten *within this run* - i.e. two entries in
    # this run's own matrix never target the same path (a real collision/bug).
    # Reusing a directory from a *previous* invocation is normal and not
    # flagged here.
    if in_run_collision:
        failures.append(("output_overwritten", "Step log output file collided with another run"))

    # required columns exist
    missing_cols = set(RUN_FIELDNAMES) - set(row)
    if missing_cols:
        failures.append(("missing_columns", f"Missing columns: {','.join(sorted(missing_cols))}"))

    # no NaN in critical metrics (None is fine for the two averages, NaN never is)
    for m in _CRITICAL_INT_METRICS:
        if row.get(m) is None or _is_nan(row.get(m)):
            failures.append(("bad_metric", f"Metric {m} contains NaN or None"))
    for m in ("avg_response_time_s", "avg_formation_error"):
        if _is_nan(row.get(m)):
            failures.append(("bad_metric", f"Metric {m} contains NaN"))

    # simulation completed
    if not metrics.get("steps_run"):
        failures.append(("simulation_not_completed", "steps_run is missing or 0"))

    # mission status is recorded
    if row.get("mission_success") not in ("Yes", "No"):
        failures.append(("mission_status_missing", "mission_success is not Yes or No"))

    # random seed is recorded
    if row.get("seed") != seed:
        failures.append(("seed_not_recorded", f"Recorded seed {row.get('seed')} does not match requested seed {seed}"))

    # configuration is copied (run_config must never mutate the shared config)
    if _config_hash(config) != config_before_hash:
        failures.append(("config_mutated", "The shared configuration dict was mutated during execution"))

    return failures


def aggregate_scenario(scenario_name, rows):
    """Automatic result aggregation: turns every run-level row for one
    scenario into a single scenario-level summary row - mission success
    rate (with its own 95% CI, treating each trial as a 0/1 Bernoulli
    outcome) plus mean/median/stdev/min/max/95% CI for every metric in
    AGGREGATE_METRICS."""
    n = len(rows)
    successes = [1.0 if r["mission_success"] == "Yes" else 0.0 for r in rows]
    success_mean, success_lo, success_hi = mean_ci95(successes)

    summary = {
        "scenario": scenario_name,
        "fusion_mode": rows[0]["fusion_mode"] if rows else None,
        "num_trials": n,
        "mission_success_rate": round(success_mean, 4) if success_mean is not None else None,
        "mission_success_rate_ci95_lower": round(max(0.0, success_lo), 4) if success_lo is not None else None,
        "mission_success_rate_ci95_upper": round(min(1.0, success_hi), 4) if success_hi is not None else None,
    }
    for metric in AGGREGATE_METRICS:
        stats = aggregate_metric([r[metric] for r in rows])
        for suffix in STAT_SUFFIXES:
            summary[f"{metric}_{suffix}"] = stats[suffix]
    return summary


def scenario_summary_fieldnames():
    fieldnames = ["scenario", "fusion_mode", "num_trials", "mission_success_rate",
                  "mission_success_rate_ci95_lower", "mission_success_rate_ci95_upper"]
    for metric in AGGREGATE_METRICS:
        fieldnames.extend(f"{metric}_{suffix}" for suffix in STAT_SUFFIXES)
    return fieldnames


def _config_hash(config):
    """Short, stable fingerprint of the exact config used, so a result
    set can be checked against config drift without embedding the whole
    (potentially large) config file in the metadata."""
    payload = json.dumps(config, sort_keys=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]


def write_metadata(path, *, config_path, config, num_trials, base_seed, seed_mode,
                    scenario_names, run_matrix, total_runs, save_step_logs,
                    started_at, wall_clock_seconds):
    """Experiment metadata logging: records exactly how this experiment
    was produced - config identity, trial/seed strategy, the per-scenario
    seeds actually used, timing, and the environment it ran on - so any
    results file can be traced back to the run that generated it."""
    metadata = {
        "started_at": started_at,
        "wall_clock_seconds": round(wall_clock_seconds, 3),
        "config_path": config_path,
        "config_sha256_16": _config_hash(config),
        "num_trials": num_trials,
        "recommended_trials": {"initial_analysis_minimum": 20, "paper_ready": "50-100"},
        "base_seed": base_seed,
        "seed_mode": seed_mode,
        "scenario_names": scenario_names,
        "num_scenarios": len(scenario_names),
        "total_runs": total_runs,
        "save_step_logs": save_step_logs,
        "seeds_by_scenario": {
            scenario_name: [r["seed"] for r in run_matrix if r["scenario"] == scenario_name]
            for scenario_name in scenario_names
        },
        "environment": {
            "python_version": platform.python_version(),
            "platform": platform.platform(),
        },
    }
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, "w") as f:
        json.dump(metadata, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Monte Carlo experiment runner: repeats every scenario for N trials, "
                     "saving run-level results, a scenario-level statistical summary, and "
                     "experiment metadata")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--trials", "--runs", dest="trials", type=int, default=20,
                         help="Trials per scenario. Recommended: >= 20 for initial analysis, "
                              "50-100 for paper-ready final results if computationally "
                              "feasible (default: 20)")
    parser.add_argument("--scenario", default=None, help="Only run this one scenario")
    parser.add_argument("--base-seed", type=int, default=None,
                         help="Master seed the run matrix is derived from "
                              "(default: config['sim']['seed'])")
    parser.add_argument("--seed-mode", choices=["sequential", "random"], default="sequential",
                         help="'sequential' (default): seed = base_seed + (trial - 1), shared "
                              "across scenarios, matching pre-Task-16 behavior. 'random': an "
                              "independent seed per (scenario, trial), drawn from a single "
                              "random.Random(base_seed) stream")
    parser.add_argument("--logs-dir", default=os.path.join(_ROOT_DIR, "logs"))
    parser.add_argument("--results-dir", default=os.path.join(_ROOT_DIR, "results"))
    parser.add_argument("--combined-log", default=os.path.join(_ROOT_DIR, "logs", "simulation_log.csv"),
                         help="Combined log covering trial 1 of every scenario")
    parser.add_argument("--run-level-output", "--summary-output", dest="run_level_output",
                         default=os.path.join(_ROOT_DIR, "results", "results_summary.csv"),
                         help="Run-level results, one row per (scenario, trial) - unchanged "
                              "schema, still what generate_plots.py/metrics_analysis.py expect")
    parser.add_argument("--scenario-summary-output", default=os.path.join(_ROOT_DIR, "results", "scenario_summary.csv"),
                         help="Scenario-level statistical summary (mean/median/stdev/min/max/"
                              "95% CI per metric, aggregated across trials)")
    parser.add_argument("--metadata-output", default=os.path.join(_ROOT_DIR, "results", "experiment_metadata.json"),
                         help="Experiment metadata: config identity, trial/seed strategy, "
                              "per-scenario seeds used, timing, environment")
    parser.add_argument("--failed-runs-output", default=os.path.join(_ROOT_DIR, "results", "failed_runs.csv"),
                         help="Runs that failed any integrity check (output file/columns/metrics/"
                              "seed/config/mission-status/overwrite) - one row per failed run")
    parser.add_argument("--skip-step-logs", action="store_true",
                         help="Don't write the per-trial step-level CSV log to --logs-dir - "
                              "keeps only the aggregated results. Recommended at 50-100 trials, "
                              "where per-trial logs dominate disk/time cost")
    args = parser.parse_args()

    if args.trials < 1:
        sys.exit(f"--trials must be >= 1 (got {args.trials})")

    os.makedirs(args.results_dir, exist_ok=True)
    if not args.skip_step_logs:
        os.makedirs(args.logs_dir, exist_ok=True)

    with open(args.config) as f:
        config = json.load(f)

    if args.scenario and args.scenario not in config["scenarios"]:
        available = ", ".join(config["scenarios"].keys())
        sys.exit(f"Unknown scenario '{args.scenario}'. Available: {available}")

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    base_seed = args.base_seed if args.base_seed is not None else config["sim"]["seed"]

    run_matrix = build_run_matrix(scenario_names, args.trials, base_seed, args.seed_mode)

    started_at = datetime.now(timezone.utc).isoformat()
    start_time = time.monotonic()

    combined_rows = []
    run_level_rows = []
    scenario_rows_by_name = {name: [] for name in scenario_names}
    failed_runs = []
    seen_out_paths = set()

    print(f"Running {len(scenario_names)} scenarios x {args.trials} trials each "
          f"({len(run_matrix)} total runs) | base_seed={base_seed} seed_mode={args.seed_mode}\n")

    for entry in run_matrix:
        scenario_name, trial, seed = entry["scenario"], entry["trial"], entry["seed"]
        config_before_hash = _config_hash(config)
        sim, metrics, out_path = run_and_save(
            config, scenario_name, trial, seed, args.logs_dir,
            save_step_log=not args.skip_step_logs)

        in_run_collision = out_path is not None and out_path in seen_out_paths
        if out_path is not None:
            seen_out_paths.add(out_path)

        if trial == 1:
            combined_rows.extend(sim.log_rows)

        row = run_level_row(config, scenario_name, trial, seed, sim, metrics)
        run_level_rows.append(row)
        scenario_rows_by_name[scenario_name].append(row)

        checks_failed = check_run_integrity(
            config_before_hash, config, scenario_name, trial, seed, sim, metrics, row,
            out_path, not args.skip_step_logs, in_run_collision)
        for failure_type, error_msg in checks_failed:
            failed_runs.append({
                "scenario_id": scenario_name,
                "trial_number": trial,
                "random_seed": seed,
                "failure_type": failure_type,
                "error_message": error_msg,
                "output_directory": os.path.dirname(out_path) if out_path else args.logs_dir,
                "rerun_status": "PENDING"
            })

        log_note = out_path if out_path else "(step log skipped)"
        print(f"[{scenario_name} trial {trial}/{args.trials} | seed={seed}] -> {log_note}")
        print(f"    mission_success={metrics['mission_success']}  "
              f"reached_goal={metrics['uavs_reached_goal']}/{metrics['num_uavs']}  "
              f"collisions={metrics['collision_count']}  "
              f"fusion_mode={metrics['fusion_mode']}  "
              f"fusion_recovery={metrics['fusion_recovery_count']}")
        if checks_failed:
            fail_str = ", ".join(f[0] for f in checks_failed)
            print(f"    INTEGRITY CHECK FAILED: {fail_str}")

    wall_clock_seconds = time.monotonic() - start_time

    # --- combined log (trial 1 of every scenario) --------------------------
    if combined_rows:
        fieldnames = list(combined_rows[0].keys())
        combined_dir = os.path.dirname(args.combined_log)
        if combined_dir:
            os.makedirs(combined_dir, exist_ok=True)
        with open(args.combined_log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(combined_rows)
        print(f"\nCombined log (trial 1 of every scenario) -> {args.combined_log}")

    if not args.skip_step_logs:
        print(f"Saved {len(run_matrix)} per-trial step-level CSV logs to {args.logs_dir}/")

    # --- run-level results ---------------------------------------------
    run_level_dir = os.path.dirname(args.run_level_output)
    if run_level_dir:
        os.makedirs(run_level_dir, exist_ok=True)
    with open(args.run_level_output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RUN_FIELDNAMES)
        writer.writeheader()
        writer.writerows(run_level_rows)
    print(f"Run-level results ({len(run_level_rows)} rows) -> {args.run_level_output}")

    # --- scenario-level summary (automatic aggregation) -----------------
    scenario_summary_rows = [
        aggregate_scenario(name, scenario_rows_by_name[name]) for name in scenario_names
    ]
    scenario_summary_dir = os.path.dirname(args.scenario_summary_output)
    if scenario_summary_dir:
        os.makedirs(scenario_summary_dir, exist_ok=True)
    with open(args.scenario_summary_output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=scenario_summary_fieldnames())
        writer.writeheader()
        writer.writerows(scenario_summary_rows)
    print(f"Scenario-level summary ({len(scenario_summary_rows)} rows) -> {args.scenario_summary_output}")

    # --- failed runs (integrity-check failures) --------------------------
    failed_runs_dir = os.path.dirname(args.failed_runs_output)
    if failed_runs_dir:
        os.makedirs(failed_runs_dir, exist_ok=True)
    with open(args.failed_runs_output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "scenario_id", "trial_number", "random_seed", "failure_type",
            "error_message", "output_directory", "rerun_status"
        ])
        writer.writeheader()
        writer.writerows(failed_runs)
    if failed_runs:
        print(f"WARNING: {len(failed_runs)} run(s) failed integrity checks -> {args.failed_runs_output}")
    else:
        print(f"All runs passed integrity checks -> {args.failed_runs_output} (empty)")

    # --- experiment metadata --------------------------------------------
    write_metadata(
        args.metadata_output, config_path=args.config, config=config,
        num_trials=args.trials, base_seed=base_seed, seed_mode=args.seed_mode,
        scenario_names=scenario_names, run_matrix=run_matrix, total_runs=len(run_matrix),
        save_step_logs=not args.skip_step_logs, started_at=started_at,
        wall_clock_seconds=wall_clock_seconds)
    print(f"Experiment metadata -> {args.metadata_output}")

    print(f"\nDone in {wall_clock_seconds:.1f}s.")
    for row in scenario_summary_rows:
        ci_lo = row["mission_success_rate_ci95_lower"]
        ci_hi = row["mission_success_rate_ci95_upper"]
        ci_str = f"[{ci_lo:.2f}, {ci_hi:.2f}]" if ci_lo is not None else "N/A"
        print(f"  {row['scenario']}: success_rate={row['mission_success_rate']:.0%} "
              f"95% CI={ci_str}  n={row['num_trials']}")


if __name__ == "__main__":
    main()