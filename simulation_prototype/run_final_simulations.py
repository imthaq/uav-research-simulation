"""
run_final_simulations.py

Task 9: runs the final, repeated Monte Carlo simulations for this project
and saves everything needed to trust and reproduce the results:

  - random seed             (per trial, plus the master --base-seed)
  - configuration            (the exact config used, hashed + copied verbatim)
  - raw log                  (per-trial step-level CSV, plus a combined log)
  - summary                  (per-scenario/per-comparison aggregated stats)
  - runtime                  (wall-clock time, per trial and totals)
  - PASS/FAIL status         (did the trial execute cleanly and produce a
                               usable metrics dict - an execution-health
                               check, kept separate from mission_success,
                               which is a *research* outcome, not a
                               validity check)

This builds directly on run_experiments.py (imported, not reimplemented)
and adds the two-tier trial plan Task 9 asks for:

  1. Every "core" scenario (every scenario defined in simulation_config.json)
     gets at least CORE_TRIALS (20) trials.
  2. The scenarios that most directly compare fusion modes - see
     FUSION_COMPARISON_SCENARIOS below - additionally get bumped to
     FUSION_COMPARISON_TRIALS (50) trials "when runtime permits": before
     committing to 50, this script times a small warm-up batch and
     extrapolates the total cost. If projected runtime exceeds
     --time-budget-seconds, those scenarios are quietly capped back down to
     CORE_TRIALS instead, and the fallback is recorded in the metadata/status
     output (never fails silently).

Usage:
    python run_final_simulations.py
    python run_final_simulations.py --time-budget-seconds 600
    python run_final_simulations.py --core-trials 20 --comparison-trials 50
"""

import argparse
import csv
import json
import os
import shutil
import sys
import time
import traceback
from datetime import datetime, timezone

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from experiments.run_experiments import (
    build_run_matrix, run_and_save, run_level_row, aggregate_scenario,
    scenario_summary_fieldnames, RUN_FIELDNAMES, _config_hash,
)

# The scenarios that most directly compare fusion modes against each other
# (either the same setup under a different fusion_mode, or a scenario
# purpose-built to stress-test one fusion strategy vs. another). This is
# the "most important fusion comparisons" set that gets bumped to 50
# trials when the runtime budget allows.
FUSION_COMPARISON_SCENARIOS = [
    "naive_fusion",
    "trust_weighted_fusion",
    "faulty_sensor_naive_fusion",
    "faulty_sensor_confidence_weighted_fusion",
    "faulty_sensor_trust_weighted_fusion_fixed",
    "faulty_sensor_trust_weighted_fusion_dynamic",
    "faulty_sensor_covariance_weighted_fusion",
]

CORE_TRIALS_DEFAULT = 20
COMPARISON_TRIALS_DEFAULT = 50


def run_group(config, scenario_names, num_trials, base_seed, seed_mode,
               logs_dir, save_step_logs, group_label):
    """Runs every (scenario, trial) in this group, using run_experiments.py's
    own run_and_save/run_level_row so the row schema and per-trial CSV
    logs are identical to what run_experiments.py itself produces.

    Wraps every trial in try/except so a single failing trial is recorded
    as PASS/FAIL rather than crashing the whole experiment - and so a
    scenario's PASS/FAIL status can be reported even if some trials in it
    fail.
    """
    run_matrix = build_run_matrix(scenario_names, num_trials, base_seed, seed_mode)
    rows = []
    combined_rows = []
    trial_statuses = []

    print(f"\n=== {group_label}: {len(scenario_names)} scenarios x {num_trials} trials "
          f"({len(run_matrix)} total runs) ===")

    for entry in run_matrix:
        scenario_name, trial, seed = entry["scenario"], entry["trial"], entry["seed"]
        t0 = time.monotonic()
        status, error = "PASS", None
        row = None
        try:
            sim, metrics, out_path = run_and_save(
                config, scenario_name, trial, seed, logs_dir, save_step_log=save_step_logs)
            if trial == 1:
                combined_rows.extend(sim.log_rows)
            row = run_level_row(config, scenario_name, trial, seed, sim, metrics)
        except Exception:
            status = "FAIL"
            error = traceback.format_exc(limit=3)
            print(f"  [FAIL] {scenario_name} trial {trial} (seed={seed}): {error.strip().splitlines()[-1]}")
        runtime_s = time.monotonic() - t0

        if row is not None:
            row["group"] = group_label
            row["runtime_seconds"] = round(runtime_s, 4)
            row["status"] = status
            rows.append(row)
        trial_statuses.append({
            "scenario": scenario_name, "trial": trial, "seed": seed,
            "status": status, "runtime_seconds": round(runtime_s, 4),
            "error": error,
        })

    return rows, combined_rows, trial_statuses


def estimate_feasible_trials(config, scenario_names, base_seed, seed_mode,
                              target_trials, time_budget_seconds, warmup_trials=3):
    """Times a small warm-up batch of trials across `scenario_names` and
    extrapolates to decide whether `target_trials` trials for all of them
    fits inside `time_budget_seconds`. Falls back to CORE_TRIALS_DEFAULT
    if not. Returns (trials_to_use, estimate_info_dict)."""
    warmup_trials = min(warmup_trials, target_trials)
    warmup_matrix = build_run_matrix(scenario_names, warmup_trials, base_seed, seed_mode)

    t0 = time.monotonic()
    for entry in warmup_matrix:
        try:
            run_and_save(config, entry["scenario"], entry["trial"], entry["seed"],
                         logs_dir=os.path.join(_ROOT_DIR, "_warmup_tmp"), save_step_log=False)
        except Exception:
            pass  # warm-up is only for timing; real errors surface in the real run
    elapsed = time.monotonic() - t0
    per_trial = elapsed / max(1, len(warmup_matrix))

    projected_total = per_trial * len(scenario_names) * target_trials
    info = {
        "warmup_trials_run": len(warmup_matrix),
        "warmup_elapsed_seconds": round(elapsed, 3),
        "per_trial_seconds_estimate": round(per_trial, 5),
        "projected_seconds_at_target_trials": round(projected_total, 2),
        "time_budget_seconds": time_budget_seconds,
    }
    if projected_total <= time_budget_seconds:
        info["decision"] = f"within budget -> using {target_trials} trials"
        return target_trials, info
    else:
        info["decision"] = (f"projected {projected_total:.1f}s exceeds budget "
                             f"{time_budget_seconds}s -> falling back to {CORE_TRIALS_DEFAULT} trials")
        return CORE_TRIALS_DEFAULT, info


def write_raw_log(rows, path):
    if not rows:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_run_level_csv(rows, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = RUN_FIELDNAMES + ["group", "runtime_seconds", "status"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_scenario_summary_csv(rows_by_scenario, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = scenario_summary_fieldnames() + ["all_trials_passed", "num_failed_trials"]
    summary_rows = []
    for scenario_name, rows in rows_by_scenario.items():
        if not rows:
            continue
        summary = aggregate_scenario(scenario_name, rows)
        num_failed = sum(1 for r in rows if r["status"] == "FAIL")
        summary["all_trials_passed"] = "PASS" if num_failed == 0 else "FAIL"
        summary["num_failed_trials"] = num_failed
        summary_rows.append(summary)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)
    return summary_rows


def main():
    parser = argparse.ArgumentParser(description="Task 9: run final repeated simulations")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--core-trials", type=int, default=CORE_TRIALS_DEFAULT,
                         help=f"Trials for every core scenario (default: {CORE_TRIALS_DEFAULT})")
    parser.add_argument("--comparison-trials", type=int, default=COMPARISON_TRIALS_DEFAULT,
                         help=f"Target trials for the key fusion-comparison scenarios, used "
                              f"only if the runtime budget allows (default: {COMPARISON_TRIALS_DEFAULT})")
    parser.add_argument("--time-budget-seconds", type=float, default=1800.0,
                         help="Max projected seconds allowed for the comparison-trial tier "
                              "before falling back to --core-trials (default: 1800 = 30 min)")
    parser.add_argument("--base-seed", type=int, default=None,
                         help="Master seed (default: config['sim']['seed'])")
    parser.add_argument("--seed-mode", choices=["sequential", "random"], default="sequential")
    parser.add_argument("--skip-step-logs", action="store_true",
                         help="Skip writing per-trial step-level CSV logs (keeps aggregates only)")
    parser.add_argument("--output-dir", default=os.path.join(_ROOT_DIR, "results"))
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    base_seed = args.base_seed if args.base_seed is not None else config["sim"]["seed"]
    all_scenarios = list(config["scenarios"].keys())
    missing = [s for s in FUSION_COMPARISON_SCENARIOS if s not in config["scenarios"]]
    if missing:
        sys.exit(f"FUSION_COMPARISON_SCENARIOS references unknown scenario(s): {missing}")

    out_dir = args.output_dir
    logs_dir = os.path.join(out_dir, "raw_logs")
    os.makedirs(out_dir, exist_ok=True)

    started_at = datetime.now(timezone.utc).isoformat()
    overall_t0 = time.monotonic()

    # --- Tier 1: every core scenario, >= core_trials ------------------------
    core_rows, core_combined, core_statuses = run_group(
        config, all_scenarios, args.core_trials, base_seed, args.seed_mode,
        logs_dir, save_step_logs=not args.skip_step_logs, group_label="core")

    # --- Tier 2: fusion comparisons, up to comparison_trials "when runtime
    #     permits" (estimated from a warm-up batch) -------------------------
    comparison_trials, estimate_info = estimate_feasible_trials(
        config, FUSION_COMPARISON_SCENARIOS, base_seed, args.seed_mode,
        args.comparison_trials, args.time_budget_seconds)
    print(f"\nFusion-comparison trial budget check: {estimate_info['decision']}")
    shutil.rmtree(os.path.join(_ROOT_DIR, "_warmup_tmp"), ignore_errors=True)

    comparison_rows, comparison_combined, comparison_statuses = run_group(
        config, FUSION_COMPARISON_SCENARIOS, comparison_trials, base_seed, args.seed_mode,
        logs_dir, save_step_logs=not args.skip_step_logs, group_label="fusion_comparison")

    overall_wall_clock = time.monotonic() - overall_t0

    # --- Save raw logs --------------------------------------------------
    write_raw_log(core_combined, os.path.join(out_dir, "core_combined_log.csv"))
    write_raw_log(comparison_combined, os.path.join(out_dir, "fusion_comparison_combined_log.csv"))
    print(f"\nRaw combined logs -> {out_dir}/core_combined_log.csv, "
          f"{out_dir}/fusion_comparison_combined_log.csv")
    if not args.skip_step_logs:
        print(f"Per-trial step-level CSV logs -> {logs_dir}/")

    # --- Save run-level results (with runtime + PASS/FAIL per trial) ----
    all_rows = core_rows + comparison_rows
    write_run_level_csv(all_rows, os.path.join(out_dir, "run_level_results.csv"))
    print(f"Run-level results ({len(all_rows)} rows) -> {out_dir}/run_level_results.csv")

    # --- Save scenario-level summaries (with PASS/FAIL per scenario) ----
    core_by_scenario = {s: [r for r in core_rows if r["scenario"] == s] for s in all_scenarios}
    comparison_by_scenario = {s: [r for r in comparison_rows if r["scenario"] == s]
                               for s in FUSION_COMPARISON_SCENARIOS}

    core_summary_rows = write_scenario_summary_csv(
        core_by_scenario, os.path.join(out_dir, "core_scenario_summary.csv"))
    comparison_summary_rows = write_scenario_summary_csv(
        comparison_by_scenario, os.path.join(out_dir, "fusion_comparison_summary.csv"))
    print(f"Scenario summaries -> {out_dir}/core_scenario_summary.csv, "
          f"{out_dir}/fusion_comparison_summary.csv")

    # --- Save the exact configuration used -------------------------------
    config_copy_path = os.path.join(out_dir, "configuration_used.json")
    shutil.copyfile(args.config, config_copy_path)
    print(f"Configuration snapshot -> {config_copy_path}")

    # --- Overall PASS/FAIL + seeds + runtime metadata ---------------------
    all_statuses = core_statuses + comparison_statuses
    num_failed = sum(1 for s in all_statuses if s["status"] == "FAIL")
    overall_status = "PASS" if num_failed == 0 else "FAIL"

    metadata = {
        "started_at": started_at,
        "overall_wall_clock_seconds": round(overall_wall_clock, 3),
        "overall_status": overall_status,
        "total_trials_run": len(all_statuses),
        "total_trials_failed": num_failed,
        "base_seed": base_seed,
        "seed_mode": args.seed_mode,
        "config_path": os.path.abspath(args.config),
        "config_sha256_16": _config_hash(config),
        "core": {
            "scenarios": all_scenarios,
            "num_scenarios": len(all_scenarios),
            "trials_per_scenario": args.core_trials,
            "seeds_by_scenario": {
                s: sorted({r["seed"] for r in core_rows if r["scenario"] == s}) for s in all_scenarios
            },
        },
        "fusion_comparison": {
            "scenarios": FUSION_COMPARISON_SCENARIOS,
            "num_scenarios": len(FUSION_COMPARISON_SCENARIOS),
            "requested_trials": args.comparison_trials,
            "trials_actually_used": comparison_trials,
            "budget_estimate": estimate_info,
            "seeds_by_scenario": {
                s: sorted({r["seed"] for r in comparison_rows if r["scenario"] == s})
                for s in FUSION_COMPARISON_SCENARIOS
            },
        },
        "pass_fail_definition": (
            "A trial is PASS if the simulation ran to completion and produced a "
            "usable metrics dict; FAIL if it raised an exception. This is an "
            "execution-health check, not a judgment of mission_success (which is "
            "a research outcome captured separately in the run-level/summary CSVs)."
        ),
    }
    metadata_path = os.path.join(out_dir, "run_metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"Run metadata (seed/config/runtime/PASS-FAIL) -> {metadata_path}")

    # Per-trial PASS/FAIL detail (including any error tracebacks), separate
    # from the metadata summary above so the summary file stays small.
    status_path = os.path.join(out_dir, "trial_status_detail.json")
    with open(status_path, "w") as f:
        json.dump({"trials": all_statuses}, f, indent=2)
    print(f"Per-trial PASS/FAIL detail -> {status_path}")

    print(f"\n=== DONE in {overall_wall_clock:.1f}s | overall_status={overall_status} "
          f"({num_failed}/{len(all_statuses)} trials failed) ===")
    for row in core_summary_rows[:5]:
        print(f"  [core] {row['scenario']}: n={row['num_trials']} "
              f"success_rate={row['mission_success_rate']} status={row['all_trials_passed']}")
    if len(core_summary_rows) > 5:
        print(f"  ... ({len(core_summary_rows) - 5} more core scenarios in core_scenario_summary.csv)")
    for row in comparison_summary_rows:
        print(f"  [fusion_comparison] {row['scenario']}: n={row['num_trials']} "
              f"success_rate={row['mission_success_rate']} status={row['all_trials_passed']}")

    return 1 if overall_status == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main())
