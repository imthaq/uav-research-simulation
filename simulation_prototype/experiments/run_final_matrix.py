"""
Task 14: Final Experiment Matrix Runner

Reads `results/final_experiment_matrix.csv` (created in Task 12) and executes
every scenario listed, saving all required artifacts (raw logs, configurations, 
random seeds, run-level metrics, scenario summaries, runtime, PASS/FAIL statuses).
"""

import argparse
import copy
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from experiments.run_experiments import (
    run_and_save, run_level_row, check_run_integrity, _config_hash, RUN_FIELDNAMES,
    aggregate_scenario, scenario_summary_fieldnames, write_metadata
)

def parse_matrix_csv(csv_path):
    plan = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Parse trial count
            trials = int(row["trial_count"])
            
            # Parse seed range (e.g. "42-61")
            seed_range_str = row["seed_range"].strip()
            if "-" in seed_range_str:
                base_seed = int(seed_range_str.split("-")[0])
            else:
                base_seed = int(seed_range_str)
                
            # Parse output directory
            out_dir = row["output_directory"].split(" ")[0].strip()
            
            plan.append({
                "scenario_id": row["scenario_id"].strip(),
                "trials": trials,
                "base_seed": base_seed,
                "out_dir": out_dir
            })
    return plan


def main():
    parser = argparse.ArgumentParser(description="Runs the frozen final experiment matrix.")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--matrix-csv", default=os.path.join(_ROOT_DIR, "results", "final_experiment_matrix.csv"))
    parser.add_argument("--skip-step-logs", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.matrix_csv):
        print(f"Error: Matrix CSV not found at {args.matrix_csv}")
        sys.exit(1)

    with open(args.config) as f:
        base_config = json.load(f)

    plan = parse_matrix_csv(args.matrix_csv)
    total_runs = sum(p["trials"] for p in plan)
    print(f"Executing {len(plan)} scenario configurations ({total_runs} total trials).")

    all_run_rows = []
    failed_runs = []
    seen_out_paths = set()
    
    overall_t0 = time.monotonic()

    for p in plan:
        scenario_id = p["scenario_id"]
        num_trials = p["trials"]
        base_seed = p["base_seed"]
        out_dir = p["out_dir"]
        
        print(f"\n--- Scenario: {scenario_id} ({num_trials} trials, base_seed: {base_seed}) ---")
        
        if scenario_id not in base_config["scenarios"]:
            print(f"ERROR: Scenario {scenario_id} not found in config! Skipping.")
            continue
            
        out_dir_abs = os.path.join(_ROOT_DIR, out_dir)
        os.makedirs(out_dir_abs, exist_ok=True)
        
        run_config = copy.deepcopy(base_config)
        config_before_hash = _config_hash(run_config)
        
        for trial in range(1, num_trials + 1):
            seed = base_seed + (trial - 1)
            t0 = time.monotonic()
            
            try:
                sim, metrics, out_path = run_and_save(
                    run_config, scenario_id, trial, seed, out_dir_abs, save_step_log=not args.skip_step_logs
                )
                runtime = time.monotonic() - t0
                
                in_run_collision = out_path is not None and out_path in seen_out_paths
                if out_path is not None:
                    seen_out_paths.add(out_path)
                
                row = run_level_row(run_config, scenario_id, trial, seed, sim, metrics)
                row["runtime_seconds"] = round(runtime, 4)
                row["status"] = "PASS"
                all_run_rows.append(row)
                
                checks_failed = check_run_integrity(
                    config_before_hash, run_config, scenario_id, trial, seed, sim, metrics, row,
                    out_path, not args.skip_step_logs, in_run_collision
                )
                
                if checks_failed:
                    for f_type, f_msg in checks_failed:
                        failed_runs.append({
                            "scenario_id": scenario_id,
                            "trial_number": trial,
                            "random_seed": seed,
                            "failure_type": f_type,
                            "error_message": f_msg,
                            "output_directory": out_dir_abs,
                            "rerun_status": "PENDING"
                        })
                    fail_str = ", ".join(f[0] for f in checks_failed)
                    print(f"  Trial {trial} (seed {seed}): INTEGRITY FAILED ({fail_str})")
                else:
                    print(f"  Trial {trial} (seed {seed}): PASS ({runtime:.2f}s)")
                    
            except Exception as e:
                import traceback
                print(f"  Trial {trial} (seed {seed}): FATAL EXCEPTION -> {e}")
                failed_runs.append({
                    "scenario_id": scenario_id,
                    "trial_number": trial,
                    "random_seed": seed,
                    "failure_type": "exception",
                    "error_message": traceback.format_exc().strip().splitlines()[-1],
                    "output_directory": out_dir_abs,
                    "rerun_status": "PENDING"
                })

    overall_time = time.monotonic() - overall_t0

    # Save run_level_results
    results_dir = os.path.join(_ROOT_DIR, "results")
    os.makedirs(results_dir, exist_ok=True)
    run_level_path = os.path.join(results_dir, "run_level_results.csv")
    fieldnames = RUN_FIELDNAMES + ["runtime_seconds", "status"]
    
    with open(run_level_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_run_rows)
        
    failed_runs_path = os.path.join(results_dir, "failed_runs.csv")
    with open(failed_runs_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "scenario_id", "trial_number", "random_seed", "failure_type",
            "error_message", "output_directory", "rerun_status"
        ])
        writer.writeheader()
        writer.writerows(failed_runs)

    # Save scenario summaries
    scenario_rows_by_name = {}
    for row in all_run_rows:
        scenario_rows_by_name.setdefault(row["scenario"], []).append(row)
    
    scenario_summary_rows = [
        aggregate_scenario(p["scenario_id"], scenario_rows_by_name[p["scenario_id"]]) 
        for p in plan if p["scenario_id"] in scenario_rows_by_name
    ]
    summary_path = os.path.join(results_dir, "scenario_summary.csv")
    with open(summary_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=scenario_summary_fieldnames())
        writer.writeheader()
        writer.writerows(scenario_summary_rows)

    # Save metadata and run configurations
    metadata_path = os.path.join(results_dir, "experiment_metadata.json")
    run_matrix = []
    for p in plan:
        for t in range(1, p["trials"] + 1):
            run_matrix.append({"scenario": p["scenario_id"], "trial": t, "seed": p["base_seed"] + (t - 1)})
            
    write_metadata(
        metadata_path, config_path=args.config, config=base_config,
        num_trials=max((p["trials"] for p in plan), default=0), 
        base_seed=plan[0]["base_seed"] if plan else 42, 
        seed_mode="sequential",
        scenario_names=[p["scenario_id"] for p in plan],
        run_matrix=run_matrix, total_runs=total_runs, 
        save_step_logs=not args.skip_step_logs,
        started_at=datetime.now(timezone.utc).isoformat(), wall_clock_seconds=overall_time
    )

    print(f"\nMatrix completed in {overall_time:.1f}s.")
    print(f"Results saved to: {run_level_path}")
    print(f"Scenario summary: {summary_path}")
    print(f"Experiment metadata: {metadata_path}")
    print(f"Failed runs log ({len(failed_runs)} failures): {failed_runs_path}")

if __name__ == "__main__":
    main()
