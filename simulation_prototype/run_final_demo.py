"""
run_final_demo.py

Task 17: one command that reproduces a small, fast, representative slice
of this project's pipeline - for sanity-checking a setup (new machine,
new environment, CI) without paying for the full experiment matrix
(see run_final_experiments.py for that).

Reuses run_final_simulations.py's own run_group()/write_*() helpers -
the exact same trial execution, row schema, CSV writers, and metadata
format the full run uses - so a demo result is directly comparable to,
and literally a subset of, what run_final_experiments.py produces.
Nothing about how a trial is run or aggregated is reimplemented here.

DEMO_SCENARIOS is a small slice of scenario sets this project already
curates elsewhere, not a fresh hand-picked list:
  - "baseline"              - core sanity scenario (every scenario summary
                               script leads with this one)
  - "naive_fusion"          - one of run_final_simulations.py's own
                               FUSION_COMPARISON_SCENARIOS
  - "high_dropout"          - a degraded-sensor core scenario
  - "communication_outage"  - the distributed-architecture scenario from
                               build_experiment_matrix.py's CORE_SCENARIOS
Together they touch the core path, the fusion-comparison path, and the
distributed/communication path in a handful of trials.

Usage:
    python run_final_demo.py
    python run_final_demo.py --trials 5
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from run_final_simulations import (
    run_group, write_raw_log, write_run_level_csv, write_scenario_summary_csv,
)
from experiments.run_experiments import write_metadata

DEMO_SCENARIOS = ["baseline", "naive_fusion", "high_dropout", "communication_outage", "simultaneous_sensor_failures"]
DEMO_TRIALS_DEFAULT = 3


def main():
    parser = argparse.ArgumentParser(description="Small representative demo run")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--trials", type=int, default=DEMO_TRIALS_DEFAULT,
                         help=f"Trials per demo scenario (default: {DEMO_TRIALS_DEFAULT})")
    parser.add_argument("--base-seed", type=int, default=None,
                         help="Master seed (default: config['sim']['seed'])")
    parser.add_argument("--seed-mode", choices=["sequential", "random"], default="sequential")
    parser.add_argument("--skip-step-logs", action="store_true",
                         help="Skip writing per-trial step-level CSV logs (keeps aggregates only)")
    parser.add_argument("--output-dir", default=os.path.join(_ROOT_DIR, "results", "demo"))
    args = parser.parse_args()

    if args.trials < 1:
        sys.exit(f"--trials must be >= 1 (got {args.trials})")

    with open(args.config) as f:
        config = json.load(f)

    missing = [s for s in DEMO_SCENARIOS if s not in config["scenarios"]]
    if missing:
        sys.exit(f"DEMO_SCENARIOS references unknown scenario(s): {missing}")

    base_seed = args.base_seed if args.base_seed is not None else config["sim"]["seed"]
    out_dir = args.output_dir
    logs_dir = os.path.join(out_dir, "raw_logs")
    os.makedirs(out_dir, exist_ok=True)

    started_at = datetime.now(timezone.utc).isoformat()
    t0 = time.monotonic()

    rows, combined_rows, statuses = run_group(
        config, DEMO_SCENARIOS, args.trials, base_seed, args.seed_mode,
        logs_dir, save_step_logs=not args.skip_step_logs, group_label="demo")

    wall_clock = time.monotonic() - t0

    write_raw_log(combined_rows, os.path.join(out_dir, "demo_combined_log.csv"))
    write_run_level_csv(rows, os.path.join(out_dir, "run_level_results.csv"))
    by_scenario = {s: [r for r in rows if r["scenario"] == s] for s in DEMO_SCENARIOS}
    summary_rows = write_scenario_summary_csv(
        by_scenario, os.path.join(out_dir, "scenario_summary.csv"))

    num_failed = sum(1 for s in statuses if s["status"] == "FAIL")
    overall_status = "PASS" if num_failed == 0 else "FAIL"

    write_metadata(
        os.path.join(out_dir, "run_metadata.json"), config_path=args.config, config=config,
        num_trials=args.trials, base_seed=base_seed, seed_mode=args.seed_mode,
        scenario_names=DEMO_SCENARIOS,
        run_matrix=[{"scenario": s["scenario"], "trial": s["trial"], "seed": s["seed"]}
                    for s in statuses],
        total_runs=len(statuses), save_step_logs=not args.skip_step_logs,
        started_at=started_at, wall_clock_seconds=wall_clock)

    print(f"\n=== DEMO DONE in {wall_clock:.1f}s | overall_status={overall_status} "
          f"({num_failed}/{len(statuses)} trials failed) ===")
    for row in summary_rows:
        print(f"  {row['scenario']}: n={row['num_trials']} "
              f"success_rate={row['mission_success_rate']} status={row['all_trials_passed']}")
    print(f"\nDemo results -> {out_dir}/ "
          f"(run_level_results.csv, scenario_summary.csv, demo_combined_log.csv, run_metadata.json)")
    if not args.skip_step_logs:
        print(f"Per-trial step logs -> {logs_dir}/")

    return 1 if overall_status == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main())
