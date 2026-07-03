import argparse
import copy
import csv
import json
import os

from simple_swarm_sim import Simulation

SUMMARY_FIELDNAMES = [
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
]


def run_and_save(config, scenario_name, run_number, seed, logs_dir):
    run_config = copy.deepcopy(config)
    run_config["sim"]["seed"] = seed

    sim = Simulation(run_config, scenario_name)
    metrics = sim.run()

    out_path = os.path.join(logs_dir, f"{scenario_name}_run{run_number}.csv")
    if sim.log_rows:
        fieldnames = list(sim.log_rows[0].keys())
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(sim.log_rows)

    return sim, metrics, out_path


def summary_row(config, scenario_name, run_number, sim, metrics):
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
    }


def main():
    parser = argparse.ArgumentParser(description="Run every scenario N times, saving one CSV log per run")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--runs", type=int, default=3, help="Repeated trials per scenario (task requires >= 3)")
    parser.add_argument("--logs-dir", default="logs")
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--combined-log", default="logs/simulation_log.csv",
                         help="Combined log covering run 1 of every scenario")
    parser.add_argument("--summary-output", default="results/results_summary.csv",
                         help="Aggregated per-run metrics summary")
    args = parser.parse_args()

    os.makedirs(args.logs_dir, exist_ok=True)
    os.makedirs(args.results_dir, exist_ok=True)

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = list(config["scenarios"].keys())
    base_seed = config["sim"]["seed"]

    combined_rows = []
    summary_rows = []
    print(f"Running {len(scenario_names)} scenarios x {args.runs} trials each...\n")

    for scenario_name in scenario_names:
        for run_number in range(1, args.runs + 1):
            seed = base_seed + (run_number - 1)
            sim, metrics, out_path = run_and_save(config, scenario_name, run_number, seed, args.logs_dir)

            if run_number == 1:
                combined_rows.extend(sim.log_rows)

            summary_rows.append(summary_row(config, scenario_name, run_number, sim, metrics))

            print(f"[{scenario_name} run {run_number} | seed={seed}] -> {out_path}")
            print(f"    mission_success={metrics['mission_success']}  "
                  f"reached_goal={metrics['uavs_reached_goal']}/{metrics['num_uavs']}  "
                  f"collisions={metrics['collision_count']}  "
                  f"fusion_mode={metrics['fusion_mode']}  "
                  f"fusion_recovery={metrics['fusion_recovery_count']}")

    if combined_rows:
        fieldnames = list(combined_rows[0].keys())
        combined_dir = os.path.dirname(args.combined_log)
        if combined_dir:
            os.makedirs(combined_dir, exist_ok=True)
        with open(args.combined_log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(combined_rows)
        print(f"\nCombined log (run 1 of every scenario) -> {args.combined_log}")

    print(f"\nSaved {len(scenario_names) * args.runs} per-run CSV logs to {args.logs_dir}/")

    summary_dir = os.path.dirname(args.summary_output)
    if summary_dir:
        os.makedirs(summary_dir, exist_ok=True)
    with open(args.summary_output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDNAMES)
        writer.writeheader()
        writer.writerows(summary_rows)
    print(f"Aggregated results summary -> {args.summary_output}")


if __name__ == "__main__":
    main()