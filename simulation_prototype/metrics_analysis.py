import argparse
import copy
import csv
import json
import os
import statistics
import sys

from simple_swarm_sim import Simulation


def scenario_params(scn):
    """Pulls out the perception-error / fusion parameters for a scenario,
    using the same defaults Perception/Simulation fall back on when a key
    is absent."""
    return {
        "false_positive_rate": scn.get("false_positive_rate", 0.0),
        "false_negative_rate": scn.get("false_negative_rate", 0.0),
        "noise_level": scn.get("position_noise_std", 0.0),
        "latency_steps": scn.get("latency_steps", 0),
        "dropout_probability": scn.get("dropout_prob", 0.0),
        "confidence_error_level": scn.get("confidence_error_level", 0.0),
        "fusion_mode": scn.get("fusion_mode", "no_fusion"),
    }


def run_once(config, scenario_name, seed):
    """Runs a single simulation with the given seed and returns a metrics dict."""
    run_config = copy.deepcopy(config)
    run_config["sim"]["seed"] = seed

    sim = Simulation(run_config, scenario_name)
    metrics = sim.run()

    # collision_risk_count is a finer-grained count than near_miss_count:
    # near_miss_count (from metrics) counts each close UAV/obstacle *pair*
    # once per step, whereas collision_risk_flag (added in Task 6's logging)
    # is set per UAV per step whenever its single nearest entity is within
    # near_miss_distance - i.e. every step-level risk event, not just pairs.
    collision_risk_count = sum(1 for row in sim.log_rows if row.get("collision_risk_flag"))

    return {
        "seed": seed,
        "total_near_misses": metrics["near_miss_count"],
        "collision_risk_count": collision_risk_count,
        "unnecessary_avoidance_count": metrics["unnecessary_avoidance_count"],
        "missed_response_count": metrics["missed_response_count"],
        "fusion_recovery_count": metrics["fusion_recovery_count"],
        "mission_success": metrics["mission_success"],
        "avg_response_time_s": metrics["avg_response_time_s"],
        "avg_formation_error": metrics["avg_formation_error"],
        "avg_confidence_error": metrics["avg_confidence_error"],
    }


def main():
    parser = argparse.ArgumentParser(description="Aggregate swarm-simulation metrics across scenarios/runs")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--runs", type=int, default=5, help="Number of seeded runs per scenario")
    parser.add_argument("--scenario", default=None, help="Only analyze this one scenario")
    parser.add_argument("--output", default="logs/final_metric_summary.csv")
    args = parser.parse_args()

    if args.runs < 1:
        sys.exit(f"--runs must be >= 1 (got {args.runs})")

    with open(args.config) as f:
        config = json.load(f)

    if args.scenario and args.scenario not in config["scenarios"]:
        available = ", ".join(config["scenarios"].keys())
        sys.exit(f"Unknown scenario '{args.scenario}'. Available: {available}")

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    base_seed = config["sim"]["seed"]

    fieldnames = [
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
        # bonus columns covering the remaining requested "calculate" metrics
        "total_near_misses",
        "avg_formation_error",
        "avg_confidence_error",
    ]

    rows = []
    scenario_runs = {}  # scenario_name -> list of per-run metric dicts, for the console summary

    for scenario_name in scenario_names:
        scn = config["scenarios"][scenario_name]
        params = scenario_params(scn)
        run_metrics_list = []

        for run_number in range(1, args.runs + 1):
            seed = base_seed + (run_number - 1)
            m = run_once(config, scenario_name, seed)
            run_metrics_list.append(m)

            rows.append({
                "scenario": scenario_name,
                "run_number": run_number,
                "fusion_mode": params["fusion_mode"],
                "false_positive_rate": params["false_positive_rate"],
                "false_negative_rate": params["false_negative_rate"],
                "noise_level": params["noise_level"],
                "latency_steps": params["latency_steps"],
                "dropout_probability": params["dropout_probability"],
                "confidence_error_level": params["confidence_error_level"],
                "collision_risk_count": m["collision_risk_count"],
                "unnecessary_avoidance_count": m["unnecessary_avoidance_count"],
                "missed_response_count": m["missed_response_count"],
                "fusion_recovery_count": m["fusion_recovery_count"],
                "mission_success": "Yes" if m["mission_success"] else "No",
                "avg_response_time_s": m["avg_response_time_s"],
                "total_near_misses": m["total_near_misses"],
                "avg_formation_error": m["avg_formation_error"],
                "avg_confidence_error": m["avg_confidence_error"],
            })

        scenario_runs[scenario_name] = run_metrics_list

    out_path = args.output
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    for scenario_name, run_metrics_list in scenario_runs.items():
        n = len(run_metrics_list)
        success_rate = sum(1 for m in run_metrics_list if m["mission_success"]) / n
        avg_collision_risk = statistics.mean(m["collision_risk_count"] for m in run_metrics_list)
        avg_near_misses = statistics.mean(m["total_near_misses"] for m in run_metrics_list)
        avg_unnecessary = statistics.mean(m["unnecessary_avoidance_count"] for m in run_metrics_list)
        avg_missed = statistics.mean(m["missed_response_count"] for m in run_metrics_list)
        avg_fusion_recovery = statistics.mean(m["fusion_recovery_count"] for m in run_metrics_list)
        response_times = [m["avg_response_time_s"] for m in run_metrics_list if m["avg_response_time_s"] is not None]
        avg_response = statistics.mean(response_times) if response_times else None
        formation_errors = [m["avg_formation_error"] for m in run_metrics_list if m["avg_formation_error"] is not None]
        avg_formation = statistics.mean(formation_errors) if formation_errors else None
        confidence_errors = [m["avg_confidence_error"] for m in run_metrics_list if m["avg_confidence_error"] is not None]
        avg_confidence = statistics.mean(confidence_errors) if confidence_errors else None

        print(f"[{scenario_name}] runs={n}  mission_success_rate={success_rate:.0%} \n"
              f"avg_collision_risk_count={avg_collision_risk:.1f}  avg_near_misses={avg_near_misses:.1f}  \n"
              f"avg_unnecessary_avoidance={avg_unnecessary:.1f}  avg_missed_response={avg_missed:.1f}  \n"
              f"avg_fusion_recovery={avg_fusion_recovery:.1f}  \n"
              f"avg_response_time_s={('%.3f' % avg_response) if avg_response is not None else 'N/A'}  \n"
              f"avg_formation_error={('%.3f' % avg_formation) if avg_formation is not None else 'N/A'}  \n"
              f"avg_confidence_error={('%.3f' % avg_confidence) if avg_confidence is not None else 'N/A'}\n")


if __name__ == "__main__":
    sys.exit(main())