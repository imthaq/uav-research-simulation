import argparse
import copy
import csv
import json
import math
import os
import statistics
import sys

from simple_swarm_sim import run_radar_track_fusion_pipeline


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


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.mean(vals), 4) if vals else None


def _rmse(pairs):
    """pairs is an iterable of (dx, dy) errors."""
    pairs = [(dx, dy) for dx, dy in pairs if dx is not None and dy is not None]
    if not pairs:
        return None
    return round(math.sqrt(statistics.mean(dx * dx + dy * dy for dx, dy in pairs)), 4)


def _run_lengths(flags):
    """Lengths of consecutive True runs in a bool sequence."""
    lengths, current = [], 0
    for f in flags:
        if f:
            current += 1
        else:
            if current:
                lengths.append(current)
            current = 0
    if current:
        lengths.append(current)
    return lengths


def perception_metrics(rows):
    """RMSE/tracking/association metrics derived from one run's per-step
    pipeline rows (see simple_swarm_sim.run_radar_track_fusion_pipeline)."""
    rmse_position = _rmse((r["detected_x"] - r["true_target_x"], r["detected_y"] - r["true_target_y"])
                           for r in rows if r["detected_x"] is not None)
    fusion_consistency_error = _rmse((r["fused_x"] - r["true_target_x"], r["fused_y"] - r["true_target_y"])
                                      for r in rows if r["fused_x"] is not None)
    velocity_errors = [abs(r["measured_radial_velocity"] - r["true_radial_velocity"])
                        for r in rows if r["measured_radial_velocity"] is not None and r["true_radial_velocity"] is not None]

    by_uav = {}
    for r in sorted(rows, key=lambda r: (r["uav_id"], r["time_step"])):
        by_uav.setdefault(r["uav_id"], []).append(r)

    confirmation_times, fragmentation_count, loss_durations = [], 0, []
    for uav_rows in by_uav.values():
        first_seen, confirmed_at, prev_id = {}, {}, None
        for r in uav_rows:
            tid = r["radar_track_id"]
            if tid is not None:
                first_seen.setdefault(tid, r["time_step"])
                if r["track_status"] == "confirmed" and tid not in confirmed_at:
                    confirmed_at[tid] = r["time_step"]
                if prev_id is not None and tid != prev_id:
                    fragmentation_count += 1
            prev_id = tid if tid is not None else prev_id
        confirmation_times.extend(confirmed_at[tid] - first_seen[tid] for tid in confirmed_at)
        loss_durations.extend(_run_lengths(r["missed_detection_flag"] for r in uav_rows))

    association_errors = sum(1 for r in rows if r["clutter_flag"] and r["track_status"] is not None)
    active_rows = [r for r in rows if r["true_range"] is not None]
    continuity = (sum(1 for r in active_rows if r["track_status"] in ("tentative", "confirmed", "coasting"))
                  / len(active_rows)) if active_rows else None

    return {
        "rmse_position_error": rmse_position,
        "velocity_estimation_error": _mean(velocity_errors),
        "track_continuity": round(continuity, 4) if continuity is not None else None,
        "track_fragmentation": fragmentation_count,
        "false_track_count": sum(1 for r in rows if r["false_alarm_flag"]),
        "missed_track_count": sum(1 for r in rows if r["missed_detection_flag"]),
        "track_confirmation_time_steps": _mean(confirmation_times),
        "track_loss_duration_steps": _mean(loss_durations),
        "association_error_count": association_errors,
        "average_covariance": _mean(r["track_covariance_trace"] for r in rows),
        "fusion_consistency_error": fusion_consistency_error,
    }


def communication_metrics(rows):
    sent = [r["fusion_comm_messages"] for r in rows if r["fusion_comm_messages"] is not None]
    sources = [r["fusion_num_sources"] for r in rows if r["fusion_num_sources"] is not None]
    delays = [r["fusion_response_time_steps"] for r in rows if r["fusion_response_time_steps"] is not None]
    # Every UAV that reported a track is a source that got fused in; the gap
    # between attempted uplinks and used sources approximates drop/staleness.
    dropped = sum(s - n for s, n in zip(sent, sources))
    return {
        "messages_sent": sum(sent),
        "messages_dropped": dropped,
        "avg_message_delay_steps": _mean(delays),
        "communication_load": _mean(sent),
    }


def run_once(config, scenario_name, seed):
    """Runs the full radar -> track -> fusion -> decision pipeline once
    with the given seed and returns a flat metrics dict."""
    run_config = copy.deepcopy(config)
    run_config["sim"]["seed"] = seed

    rows, metrics = run_radar_track_fusion_pipeline(run_config, scenario_name)
    collision_risk_count = sum(1 for r in rows if r["collision_risk_flag"])
    wrong_decisions = metrics["unnecessary_avoidance_count"] + metrics["missed_response_count"]
    formation_errors = [r["formation_error"] for r in rows if r["formation_error"] is not None]
    swarm_stability = round(statistics.pstdev(formation_errors), 4) if len(formation_errors) > 1 else None

    out = {
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
        "wrong_decisions": wrong_decisions,
        "swarm_stability": swarm_stability,
    }
    out.update(perception_metrics(rows))
    out.update(communication_metrics(rows))
    return out


PERCEPTION_FIELDS = [
    "rmse_position_error", "velocity_estimation_error", "track_continuity",
    "track_fragmentation", "false_track_count", "missed_track_count",
    "track_confirmation_time_steps", "track_loss_duration_steps",
    "association_error_count", "average_covariance", "fusion_consistency_error",
]
COMMUNICATION_FIELDS = [
    "messages_sent", "messages_dropped", "avg_message_delay_steps", "communication_load",
]
SWARM_FIELDS = [
    "collision_risk_count", "total_near_misses", "mission_success", "avg_response_time_s",
    "avg_formation_error", "unnecessary_avoidance_count", "missed_response_count",
    "wrong_decisions", "swarm_stability",
]


def main():
    parser = argparse.ArgumentParser(description="Aggregate swarm-simulation metrics across scenarios/runs")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--runs", type=int, default=5, help="Number of seeded runs per scenario")
    parser.add_argument("--scenario", default=None, help="Only analyze this one scenario")
    parser.add_argument("--output", default="results/results_summary.csv")
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

    fieldnames = ([
        "scenario", "run_number", "fusion_mode", "false_positive_rate", "false_negative_rate",
        "noise_level", "latency_steps", "dropout_probability", "confidence_error_level",
    ] + SWARM_FIELDS + PERCEPTION_FIELDS + COMMUNICATION_FIELDS)

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

            row = {
                "scenario": scenario_name,
                "run_number": run_number,
                **params,
                "mission_success": "Yes" if m["mission_success"] else "No",
            }
            for key in SWARM_FIELDS + PERCEPTION_FIELDS + COMMUNICATION_FIELDS:
                if key != "mission_success":
                    row[key] = m[key]
            rows.append(row)

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
        print(f"[{scenario_name}] runs={n}  mission_success_rate={success_rate:.0%}  "
              f"avg_collision_risk={_mean(m['collision_risk_count'] for m in run_metrics_list)}  "
              f"avg_rmse_position={_mean(m['rmse_position_error'] for m in run_metrics_list)}  "
              f"avg_track_continuity={_mean(m['track_continuity'] for m in run_metrics_list)}  "
              f"avg_messages_sent={_mean(m['messages_sent'] for m in run_metrics_list)}")


if __name__ == "__main__":
    sys.exit(main())