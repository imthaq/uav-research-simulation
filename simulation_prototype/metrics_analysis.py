"""
run_experiments.py / metrics_analysis.py

Aggregates swarm-simulation metrics across scenarios/runs.
Includes comprehensive Tracking, Fusion, Swarm, and Communication metrics.
"""
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
    """Pulls out the perception-error / fusion parameters for a scenario."""
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


def _rmse_1d(vals):
    """vals is an iterable of 1D errors/deviations."""
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return round(math.sqrt(statistics.mean(v * v for v in vals)), 4)


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
    """Tracking/association metrics - optimized for large datasets."""
    rmse_pairs = []
    velocity_errors = []
    false_detections = 0
    missed_detections = 0
    false_tracks = 0
    missed_tracks = 0
    association_errors = 0
    active_count = 0
    active_good = 0
    covariance_traces = []
    
    by_uav = {}  
    
    for r in rows:
        # RMSE position
        if r.get("detected_x") is not None and r.get("true_target_x") is not None:
            rmse_pairs.append((r["detected_x"] - r["true_target_x"], 
                               r["detected_y"] - r["true_target_y"]))
        
        # Velocity errors (tracked as 1D deviations for velocity RMSE)
        if r.get("measured_radial_velocity") is not None and r.get("true_radial_velocity") is not None:
            velocity_errors.append(abs(r["measured_radial_velocity"] - r["true_radial_velocity"]))
        
        # Covariance
        if r.get("track_covariance_trace") is not None:
            covariance_traces.append(r["track_covariance_trace"])
        
        # Detections vs Tracks flags
        if r.get("false_detection_flag") or r.get("false_alarm_flag"):
            false_detections += 1
        if r.get("missed_detection_flag"):
            missed_detections += 1
        if r.get("false_track_flag"):
            false_tracks += 1
        if r.get("missed_track_flag"):
            missed_tracks += 1
        
        # Association errors
        if r.get("association_error_flag") or (r.get("clutter_flag") and r.get("track_status") is not None):
            association_errors += 1
        
        # Track continuity
        if r.get("true_range") is not None:
            active_count += 1
            if r.get("track_status") in ("tentative", "confirmed", "coasting"):
                active_good += 1
        
        # Group by UAV
        uav_id = r.get("uav_id")
        if uav_id is not None:
            by_uav.setdefault(uav_id, []).append(r)
    
    # Track confirmation, fragmentation, and lifetime analysis
    confirmation_times, fragmentation_count, loss_durations = [], 0, []
    lifetimes = []
    
    for uav_rows in by_uav.values():
        first_seen, confirmed_at, last_seen, prev_id = {}, {}, {}, None
        for r in uav_rows:
            tid = r.get("radar_track_id")
            t = r.get("time_step")
            if tid is not None:
                first_seen.setdefault(tid, t)
                last_seen[tid] = t
                if r.get("track_status") == "confirmed" and tid not in confirmed_at:
                    confirmed_at[tid] = t
                if prev_id is not None and tid != prev_id:
                    fragmentation_count += 1
            prev_id = tid if tid is not None else prev_id
            
        confirmation_times.extend(confirmed_at[tid] - first_seen[tid] for tid in confirmed_at)
        lifetimes.extend(last_seen[tid] - first_seen[tid] for tid in first_seen)
        loss_durations.extend(_run_lengths(r.get("missed_detection_flag") for r in uav_rows))
    
    continuity = (active_good / active_count) if active_count else None
    
    return {
        "rmse_position_error": _rmse(rmse_pairs),
        "velocity_rmse": _rmse_1d(velocity_errors),
        "track_continuity": round(continuity, 4) if continuity is not None else None,
        "track_fragmentation": fragmentation_count,
        "false_detections": false_detections,
        "missed_detections": missed_detections,
        "false_tracks": false_tracks,
        "missed_tracks": missed_tracks,
        "track_confirmation_time_steps": _mean(confirmation_times),
        "track_loss_duration_steps": _mean(loss_durations),
        "average_track_lifetime_steps": _mean(lifetimes),
        "association_errors": association_errors,
        "average_covariance": _mean(covariance_traces),
    }


def fusion_metrics(rows):
    """Multi-source Fusion metrics."""
    fused_pairs = []
    nees_list = []
    stale_count = 0
    faulty_influence = []
    sensor_contributions = []

    for r in rows:
        # Fused position RMSE
        if r.get("fused_x") is not None and r.get("true_target_x") is not None:
            fused_pairs.append((r["fused_x"] - r["true_target_x"], 
                                r["fused_y"] - r["true_target_y"]))
        
        # Covariance consistency (NEES)
        if r.get("fused_x") is not None and r.get("fused_covariance_x") is not None:
            ex = r["fused_x"] - r["true_target_x"]
            ey = r["fused_y"] - r["true_target_y"]
            cov_x = max(r["fused_covariance_x"], 1e-6)
            cov_y = max(r.get("fused_covariance_y", cov_x), 1e-6)
            nees_list.append((ex**2 / cov_x) + (ey**2 / cov_y))
            
        if r.get("stale_data_flag"):
            stale_count += 1
            
        if r.get("sensor_contribution") is not None:
            sensor_contributions.append(r["sensor_contribution"])
            
        if r.get("faulty_sensor_influence") is not None:
            faulty_influence.append(r["faulty_sensor_influence"])

    return {
        "fused_position_rmse": _rmse(fused_pairs),
        "covariance_consistency_nees": _mean(nees_list),
        "avg_sensor_contribution": _mean(sensor_contributions),
        "stale_data_count": stale_count,
        "avg_faulty_sensor_influence": _mean(faulty_influence),
    }


def communication_metrics(rows):
    """Communication load and reliability metrics."""
    total_sent = 0
    sent_list = []
    sources_list = []
    delays = []
    stale_messages = 0
    outage_lengths = []
    recovery_times = []
    
    current_outage = 0
    
    for r in rows:
        msg_count = r.get("fusion_comm_messages")
        if msg_count is not None:
            total_sent += msg_count
            sent_list.append(msg_count)
            
            # Simple global outage tracker (0 messages received when sent > 0)
            if msg_count > 0 and r.get("fusion_num_sources", 0) == 0:
                current_outage += 1
            elif current_outage > 0:
                outage_lengths.append(current_outage)
                current_outage = 0
        
        num_sources = r.get("fusion_num_sources")
        if num_sources is not None:
            sources_list.append(num_sources)
        
        delay = r.get("fusion_response_time_steps")
        if delay is not None:
            delays.append(delay)
            
        if r.get("stale_message_flag"):
            stale_messages += 1
            
        if r.get("recovery_time_steps") is not None:
            recovery_times.append(r.get("recovery_time_steps"))
            
    if current_outage > 0:
        outage_lengths.append(current_outage)
    
    dropped = sum(s - n for s, n in zip(sent_list, sources_list))
    
    return {
        "messages_sent": total_sent,
        "messages_received": total_sent - dropped,
        "messages_dropped": dropped,
        "stale_messages": stale_messages,
        "avg_message_delay_steps": _mean(delays),
        "communication_load": _mean(sent_list),
        "max_outage_duration_steps": max(outage_lengths) if outage_lengths else 0,
        "avg_recovery_time_steps": _mean(recovery_times),
    }


def dependability_metrics(rows):
    """System-level dependability."""
    abstention_flags = []
    correct_abstention = []
    unnecessary_abstention = []
    handoff_success = []
    handoff_failure = []
    degraded_count = 0
    critical_count = 0
    safety_margin_values = []
    
    for r in rows:
        if r.get("abstention_flag") is not None:
            abstention_flags.append(r.get("abstention_flag"))
        if r.get("correct_abstention_flag") is not None:
            correct_abstention.append(r.get("correct_abstention_flag"))
        if r.get("unnecessary_abstention_flag") is not None:
            unnecessary_abstention.append(r.get("unnecessary_abstention_flag"))
        if r.get("handoff_success_flag") is not None:
            handoff_success.append(r.get("handoff_success_flag"))
        if r.get("handoff_failure_flag") is not None:
            handoff_failure.append(r.get("handoff_failure_flag"))
        if r.get("degraded_mode_flag"):
            degraded_count += 1
        if r.get("critical_mode_flag"):
            critical_count += 1
        if r.get("safety_margin_increase") is not None:
            safety_margin_values.append(r.get("safety_margin_increase"))
    
    return {
        "abstention_rate": round(sum(abstention_flags) / len(abstention_flags), 4) if abstention_flags else None,
        "correct_abstention_rate": round(sum(correct_abstention) / len(correct_abstention), 4) if correct_abstention else None,
        "unnecessary_abstention_rate": round(sum(unnecessary_abstention) / len(unnecessary_abstention), 4) if unnecessary_abstention else None,
        "handoff_success_rate": round(sum(handoff_success) / len(handoff_success), 4) if handoff_success else None,
        "handoff_failure_rate": round(sum(handoff_failure) / len(handoff_failure), 4) if handoff_failure else None,
        "time_in_degraded_mode": degraded_count,
        "time_in_critical_mode": critical_count,
        "mean_safety_margin_increase": _mean(safety_margin_values),
    }


def radar_specific_metrics(rows):
    """Radar phenomenological metrics."""
    ghost_count = 0
    fragmentation_count = 0
    doppler_count = 0
    multipath_count = 0
    
    for r in rows:
        if r.get("ghost_track_flag"):
            ghost_count += 1
        if r.get("extended_target_fragmentation_flag"):
            fragmentation_count += 1
        if r.get("doppler_ambiguity_flag"):
            doppler_count += 1
        if r.get("multipath_false_track_flag"):
            multipath_count += 1
    
    return {
        "ghost_track_count": ghost_count,
        "extended_target_fragmentation": fragmentation_count,
        "doppler_ambiguity_count": doppler_count,
        "multipath_false_track_count": multipath_count,
    }


def _calibration_pairs(rows):
    """Extracts (confidence, confidence_correct) pairs."""
    pairs = []
    for r in rows:
        conf = r.get("confidence_score")
        correct = r.get("confidence_correct")
        if conf is not None and correct is not None:
            pairs.append((float(conf), bool(correct)))
    return pairs


def _reliability_bins(pairs, num_bins=10):
    bins = [[] for _ in range(num_bins)]
    for conf, correct in pairs:
        idx = min(int(conf * num_bins), num_bins - 1)
        idx = max(idx, 0)
        bins[idx].append((conf, correct))
    return bins


def confidence_calibration_metrics(rows, num_bins=10):
    pairs = _calibration_pairs(rows)
    n = len(pairs)
    if n == 0:
        return {
            "n_samples": 0, "expected_calibration_error": None,
            "maximum_calibration_error": None, "brier_score": None,
            "negative_log_likelihood": None, "overconfidence_rate": None,
            "underconfidence_rate": None, "reliability_bins": []
        }

    eps = 1e-7
    brier_terms, nll_terms = [], []
    for conf, correct in pairs:
        y = 1.0 if correct else 0.0
        brier_terms.append((conf - y) ** 2)
        p = min(max(conf, eps), 1.0 - eps)
        nll_terms.append(-(y * math.log(p) + (1.0 - y) * math.log(1.0 - p)))

    bins = _reliability_bins(pairs, num_bins)
    ece, mce = 0.0, 0.0
    overconfident_n, underconfident_n = 0, 0
    formatted_bins = []
    for i, b in enumerate(bins):
        if not b:
            formatted_bins.append({"n": 0, "bin_confidence": 0.0, "bin_accuracy": 0.0})
            continue
        bin_confidence = statistics.mean(c for c, _ in b)
        bin_accuracy = statistics.mean(1.0 if y else 0.0 for _, y in b)
        gap = bin_confidence - bin_accuracy
        weight = len(b) / n
        ece += weight * abs(gap)
        mce = max(mce, abs(gap))
        if gap > 0:
            overconfident_n += len(b)
        elif gap < 0:
            underconfident_n += len(b)
            
        formatted_bins.append({
            "n": len(b),
            "bin_confidence": round(bin_confidence, 4),
            "bin_accuracy": round(bin_accuracy, 4)
        })

    return {
        "n_samples": n,
        "expected_calibration_error": round(ece, 4),
        "maximum_calibration_error": round(mce, 4),
        "brier_score": round(statistics.mean(brier_terms), 6),
        "negative_log_likelihood": round(statistics.mean(nll_terms), 6),
        "overconfidence_rate": round(overconfident_n / n, 4),
        "underconfidence_rate": round(underconfident_n / n, 4),
        "reliability_bins": formatted_bins,
    }


def run_once(config, scenario_name, seed, attach_dependability=False):
    """Runs the full pipeline once and returns a flat metrics dict."""
    run_config = copy.deepcopy(config)
    run_config["sim"]["seed"] = seed

    from simple_swarm_sim import run_radar_track_fusion_pipeline
    rows, metrics = run_radar_track_fusion_pipeline(run_config, scenario_name, attach_dependability=attach_dependability)
    
    # Swarm metric derivations
    collision_count = sum(1 for r in rows if r.get("collision_flag"))
    collision_risk_count = sum(1 for r in rows if r.get("collision_risk_flag"))
    wrong_decisions = metrics.get("unnecessary_avoidance_count", 0) + metrics.get("missed_response_count", 0)
    
    formation_errors = [r["formation_error"] for r in rows if r.get("formation_error") is not None]
    formation_error_rmse = _rmse_1d(formation_errors)
    swarm_stability = round(statistics.pstdev(formation_errors), 4) if len(formation_errors) > 1 else None
    
    min_separation = min((r["nearest_entity_distance"] for r in rows if r.get("nearest_entity_distance") is not None), default=None)

    out = {
        "seed": seed,
        "collision_count": collision_count,
        "total_near_misses": metrics.get("near_miss_count", 0),
        "collision_risk_count": collision_risk_count,
        "minimum_separation": min_separation,
        "unnecessary_avoidance_count": metrics.get("unnecessary_avoidance_count", 0),
        "missed_response_count": metrics.get("missed_response_count", 0),
        "hold_duration": metrics.get("hold_duration_steps", 0),
        "fusion_recovery_count": metrics.get("fusion_recovery_count", 0),
        "mission_success": metrics.get("mission_success", False),
        "mission_completion_time_s": metrics.get("mission_completion_time_s"),
        "avg_response_time_s": metrics.get("avg_response_time_s"),
        "formation_error_rmse": formation_error_rmse,
        "avg_confidence_error": metrics.get("avg_confidence_error"),
        "wrong_decisions": wrong_decisions,
        "swarm_stability": swarm_stability,
    }
    
    out.update(perception_metrics(rows))
    out.update(fusion_metrics(rows))
    out.update(communication_metrics(rows))
    
    calibration = confidence_calibration_metrics(rows)
    out.update(calibration)
    
    out.update(dependability_metrics(rows))
    out.update(radar_specific_metrics(rows))
    
    return out


# --- Field Name Definitions for CSV Export ---
PERCEPTION_FIELDS = [
    "rmse_position_error", "velocity_rmse", "track_continuity",
    "track_fragmentation", "false_detections", "missed_detections", 
    "false_tracks", "missed_tracks", "track_confirmation_time_steps", 
    "track_loss_duration_steps", "average_track_lifetime_steps",
    "association_errors", "average_covariance",
]
FUSION_FIELDS = [
    "fused_position_rmse", "covariance_consistency_nees", 
    "avg_sensor_contribution", "stale_data_count", "avg_faulty_sensor_influence",
]
COMMUNICATION_FIELDS = [
    "messages_sent", "messages_received", "messages_dropped", "stale_messages",
    "avg_message_delay_steps", "communication_load", "max_outage_duration_steps",
    "avg_recovery_time_steps"
]
SWARM_FIELDS = [
    "collision_count", "total_near_misses", "collision_risk_count", "minimum_separation",
    "mission_success", "mission_completion_time_s", "avg_response_time_s",
    "formation_error_rmse", "unnecessary_avoidance_count", "missed_response_count",
    "hold_duration", "wrong_decisions", "swarm_stability",
]
CALIBRATION_FIELDS = [
    "expected_calibration_error", "maximum_calibration_error", "brier_score",
    "negative_log_likelihood", "overconfidence_rate", "underconfidence_rate",
    "n_samples",
]
DEPENDABILITY_FIELDS = [
    "abstention_rate", "correct_abstention_rate", "unnecessary_abstention_rate",
    "handoff_success_rate", "handoff_failure_rate", 
    "time_in_degraded_mode", "time_in_critical_mode", "mean_safety_margin_increase",
]
RADAR_SPECIFIC_FIELDS = [
    "ghost_track_count", "extended_target_fragmentation",
    "doppler_ambiguity_count", "multipath_false_track_count",
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
    ] + SWARM_FIELDS + PERCEPTION_FIELDS + FUSION_FIELDS + COMMUNICATION_FIELDS 
      + CALIBRATION_FIELDS + DEPENDABILITY_FIELDS + RADAR_SPECIFIC_FIELDS)

    rows = []
    scenario_runs = {} 

    total_runs = len(scenario_names) * args.runs
    done = 0
    print(f"Analyzing {len(scenario_names)} scenarios, {args.runs} runs each (Total: {total_runs})...")

    for scenario_name in scenario_names:
        scn = config["scenarios"][scenario_name]
        params = scenario_params(scn)
        run_metrics_list = []

        for run_number in range(1, args.runs + 1):
            seed = base_seed + (run_number - 1)
            m = run_once(config, scenario_name, seed)
            run_metrics_list.append(m)
            
            done += 1
            print(f"  [{done}/{total_runs}] Completed {scenario_name} run {run_number} (seed={seed})")

            row = {
                "scenario": scenario_name,
                "run_number": run_number,
                **params,
                "mission_success": "Yes" if m["mission_success"] else "No",
            }
            
            for key in (SWARM_FIELDS + PERCEPTION_FIELDS + FUSION_FIELDS + 
                        COMMUNICATION_FIELDS + CALIBRATION_FIELDS + 
                        DEPENDABILITY_FIELDS + RADAR_SPECIFIC_FIELDS):
                if key != "mission_success":
                    row[key] = m.get(key)
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
        success_rate = sum(1 for m in run_metrics_list if m.get("mission_success")) / n
        print(f"[{scenario_name}] runs={n}  mission_success_rate={success_rate:.0%}  "
              f"avg_collision_count={_mean(m.get('collision_count') for m in run_metrics_list)}  "
              f"avg_fused_rmse={_mean(m.get('fused_position_rmse') for m in run_metrics_list)}  "
              f"avg_rmse_position={_mean(m.get('rmse_position_error') for m in run_metrics_list)}  "
              f"avg_track_continuity={_mean(m.get('track_continuity') for m in run_metrics_list)}  "
              f"avg_messages_sent={_mean(m.get('messages_sent') for m in run_metrics_list)}")


if __name__ == "__main__":
    sys.exit(main())