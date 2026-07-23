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


def dependability_metrics(rows):
    """Dependability metrics: abstention, handoff, recovery, and degraded/critical mode durations."""
    abstention_flags = [r.get("abstention_flag") for r in rows if r.get("abstention_flag") is not None]
    correct_abstention = [r.get("correct_abstention_flag") for r in rows if r.get("correct_abstention_flag") is not None]
    unnecessary_abstention = [r.get("unnecessary_abstention_flag") for r in rows if r.get("unnecessary_abstention_flag") is not None]
    
    handoff_success = [r.get("handoff_success_flag") for r in rows if r.get("handoff_success_flag") is not None]
    handoff_failure = [r.get("handoff_failure_flag") for r in rows if r.get("handoff_failure_flag") is not None]
    recovery_times = [r.get("recovery_time_steps") for r in rows if r.get("recovery_time_steps") is not None]
    
    degraded_mode_flags = [r.get("degraded_mode_flag") for r in rows if r.get("degraded_mode_flag") is not None]
    critical_mode_flags = [r.get("critical_mode_flag") for r in rows if r.get("critical_mode_flag") is not None]
    safety_margin_increase = [r.get("safety_margin_increase") for r in rows if r.get("safety_margin_increase") is not None]
    
    return {
        "abstention_rate": round(sum(abstention_flags) / len(abstention_flags), 4) if abstention_flags else None,
        "correct_abstention_rate": round(sum(correct_abstention) / len(correct_abstention), 4) if correct_abstention else None,
        "unnecessary_abstention_rate": round(sum(unnecessary_abstention) / len(unnecessary_abstention), 4) if unnecessary_abstention else None,
        "handoff_success_rate": round(sum(handoff_success) / len(handoff_success), 4) if handoff_success else None,
        "handoff_failure_rate": round(sum(handoff_failure) / len(handoff_failure), 4) if handoff_failure else None,
        "mean_recovery_time_steps": _mean(recovery_times),
        "time_in_degraded_mode": sum(degraded_mode_flags) if degraded_mode_flags else 0,
        "time_in_critical_mode": sum(critical_mode_flags) if critical_mode_flags else 0,
        "mean_safety_margin_increase": _mean(safety_margin_increase),
    }


def radar_specific_metrics(rows):
    """Radar-specific metrics: ghost tracks, extended-target fragmentation, Doppler ambiguity, multipath false tracks."""
    ghost_track_count = sum(1 for r in rows if r.get("ghost_track_flag"))
    extended_target_fragmentation = sum(1 for r in rows if r.get("extended_target_fragmentation_flag"))
    doppler_ambiguity_count = sum(1 for r in rows if r.get("doppler_ambiguity_flag"))
    multipath_false_track_count = sum(1 for r in rows if r.get("multipath_false_track_flag"))
    
    return {
        "ghost_track_count": ghost_track_count,
        "extended_target_fragmentation": extended_target_fragmentation,
        "doppler_ambiguity_count": doppler_ambiguity_count,
        "multipath_false_track_count": multipath_false_track_count,
    }


def _calibration_pairs(rows):
    """Extracts (probability_of_detection, detected) pairs for confidence-
    calibration analysis - see radar_like_model.calibration_pairs, which
    this mirrors so the fusion-pipeline rows (which carry the same field
    names) can be analyzed the same way without importing radar_like_model
    here. Pairs the radar's own reported probability_of_detection for a
    real target against whether it was actually detected, excluding
    false-alarm/clutter rows, dropout rows, and hard range/FOV-gated
    misses (radar_pd_miss_flag) - none of those outcomes were actually
    determined by the PD roll, so pairing them in would make the check
    tautological or contaminated rather than a real calibration signal.
    confidence_score is deliberately not used here (see radar_like_model
    for why: it's only ever present on rows already known, by
    construction, to be a genuine detection or confirmed false alarm, so
    calibrating it against that trivially-true label is uninformative)."""
    pairs = []
    for r in rows:
        if r.get("false_alarm_flag") or r.get("dropout_flag") or r.get("radar_pd_miss_flag"):
            continue
        status = r.get("detection_status")
        if status not in ("detected", "missed"):
            continue
        p = r.get("probability_of_detection")
        if p is None:
            continue
        pairs.append((float(p), status == "detected"))
    return pairs


def _reliability_bins(pairs, num_bins=10):
    """Buckets (confidence, correct) pairs into num_bins equal-width bins
    over [0, 1], e.g. bin 7 of 10 holds confidences in [0.7, 0.8)."""
    bins = [[] for _ in range(num_bins)]
    for conf, correct in pairs:
        idx = min(int(conf * num_bins), num_bins - 1)
        idx = max(idx, 0)
        bins[idx].append((conf, correct))
    return bins


def confidence_calibration_metrics(rows, num_bins=10):
    """Confidence-calibration metrics answering the core question: does a
    reported radar detection probability of, say, 0.8 actually correspond
    to about an 80% correct-detection frequency across repeated trials?
    (See radar_like_model.calibration_pairs / _calibration_pairs above for
    exactly which rows count and why confidence_score isn't the right
    field to use here.)

    Computes, over every (probability_of_detection, detected) pair found
    in rows (see _calibration_pairs):
      - expected_calibration_error (ECE): bin-count-weighted average gap
        between each bin's mean confidence and its actual accuracy.
      - maximum_calibration_error (MCE): the worst single-bin gap.
      - brier_score: mean squared error between confidence and the
        binary correctness outcome.
      - negative_log_likelihood: mean binary log-loss of confidence as a
        predicted probability of correctness.
      - overconfidence_rate / underconfidence_rate: fraction of samples
        falling in bins where confidence exceeds accuracy (overconfident)
        or falls short of it (underconfident).
      - reliability_bins: per-bin sample count, mean accuracy, mean
        confidence, and confidence-minus-accuracy gap - the data behind a
        reliability diagram.
    """
    pairs = _calibration_pairs(rows)
    n = len(pairs)
    if n == 0:
        return {
            "n_samples": 0,
            "expected_calibration_error": None,
            "maximum_calibration_error": None,
            "brier_score": None,
            "negative_log_likelihood": None,
            "overconfidence_rate": None,
            "underconfidence_rate": None,
            "reliability_bins": [],
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
    reliability_bins = []
    for i, b in enumerate(bins):
        lo, hi = i / num_bins, (i + 1) / num_bins
        if not b:
            reliability_bins.append({
                "bin_range": [round(lo, 4), round(hi, 4)],
                "n": 0, "bin_accuracy": None, "bin_confidence": None, "gap": None,
            })
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
        reliability_bins.append({
            "bin_range": [round(lo, 4), round(hi, 4)],
            "n": len(b),
            "bin_accuracy": round(bin_accuracy, 4),
            "bin_confidence": round(bin_confidence, 4),
            "gap": round(gap, 4),
        })

    return {
        "n_samples": n,
        "expected_calibration_error": round(ece, 4),
        "maximum_calibration_error": round(mce, 4),
        "brier_score": round(statistics.mean(brier_terms), 6),
        "negative_log_likelihood": round(statistics.mean(nll_terms), 6),
        "overconfidence_rate": round(overconfident_n / n, 4),
        "underconfidence_rate": round(underconfident_n / n, 4),
        "reliability_bins": reliability_bins,
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

    calibration = confidence_calibration_metrics(rows)
    out.update({
        "expected_calibration_error": calibration["expected_calibration_error"],
        "maximum_calibration_error": calibration["maximum_calibration_error"],
        "brier_score": calibration["brier_score"],
        "negative_log_likelihood": calibration["negative_log_likelihood"],
        "overconfidence_rate": calibration["overconfidence_rate"],
        "underconfidence_rate": calibration["underconfidence_rate"],
        "calibration_n_samples": calibration["n_samples"],
    })
    
    dependability = dependability_metrics(rows)
    out.update(dependability)
    
    radar_specific = radar_specific_metrics(rows)
    out.update(radar_specific)
    
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
CALIBRATION_FIELDS = [
    "expected_calibration_error", "maximum_calibration_error", "brier_score",
    "negative_log_likelihood", "overconfidence_rate", "underconfidence_rate",
    "calibration_n_samples",
]
DEPENDABILITY_FIELDS = [
    "abstention_rate", "correct_abstention_rate", "unnecessary_abstention_rate",
    "handoff_success_rate", "handoff_failure_rate", "mean_recovery_time_steps",
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
    ] + SWARM_FIELDS + PERCEPTION_FIELDS + COMMUNICATION_FIELDS + CALIBRATION_FIELDS
      + DEPENDABILITY_FIELDS + RADAR_SPECIFIC_FIELDS)

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
            for key in SWARM_FIELDS + PERCEPTION_FIELDS + COMMUNICATION_FIELDS + CALIBRATION_FIELDS + DEPENDABILITY_FIELDS + RADAR_SPECIFIC_FIELDS:
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
              f"avg_messages_sent={_mean(m['messages_sent'] for m in run_metrics_list)}  "
              f"avg_ece={_mean(m['expected_calibration_error'] for m in run_metrics_list)}  "
              f"avg_brier={_mean(m['brier_score'] for m in run_metrics_list)}")


if __name__ == "__main__":
    sys.exit(main())