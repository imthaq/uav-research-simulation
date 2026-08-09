"""
Usage:
    python run_final_dependability_experiments.py
    python run_final_dependability_experiments.py --trials 5 --output-dir results/dependability
"""

import argparse
import copy
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime, timezone

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEPENDABILITY_DIR = os.path.join(_ROOT_DIR, "dependability")
for _p in (_ROOT_DIR, _DEPENDABILITY_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dependability_common import clone_scenario, run_trials, seed_range, DependabilityWriter

from calibration.radar_calibration_analysis import analyze_scenario as radar_calibration_analyze
from models.radar_like_model import RadarLikeModel
from tracking.radar_track_model import build_tracks
from fusion.fusion_model import (
    build_fused_log, TrustTracker, ARCHITECTURE_CENTRALIZED, ARCHITECTURE_DISTRIBUTED,
    COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION,
    _as_source, _cluster,
)
from dependability.perception_quality_monitor import PerceptionQualityMonitor
from dependability.selective_swarm_decision import SelectiveDecisionMaker

from experiments.run_experiments import (
    build_run_matrix, run_and_save, run_level_row, aggregate_scenario,
)


# ---------------------------------------------------------------------
# small shared numeric helpers
# ---------------------------------------------------------------------
def _mean_stdev(values):
    values = [v for v in values if v is not None]
    if not values:
        return None, None
    if len(values) == 1:
        return values[0], 0.0
    return statistics.mean(values), statistics.stdev(values)


def _true_positions_by_step(detection_rows):
    """dict[time_step] -> list of unique (x, y) true target positions,
    deduped by target_id (every radar reports the same true target
    position redundantly each step)."""
    by_step = {}
    for r in detection_rows:
        if r.get("target_id") is None or r.get("true_target_x") is None:
            continue
        seen = by_step.setdefault(r["time_step"], {})
        seen[r["target_id"]] = (r["true_target_x"], r["true_target_y"])
    return {t: list(v.values()) for t, v in by_step.items()}


def _nearest_object_error(fused_rows, true_by_step):
    """For each fused row, the distance to whichever real target was
    closest at that time_step (association-free proxy metric - fusion
    output isn't labeled with which real object it corresponds to)."""
    errors = []
    for r in fused_rows:
        truths = true_by_step.get(r["time_step"])
        if not truths:
            continue
        fx, fy = r["fused_x"], r["fused_y"]
        d = min(math.hypot(fx - tx, fy - ty) for tx, ty in truths)
        errors.append(d)
    if not errors:
        return {"mean_error": None, "n": 0}
    return {"mean_error": round(sum(errors) / len(errors), 4), "n": len(errors)}


def head_to_head(arm_a_name, values_a, arm_b_name, values_b, lower_is_better=True):
    mean_a, sd_a = _mean_stdev(values_a)
    mean_b, sd_b = _mean_stdev(values_b)
    verdict = None
    if mean_a is not None and mean_b is not None:
        better = arm_a_name if (mean_a < mean_b) == lower_is_better else arm_b_name
        verdict = f"{better} performed better on this metric ({mean_a} vs {mean_b})"
    return {
        arm_a_name: {"mean": mean_a, "stdev": sd_a, "n": len(values_a)},
        arm_b_name: {"mean": mean_b, "stdev": sd_b, "n": len(values_b)},
        "verdict": verdict,
    }


# ---------------------------------------------------------------------
# 1. calibrated vs uncalibrated confidence
# ---------------------------------------------------------------------
def compare_calibrated_vs_uncalibrated(config, writer, base_seed, runs):
    arms = {
        "calibrated": "correctly_calibrated_radar",
        "mildly_overconfident": "mildly_overconfident_radar",
        "severely_overconfident": "severely_overconfident_radar",
        "underconfident": "underconfident_radar",
    }
    seeds = seed_range(base_seed, runs)
    metrics_by_arm, failures_by_arm, raw_by_arm = {}, {}, {}
    for arm, scenario in arms.items():
        def trial(seed, scenario=scenario):
            return radar_calibration_analyze(config, scenario, num_runs=1, base_seed=seed, num_bins=10)
        results, failures, _ = run_trials(f"calibrated_vs_uncalibrated/{arm}", trial, seeds)
        metrics_by_arm[arm] = [r["value"]["expected_calibration_error"] for r in results
                                if r["value"]["n_samples"] > 0]
        failures_by_arm[arm] = failures
        raw_by_arm[arm] = [r["value"] for r in results]

    writer.write_configuration(config, list(arms.values()))
    writer.write_seeds({arm: seeds for arm in arms})
    writer.write_failed_runs(failures_by_arm)
    for arm, rows in raw_by_arm.items():
        writer.write_raw_log(f"raw_log_{arm}.json", rows)
    writer.write_run_summary({arm: vals for arm, vals in metrics_by_arm.items()})

    aggregated = head_to_head("calibrated", metrics_by_arm["calibrated"],
                               "mildly_overconfident", metrics_by_arm["mildly_overconfident"])
    aggregated["all_arms_ece"] = {arm: _mean_stdev(vals)[0] for arm, vals in metrics_by_arm.items()}
    aggregated["metric"] = "Expected Calibration Error (lower = better calibrated)"
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 2. fixed vs dynamic trust
# ---------------------------------------------------------------------
def compare_fixed_vs_dynamic_trust(config, writer, base_seed, trials):
    scenario = "faulty_sensor_trust_weighted_fusion_dynamic"
    seeds = seed_range(base_seed, trials)

    def run_arm(use_adaptive_trust):
        def trial(seed):
            run_config = copy.deepcopy(config)
            run_config["sim"]["seed"] = seed
            model = RadarLikeModel(run_config, scenario)
            detection_rows = model.run()
            true_by_step = _true_positions_by_step(detection_rows)
            fused_rows = build_fused_log(scenario, run_config, architecture=ARCHITECTURE_CENTRALIZED,
                                          seed=seed, use_adaptive_trust=use_adaptive_trust)
            err = _nearest_object_error(fused_rows, true_by_step)
            return {"mean_error": err["mean_error"], "n": err["n"],
                    "final_faulty_trust": (fused_rows[-1].get("avg_persistent_trust")
                                            if fused_rows else None)}
        return run_trials("fixed_vs_dynamic_trust", trial, seeds)

    results_fixed, failures_fixed, _ = run_arm(False)
    results_dynamic, failures_dynamic, _ = run_arm(True)

    writer.write_configuration(config, [scenario])
    writer.write_seeds({"fixed": seeds, "dynamic": seeds})
    writer.write_failed_runs({"fixed": failures_fixed, "dynamic": failures_dynamic})
    writer.write_raw_log("raw_log_fixed.json", [r["value"] for r in results_fixed])
    writer.write_raw_log("raw_log_dynamic.json", [r["value"] for r in results_dynamic])

    err_fixed = [r["value"]["mean_error"] for r in results_fixed]
    err_dynamic = [r["value"]["mean_error"] for r in results_dynamic]
    writer.write_run_summary({"fixed_mean_error_per_trial": err_fixed,
                               "dynamic_mean_error_per_trial": err_dynamic})

    aggregated = head_to_head("fixed", err_fixed, "dynamic", err_dynamic)
    aggregated["metric"] = "nearest-true-object fused position error, meters (lower = better)"
    aggregated["note"] = ("Uses build_fused_log(use_adaptive_trust=True/False) on the same "
                           "overconfident-faulty-sensor scenario, so any difference is "
                           "attributable to trust adaptation alone.")
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 3. fixed vs adaptive safety margin
# ---------------------------------------------------------------------
def compare_safety_margin(config, writer, base_seed, trials):
    arms = {
        "fixed": "safety_margin_fixed",
        "adaptive_covariance": "safety_margin_covariance",
        "adaptive_confidence": "safety_margin_confidence",
        "adaptive_quality_monitor": "safety_margin_quality_monitor",
    }
    return _run_mission_sim_comparison(config, writer, base_seed, trials, arms,
                                        metric_note="mission-level Monte Carlo via Simulation.run() "
                                                     "(safety_margin_mode is read directly by the core sim)")


# ---------------------------------------------------------------------
# 8. single fault vs combined fault
# ---------------------------------------------------------------------
def compare_single_vs_combined_fault(config, writer, base_seed, trials):
    arms = {
        "single_low_pd": "very_low_P_D",
        "single_high_pfa": "very_high_P_FA",
        "single_high_dropout": "high_dropout",
        "single_high_latency": "high_latency",
        "combined_all_faults": "simultaneous_sensor_failures",
    }
    return _run_mission_sim_comparison(config, writer, base_seed, trials, arms,
                                        metric_note="mission-level Monte Carlo; combined_all_faults "
                                                     "stacks every single-fault condition at once")


def _run_mission_sim_comparison(config, writer, base_seed, trials, arms, metric_note):
    seeds = seed_range(base_seed, trials)
    run_matrix_by_arm = {arm: build_run_matrix([scn], trials, base_seed, "sequential")
                          for arm, scn in arms.items()}
    all_rows, failures_by_arm, raw_by_arm = [], {}, {}

    for arm, scenario in arms.items():
        def trial(seed, scenario=scenario):
            sim, metrics, _ = run_and_save(config, scenario, 1, seed, logs_dir="/tmp/_dep_logs",
                                            save_step_log=False)
            return run_level_row(config, scenario, 1, seed, sim, metrics)
        results, failures, _ = run_trials(f"mission_sim/{arm}", trial, seeds)
        failures_by_arm[arm] = failures
        rows = [r["value"] for r in results]
        raw_by_arm[arm] = rows
        all_rows.extend(rows)

    writer.write_configuration(config, list(arms.values()))
    writer.write_seeds({arm: seeds for arm in arms})
    writer.write_failed_runs(failures_by_arm)
    for arm, rows in raw_by_arm.items():
        writer.write_raw_log(f"raw_log_{arm}.json", rows)
    writer.write_run_summary(all_rows)

    scenario_summaries = {arm: aggregate_scenario(scn, raw_by_arm[arm]) if raw_by_arm[arm] else None
                           for arm, scn in arms.items()}
    aggregated = {
        "metric": "mission_success_rate (higher = better) plus collision_risk_count (lower = better)",
        "note": metric_note,
        "by_arm": scenario_summaries,
    }
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 4. no abstention vs abstention
# ---------------------------------------------------------------------
def compare_abstention(config, writer, base_seed, trials):
    """Wires selective_swarm_decision.SelectiveDecisionMaker against the
    real per-step, per-UAV track stream (RadarLikeModel -> build_tracks,
    with a live TrustTracker feeding current_trust_value/sensor_agreement
    exactly as perception_quality_monitor.py's own docstring prescribes).

    Scope note (documented, not hidden): this measures the abstention
    layer's *decisions* against real perception-quality signals - trigger
    rate, episode outcomes, fallback distribution - not yet its effect on
    UAV actuation/mission outcome, since closing that loop means changing
    simple_swarm_sim.py's own decide_move(), which is out of scope here.
    "No abstention" = every degraded/critical step silently proceeds
    (the status quo); "abstention" = the same steps get a decision layer.
    """
    base_scenario = "faulty_sensor_trust_weighted_fusion_dynamic"
    scenario = "abstention_stress_test"
    stress_config = clone_scenario(
        config, base_scenario, scenario,
        {"radar_dropout_probability": 0.4, "radar_false_alarm_probability": 0.3,
         "radar_clutter_density": 0.3, "radar_latency_steps": 4},
        description="Task 23: degraded-perception stress scenario for the abstention comparison "
                    "- elevated dropout/false-alarm/clutter/latency on top of the existing "
                    "overconfident-faulty-sensor setup, so both GOOD and DEGRADED/CRITICAL "
                    "perception-quality steps actually occur.")
    seeds = seed_range(base_seed, trials)

    def trial(seed):
        run_config = copy.deepcopy(stress_config)
        run_config["sim"]["seed"] = seed
        model = RadarLikeModel(run_config, scenario)
        detection_rows = model.run()
        dt = run_config["sim"]["dt"]
        tracks = build_tracks(scenario, detection_rows, dt)

        trust_tracker = TrustTracker()
        monitor = PerceptionQualityMonitor()
        decision_maker = SelectiveDecisionMaker(quality_monitor=monitor)

        by_step = {}
        for row in tracks:
            by_step.setdefault(row["time_step"], {})[row["radar_id"]] = row

        no_abstention_degraded_or_critical = 0
        abstention_triggered = 0
        total_steps = 0
        for t in sorted(by_step):
            # Advance the real TrustTracker exactly the way fusion_model.py
            # itself does: normalize each radar's track row into a
            # "source" (_as_source), cluster them, then update().
            step_sources = [_as_source(row, persistent_trust=trust_tracker.get(rid))
                             for rid, row in by_step[t].items()]
            clusters = _cluster(step_sources) if step_sources else []
            if step_sources:
                trust_tracker.update(step_sources, clusters)
            for radar_id, row in by_step[t].items():
                total_steps += 1
                last_sig = trust_tracker.last_signals(radar_id)
                level, composite, _ = monitor.evaluate_track_row(
                    row, sensor_agreement=last_sig.get("agreement") if last_sig else None,
                    current_trust_value=trust_tracker.get(radar_id))
                if level != "good":
                    no_abstention_degraded_or_critical += 1
                decision = decision_maker.decide(radar_id, t, {
                    "track_covariance": row.get("covariance"), "track_age": row.get("age"),
                    "missed_update_count": row.get("missed_count"),
                    "sensor_agreement": last_sig.get("agreement") if last_sig else None,
                    "current_trust_value": trust_tracker.get(radar_id),
                })
                if decision["abstain"]:
                    abstention_triggered += 1

        episodes = decision_maker.log
        n_episodes_resolved = sum(1 for e in episodes if e["event"] == "abstention_resolved")
        outcomes = {}
        for e in episodes:
            if e["event"] == "abstention_resolved":
                outcomes[e["final_outcome"]] = outcomes.get(e["final_outcome"], 0) + 1

        return {
            "total_steps": total_steps,
            "no_abstention_unmitigated_degraded_or_critical_steps": no_abstention_degraded_or_critical,
            "no_abstention_unmitigated_rate": round(no_abstention_degraded_or_critical / total_steps, 4)
                if total_steps else None,
            "abstention_triggered_steps": abstention_triggered,
            "abstention_trigger_rate": round(abstention_triggered / total_steps, 4) if total_steps else None,
            "abstention_episodes_resolved": n_episodes_resolved,
            "abstention_episode_outcomes": outcomes,
        }

    results, failures, _ = run_trials("abstention", trial, seeds)

    writer.write_configuration(stress_config, [scenario])
    writer.write_seeds({"no_abstention": seeds, "abstention": seeds})
    writer.write_failed_runs({"abstention": failures})
    writer.write_raw_log("raw_log_abstention.json", [r["value"] for r in results])
    writer.write_run_summary([r["value"] for r in results])

    unmitigated_rates = [r["value"]["no_abstention_unmitigated_rate"] for r in results]
    trigger_rates = [r["value"]["abstention_trigger_rate"] for r in results]
    aggregated = head_to_head("no_abstention_unmitigated_rate", unmitigated_rates,
                               "abstention_mitigated_trigger_rate", trigger_rates, lower_is_better=True)
    aggregated["metric"] = ("fraction of degraded/critical-perception steps left unmitigated (no "
                             "abstention) vs given an active fallback decision (abstention)")
    aggregated["scope_note"] = compare_abstention.__doc__.strip().split("\n\n")[1]
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 6. centralized vs distributed fusion
# ---------------------------------------------------------------------
def compare_centralized_vs_distributed(config, writer, base_seed, trials):
    scenario = "high_packet_loss"
    seeds = seed_range(base_seed, trials)

    def run_arch(architecture):
        def trial(seed):
            run_config = copy.deepcopy(config)
            run_config["sim"]["seed"] = seed
            model = RadarLikeModel(run_config, scenario)
            detection_rows = model.run()
            true_by_step = _true_positions_by_step(detection_rows)
            fused_rows = build_fused_log(scenario, run_config, architecture=architecture, seed=seed)
            err = _nearest_object_error(fused_rows, true_by_step)
            delivered = [r.get("comm_messages_delivered") for r in fused_rows
                         if r.get("comm_messages_delivered") is not None]
            sent = [r.get("comm_messages") for r in fused_rows if r.get("comm_messages") is not None]
            delivery_rate = (sum(delivered) / sum(sent)) if sent and sum(sent) else None
            return {"mean_error": err["mean_error"], "n": err["n"], "delivery_rate": delivery_rate}
        return run_trials(f"centralized_vs_distributed/{architecture}", trial, seeds)

    results_central, failures_central, _ = run_arch(ARCHITECTURE_CENTRALIZED)
    results_dist, failures_dist, _ = run_arch(ARCHITECTURE_DISTRIBUTED)

    writer.write_configuration(config, [scenario])
    writer.write_seeds({"centralized": seeds, "distributed": seeds})
    writer.write_failed_runs({"centralized": failures_central, "distributed": failures_dist})
    writer.write_raw_log("raw_log_centralized.json", [r["value"] for r in results_central])
    writer.write_raw_log("raw_log_distributed.json", [r["value"] for r in results_dist])

    err_c = [r["value"]["mean_error"] for r in results_central]
    err_d = [r["value"]["mean_error"] for r in results_dist]
    writer.write_run_summary({"centralized_mean_error_per_trial": err_c,
                               "distributed_mean_error_per_trial": err_d})
    aggregated = head_to_head("centralized", err_c, "distributed", err_d)
    aggregated["metric"] = "nearest-true-object fused position error, meters, under 40% packet loss"
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 7. covariance fusion vs Covariance Intersection
# ---------------------------------------------------------------------
def compare_covariance_vs_ci(config, writer, base_seed, trials):
    base_scenario = "faulty_sensor_covariance_weighted_fusion"
    ci_scenario = "faulty_sensor_covariance_intersection_fusion"
    ci_config = clone_scenario(config, base_scenario, ci_scenario,
                                {"fusion_mode": COVARIANCE_INTERSECTION_FUSION},
                                description="Task 23: same faulty-sensor setup as "
                                            f"{base_scenario}, fused with Covariance Intersection "
                                            "instead of naive covariance-weighted (information) fusion.")
    seeds = seed_range(base_seed, trials)

    def run_mode(mode_config, scenario_name):
        def trial(seed):
            run_config = copy.deepcopy(mode_config)
            run_config["sim"]["seed"] = seed
            model = RadarLikeModel(run_config, scenario_name)
            detection_rows = model.run()
            true_by_step = _true_positions_by_step(detection_rows)
            fused_rows = build_fused_log(scenario_name, run_config, architecture=ARCHITECTURE_CENTRALIZED,
                                          seed=seed)
            err = _nearest_object_error(fused_rows, true_by_step)
            return {"mean_error": err["mean_error"], "n": err["n"]}
        return run_trials(f"covariance_vs_ci/{scenario_name}", trial, seeds)

    results_cov, failures_cov, _ = run_mode(config, base_scenario)
    results_ci, failures_ci, _ = run_mode(ci_config, ci_scenario)

    combined_config = copy.deepcopy(config)
    combined_config["scenarios"][ci_scenario] = ci_config["scenarios"][ci_scenario]
    writer.write_configuration(combined_config, [base_scenario, ci_scenario])
    writer.write_seeds({"covariance_weighted": seeds, "covariance_intersection": seeds})
    writer.write_failed_runs({"covariance_weighted": failures_cov, "covariance_intersection": failures_ci})
    writer.write_raw_log("raw_log_covariance_weighted.json", [r["value"] for r in results_cov])
    writer.write_raw_log("raw_log_covariance_intersection.json", [r["value"] for r in results_ci])

    err_cov = [r["value"]["mean_error"] for r in results_cov]
    err_ci = [r["value"]["mean_error"] for r in results_ci]
    writer.write_run_summary({"covariance_weighted_mean_error_per_trial": err_cov,
                               "covariance_intersection_mean_error_per_trial": err_ci})
    aggregated = head_to_head("covariance_weighted", err_cov, "covariance_intersection", err_ci)
    aggregated["metric"] = "nearest-true-object fused position error, meters"
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 9. normal radar vs ghost/aliasing radar conditions
# ---------------------------------------------------------------------
def compare_normal_vs_ghost_aliasing(config, writer, base_seed, trials):
    base_scenario = "correctly_calibrated_radar"
    ghost_scenario = "ghost_aliasing_radar_test"
    ghost_config = clone_scenario(
        config, base_scenario, ghost_scenario,
        {"extended_target_enabled": True, "mean_returns_per_target": 3.0, "return_spread_std": 0.6,
         "max_unambiguous_radial_velocity": 0.5, "doppler_aliasing_enabled": True,
         "fusion_mode": "trust_weighted_fusion"},
        description="Task 23: same radar/noise/clutter baseline as correctly_calibrated_radar, "
                    "plus multipath ghost returns (extended_target_enabled, mean_returns_per_target=3) "
                    "and Doppler aliasing (max_unambiguous_radial_velocity=0.5, "
                    "doppler_aliasing_enabled) - a phantom-detection + velocity-wrap stress case.")
    normal_config = clone_scenario(config, base_scenario, "normal_radar_reference",
                                    {"fusion_mode": "trust_weighted_fusion"})
    seeds = seed_range(base_seed, trials)

    def trial_for(cfg, scenario_name):
        def trial(seed):
            run_config = copy.deepcopy(cfg)
            run_config["sim"]["seed"] = seed
            model = RadarLikeModel(run_config, scenario_name)
            rows = model.run()
            n = len(rows)
            n_extended = sum(1 for r in rows if r.get("is_extended_return"))
            n_aliased = sum(1 for r in rows if r.get("doppler_ambiguity_flag"))
            true_by_step = _true_positions_by_step(rows)
            fused_rows = build_fused_log(scenario_name, run_config, architecture=ARCHITECTURE_CENTRALIZED,
                                          seed=seed)
            err = _nearest_object_error(fused_rows, true_by_step)
            return {"n_detection_rows": n, "ghost_return_rate": round(n_extended / n, 4) if n else None,
                    "aliasing_rate": round(n_aliased / n, 4) if n else None,
                    "mean_fused_error": err["mean_error"], "num_fused_tracks_per_step":
                        round(len(fused_rows) / max(1, len(set(r["time_step"] for r in fused_rows))), 3)
                        if fused_rows else None}
        return trial

    results_normal, failures_normal, _ = run_trials(
        "normal_vs_ghost_aliasing/normal", trial_for(normal_config, "normal_radar_reference"), seeds)
    results_ghost, failures_ghost, _ = run_trials(
        "normal_vs_ghost_aliasing/ghost_aliasing", trial_for(ghost_config, ghost_scenario), seeds)

    combined_config = copy.deepcopy(config)
    combined_config["scenarios"]["normal_radar_reference"] = normal_config["scenarios"]["normal_radar_reference"]
    combined_config["scenarios"][ghost_scenario] = ghost_config["scenarios"][ghost_scenario]
    writer.write_configuration(combined_config, ["normal_radar_reference", ghost_scenario])
    writer.write_seeds({"normal": seeds, "ghost_aliasing": seeds})
    writer.write_failed_runs({"normal": failures_normal, "ghost_aliasing": failures_ghost})
    writer.write_raw_log("raw_log_normal.json", [r["value"] for r in results_normal])
    writer.write_raw_log("raw_log_ghost_aliasing.json", [r["value"] for r in results_ghost])

    err_normal = [r["value"]["mean_fused_error"] for r in results_normal]
    err_ghost = [r["value"]["mean_fused_error"] for r in results_ghost]
    track_infl_normal = [r["value"]["num_fused_tracks_per_step"] for r in results_normal]
    track_infl_ghost = [r["value"]["num_fused_tracks_per_step"] for r in results_ghost]
    writer.write_run_summary({"normal_mean_error_per_trial": err_normal,
                               "ghost_aliasing_mean_error_per_trial": err_ghost})
    aggregated = head_to_head("normal", err_normal, "ghost_aliasing", err_ghost)
    aggregated["track_count_inflation"] = head_to_head(
        "normal", track_infl_normal, "ghost_aliasing", track_infl_ghost, lower_is_better=True)
    aggregated["ghost_return_rate_mean"] = _mean_stdev(
        [r["value"]["ghost_return_rate"] for r in results_ghost])[0]
    aggregated["aliasing_rate_mean"] = _mean_stdev(
        [r["value"]["aliasing_rate"] for r in results_ghost])[0]
    aggregated["metric"] = "nearest-true-object fused position error, meters, plus fused-track count inflation"
    writer.write_aggregated_results(aggregated)
    return aggregated


# ---------------------------------------------------------------------
# 5. no handoff vs handoff - STUB
# ---------------------------------------------------------------------
def handoff_stub(writer):
    """"Handoff" (transferring track responsibility/custody for a target
    between UAVs, e.g. as one leaves sensor range and another enters it)
    does not exist anywhere in this codebase - not as a scenario config
    key, not as a function, not as a concept mentioned in any module
    docstring. Implementing it would mean designing and building a new
    feature, not comparing two existing configurations, so it's left as
    a documented placeholder rather than faked."""
    note = {
        "status": "NOT IMPLEMENTED - stub only",
        "reason": handoff_stub.__doc__.strip(),
        "proposed_definition_for_review": (
            "A UAV whose track on a target is about to go stale (target leaving its "
            "radar_max_range/FOV, or its own dropout/latency degrading past a threshold) "
            "explicitly transfers track custody - last known state, covariance, id - to "
            "a UAV better positioned to continue it, rather than letting the track lapse "
            "and be re-acquired from scratch. 'No handoff' = current behavior (tracks lapse "
            "and are re-initialized independently per radar, exactly what "
            "tracking/radar_track_model.py's per-radar RadarTracker does today). "
            "'Handoff' would need: (1) a trigger condition, (2) a custody-transfer message "
            "format over communication_model.py, (3) a receiving-UAV track continuation path "
            "instead of RadarTracker spawning a new tentative track."
        ),
        "next_step": "Confirm this definition (or an alternative) before implementation.",
    }
    writer.write_json("handoff_stub.json", note)
    return note


# ---------------------------------------------------------------------
# report + orchestration
# ---------------------------------------------------------------------
REPORT_LINES = {}


def note(comparison, text):
    REPORT_LINES[comparison] = text


def main():
    parser = argparse.ArgumentParser(description="Task 23: final dependability experiments")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--base-seed", type=int, default=None)
    parser.add_argument("--trials", type=int, default=10,
                         help="Trials per arm for the pipeline-level comparisons (default: 10)")
    parser.add_argument("--calibration-runs", type=int, default=15,
                         help="Runs per arm for the calibration comparison (default: 15)")
    parser.add_argument("--mission-trials", type=int, default=20,
                         help="Trials per arm for the cheap mission-sim comparisons (default: 20)")
    parser.add_argument("--output-dir", default=os.path.join(_ROOT_DIR, "results", "dependability"))
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)
    base_seed = args.base_seed if args.base_seed is not None else config["sim"]["seed"]

    os.makedirs(args.output_dir, exist_ok=True)
    started_at = datetime.now(timezone.utc).isoformat()
    t0 = time.monotonic()

    comparisons = [
        ("calibrated_vs_uncalibrated_confidence", lambda w: compare_calibrated_vs_uncalibrated(
            config, w, base_seed, args.calibration_runs)),
        ("fixed_vs_dynamic_trust", lambda w: compare_fixed_vs_dynamic_trust(
            config, w, base_seed, args.trials)),
        ("fixed_vs_adaptive_safety_margin", lambda w: compare_safety_margin(
            config, w, base_seed, args.mission_trials)),
        ("no_abstention_vs_abstention", lambda w: compare_abstention(
            config, w, base_seed, args.trials)),
        ("no_handoff_vs_handoff", lambda w: handoff_stub(w)),
        ("centralized_vs_distributed_fusion", lambda w: compare_centralized_vs_distributed(
            config, w, base_seed, args.trials)),
        ("covariance_fusion_vs_covariance_intersection", lambda w: compare_covariance_vs_ci(
            config, w, base_seed, args.trials)),
        ("single_fault_vs_combined_fault", lambda w: compare_single_vs_combined_fault(
            config, w, base_seed, args.mission_trials)),
        ("normal_vs_ghost_aliasing_radar", lambda w: compare_normal_vs_ghost_aliasing(
            config, w, base_seed, args.trials)),
    ]

    all_results = {}
    for name, fn in comparisons:
        print(f"\n=== {name} ===")
        writer = DependabilityWriter(args.output_dir, name)
        t_c0 = time.monotonic()
        try:
            result = fn(writer)
            status = "OK"
        except Exception as exc:
            import traceback
            result = {"error": traceback.format_exc(limit=8)}
            status = "ERROR"
            print(f"  [COMPARISON FAILED] {name}: {exc}")
        runtime = time.monotonic() - t_c0
        all_results[name] = {"status": status, "runtime_seconds": round(runtime, 2), "result": result}
        print(f"  status={status} runtime={runtime:.1f}s")

    wall_clock = time.monotonic() - t0

    # --- top-level report (2-3 lines per comparison) ---------------------
    report_path = os.path.join(args.output_dir, "dependability_report.md")
    with open(report_path, "w") as f:
        f.write("#Final Dependability Experiments\n\n")
        f.write(f"Run at {started_at}, base_seed={base_seed}, wall_clock={wall_clock:.1f}s.\n\n")
        for name, entry in all_results.items():
            f.write(f"## {name}\n\n")
            if entry["status"] != "OK":
                f.write(f"FAILED to run - see run_metadata.json / {name}/ for details.\n\n")
                continue
            r = entry["result"]
            if name == "no_handoff_vs_handoff":
                f.write("Not implemented: no \"handoff\" concept exists in this codebase to compare. "
                        "See no_handoff_vs_handoff/handoff_stub.json for a proposed definition.\n\n")
                continue
            metric = r.get("metric", "")
            verdict = r.get("verdict", "see aggregated_results.json")
            f.write(f"{metric}. {verdict}\n\n")

    # --- overall metadata --------------------------------------------------
    num_failed_comparisons = sum(1 for e in all_results.values() if e["status"] != "OK")
    metadata = {
        "started_at": started_at,
        "wall_clock_seconds": round(wall_clock, 3),
        "base_seed": base_seed,
        "config_path": os.path.abspath(args.config),
        "trials_per_arm": {"pipeline": args.trials, "calibration_runs": args.calibration_runs,
                            "mission_sim": args.mission_trials},
        "comparisons_run": list(all_results.keys()),
        "comparisons_failed": num_failed_comparisons,
        "per_comparison_runtime_seconds": {k: v["runtime_seconds"] for k, v in all_results.items()},
        "output_dir": os.path.abspath(args.output_dir),
    }
    with open(os.path.join(args.output_dir, "run_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"\n=== DONE in {wall_clock:.1f}s | {len(comparisons) - num_failed_comparisons}/"
          f"{len(comparisons)} comparisons completed ===")
    print(f"Report -> {report_path}")
    print(f"Per-comparison artifacts -> {args.output_dir}/<comparison_name>/")

    return 1 if num_failed_comparisons else 0


if __name__ == "__main__":
    sys.exit(main())
