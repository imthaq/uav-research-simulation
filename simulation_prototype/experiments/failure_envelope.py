"""Task 18: failure-envelope parameter sweeps.

One parameter at a time, holding everything else at its scenario-config
default, using controller 5 (dynamic_trust_handoff - the full radar/
track/fusion/handoff pipeline, and the only one of the Task 17
controllers with something to say about every parameter below) as the
single representative controller. Task 17 already covers cross-
controller comparison; re-running all five here per parameter would be
5x the cost for a question ("when does the SWARM become unsafe") that
doesn't depend on which controller is driving.

Each (parameter, value) point runs SEEDS_PER_POINT seeds and is
classified by the worst (most severe) outcome across those seeds:
    SAFETY FAILURE          - any real collision occurred
    MISSION FAILURE         - no collision, but the swarm didn't finish
                              (stuck in HOLD / handoff, or ran out of steps)
    DEGRADED BUT FUNCTIONAL - finished, no collision, but had near-misses
                              and/or formation error well above baseline
    SAFE                    - finished, no collision, no near-misses,
                              formation error close to baseline
"""
import copy
import csv
import math
import os
import random
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

import json
from dependability_controllers import attach_dependability_layer, _dependability_metrics
from models.radar_like_model import RadarLikeModel
from tracking.radar_track_model import RadarTracker
from fusion.fusion_model import fuse_step, TrustTracker

SEEDS_PER_POINT = 2
OUT_PATH = os.path.join(_ROOT_DIR, "results", "swarm_failure_envelope.csv")


def run_stress_pipeline(config, scenario_name, seed, num_faulty_uavs=0,
                         fault_duration_steps=0, fault_start_step=None,
                         packet_loss_prob=0.0, disagreement_bias_std=0.0,
                         registration_bias=0.0, comm_delay_steps=0,
                         instrument=False):
    """run_dynamic_trust_controller (Task 17) plus the fault-injection
    hooks the failure envelope / combined-fault sweeps need and nothing
    else does. instrument=True (Task 20 only) additionally times the
    fuse_step calls and counts messages/tracks - off by default so it
    costs Task 18/19 nothing."""
    import time
    cfg = copy.deepcopy(config)
    cfg["sim"]["seed"] = seed
    scn = cfg["scenarios"].setdefault(scenario_name, {})
    scn["safety_margin_mode"] = "quality_monitor"
    scn.setdefault("fusion_mode", "trust_weighted_fusion")
    scn.setdefault("trust_adaptation", {"enabled": True})

    model = RadarLikeModel(cfg, scenario_name)
    sim = model.sim
    attach_dependability_layer(sim, abstention=False, handoff=True)
    dt = sim.dt
    fusion_mode = sim.fusion_mode

    if fault_start_step is None:
        fault_start_step = sim.max_steps // 3
    faulty_uavs = set(range(min(num_faulty_uavs, sim.num_uavs)))
    rng = random.Random(seed * 7919 + 1)
    bias = {i: (rng.gauss(0, disagreement_bias_std), rng.gauss(0, disagreement_bias_std))
            for i in range(sim.num_uavs)}
    reg_dx = registration_bias / math.sqrt(2)
    reg_dy = registration_bias / math.sqrt(2)

    trackers = {i: RadarTracker(i) for i in range(sim.num_uavs)}
    obstacle_track_id = {}
    pending_estimates = {}
    trust_tracker = TrustTracker()
    delay_queue = []  # (release_step, estimates_dict), used when comm_delay_steps > 0
    fusion_time_total = 0.0
    message_count = 0
    consistency_spreads = []

    t = 0
    for t in range(sim.max_steps):
        if all(sim.reached_goal):
            break
        model._capture = {}
        model._current_t = t
        in_fault_window = fault_start_step <= t < fault_start_step + fault_duration_steps

        true_dets_all, raw_percepts = sim.sense(t)
        active_this_step = [i for i in raw_percepts
                             if not (in_fault_window and i in faulty_uavs)]
        raw_snapshot = {i: [dict(d) for d in raw_percepts[i]] for i in active_this_step}

        obstacle_track_row_by_uav = {}
        for i in active_this_step:
            bx, by = bias[i]
            dets_for_tracker = [{"x": d["x"] + bx + reg_dx, "y": d["y"] + by + reg_dy,
                                  "confidence": d.get("confidence")}
                                 for d in raw_snapshot[i]]
            track_rows = trackers[i].update(t, dets_for_tracker, dt)
            obstacle_det = next((d for d in raw_snapshot[i] if d.get("id") == "obstacle_0"), None)
            if obstacle_det is not None and track_rows:
                match = min(track_rows, key=lambda r: math.hypot(
                    r["est_x"] - obstacle_det["x"], r["est_y"] - obstacle_det["y"]))
                obstacle_track_id[i] = match["track_id"]
            obs_row = next((r for r in track_rows if r["track_id"] == obstacle_track_id.get(i)), None)
            if obs_row is None:
                obstacle_track_id.pop(i, None)
            else:
                obstacle_track_row_by_uav[i] = obs_row

        if instrument:
            message_count += len(obstacle_track_row_by_uav)
            rows = list(obstacle_track_row_by_uav.values())
            if len(rows) >= 2:
                xs = [r["est_x"] for r in rows]
                ys = [r["est_y"] for r in rows]
                mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
                spread = (sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys))
                          / len(xs)) ** 0.5
                consistency_spreads.append(spread)
            t0 = time.perf_counter()
        fused_clusters = fuse_step(list(obstacle_track_row_by_uav.values()), fusion_mode,
                                    trust_tracker=trust_tracker)
        if instrument:
            fusion_time_total += time.perf_counter() - t0
        track_id_to_uav = {tid: uav for uav, tid in obstacle_track_id.items()}
        fused_by_uav = {}
        for cluster in fused_clusters:
            for tid in cluster["source_ids"]:
                uav = track_id_to_uav.get(tid)
                if uav is not None:
                    fused_by_uav[uav] = cluster

        sim.decide_move(t, true_dets_all, raw_percepts, external_estimates=pending_estimates)
        fresh_estimates = {i: {"x": c["x"], "y": c["y"], "confidence": c["confidence"],
                                "num_sources": c["num_sources"],
                                "position_variance": c.get("position_variance")}
                           for i, c in fused_by_uav.items()
                           if rng.random() >= packet_loss_prob}
        if comm_delay_steps > 0:
            delay_queue.append((t + 1 + comm_delay_steps, fresh_estimates))
            pending_estimates = {}
            while delay_queue and delay_queue[0][0] <= t + 1:
                pending_estimates.update(delay_queue.pop(0)[1])
        else:
            pending_estimates = fresh_estimates

    metrics = sim._metrics(t)
    metrics.update(_dependability_metrics(sim))
    if instrument:
        steps_run = t + 1
        metrics["fusion_update_time_ms"] = round(1000 * fusion_time_total / steps_run, 4)
        metrics["message_count"] = message_count
        metrics["tracks_created"] = sum(trk._next_track_num - 1 for trk in trackers.values())
        metrics["distributed_consistency_std"] = (
            round(sum(consistency_spreads) / len(consistency_spreads), 4)
            if consistency_spreads else None)
    return metrics


# Each entry: parameter name, list of (value, scn_overrides_or_stress_kwargs).
# scn_overrides land in cfg["scenarios"][name]; stress kwargs go straight to
# run_stress_pipeline. Fixed companion values for the two fault axes are
# chosen so each isolates its own effect (see module docstring).
def _scn_axis(param, key, values):
    return (param, [(v, {"scn": {key: v}, "stress": {}}) for v in values])


AXES = [
    _scn_axis("P_D", "radar_detection_probability", [1.0, 0.7, 0.5, 0.3, 0.1]),
    _scn_axis("P_FA", "radar_false_alarm_probability", [0.0, 0.1, 0.2, 0.3, 0.4]),
    _scn_axis("clutter_intensity", "radar_clutter_density", [0.0, 0.5, 1.0, 2.0, 3.0]),
    _scn_axis("covariance_magnitude", "radar_range_noise_std", [0.2, 0.5, 1.0, 2.0, 3.0]),
    _scn_axis("calibration_error", "radar_confidence_error", [0.0, 0.1, 0.2, 0.35, 0.5]),
    _scn_axis("latency", "radar_latency_steps", [0, 1, 2, 5, 10]),
    _scn_axis("dropout", "radar_dropout_probability", [0.0, 0.05, 0.1, 0.2, 0.4]),
    ("registration_error", [(v, {"scn": {}, "stress": {"registration_bias": v}})
                             for v in [0.0, 0.5, 1.0, 2.0, 4.0]]),
    ("packet_loss", [(v, {"scn": {}, "stress": {"packet_loss_prob": v}})
                      for v in [0.0, 0.1, 0.2, 0.4, 0.7]]),
    ("sensor_disagreement", [(v, {"scn": {}, "stress": {"disagreement_bias_std": v}})
                              for v in [0.0, 0.5, 1.0, 2.0, 4.0]]),
    ("num_faulty_uavs", [(v, {"scn": {}, "stress": {"num_faulty_uavs": v,
                                                     "fault_duration_steps": 100}})
                          for v in [0, 1, 2, 3]]),
    ("fault_duration", [(v, {"scn": {}, "stress": {"num_faulty_uavs": 2,
                                                    "fault_duration_steps": v}})
                         for v in [0, 20, 50, 100, 200]]),
]

_SEVERITY = ["SAFE", "DEGRADED BUT FUNCTIONAL", "MISSION FAILURE", "SAFETY FAILURE"]


def classify(m, baseline_near_miss, baseline_formation_error):
    if m["collision_count"] > 0:
        return "SAFETY FAILURE"
    if not m["mission_success"]:
        return "MISSION FAILURE"
    degraded = (
        (baseline_near_miss is not None and m["near_miss_count"] > baseline_near_miss * 1.5)
        or (m["avg_formation_error"] is not None and baseline_formation_error is not None
            and m["avg_formation_error"] > baseline_formation_error * 1.5)
        or (m["handoff_hold_steps"] or 0) > 0
    )
    return "DEGRADED BUT FUNCTIONAL" if degraded else "SAFE"


def run_point(config, scn_overrides, stress_kwargs, seeds):
    results = []
    for seed in seeds:
        cfg = copy.deepcopy(config)
        scn = dict(cfg["scenarios"].get("baseline", {}))
        scn.update(scn_overrides)
        cfg["scenarios"] = dict(cfg["scenarios"])
        cfg["scenarios"]["_failure_envelope_point"] = scn
        results.append(run_stress_pipeline(cfg, "_failure_envelope_point", seed, **stress_kwargs))
    return results


def main():
    config = json.load(open(os.path.join(_ROOT_DIR, "simulation_config.json")))
    seeds = list(range(1, SEEDS_PER_POINT + 1))

    baseline_runs = run_point(config, {}, {}, seeds)
    baseline_near_miss = sum(r["near_miss_count"] for r in baseline_runs) / len(baseline_runs)
    formation_vals = [r["avg_formation_error"] for r in baseline_runs if r["avg_formation_error"]]
    baseline_formation_error = sum(formation_vals) / len(formation_vals) if formation_vals else None

    rows = []
    for param, points in AXES:
        for value, overrides in points:
            runs = run_point(config, overrides["scn"], overrides["stress"], seeds)
            classes = [classify(r, baseline_near_miss, baseline_formation_error) for r in runs]
            worst = max(classes, key=_SEVERITY.index)
            n = len(runs)
            rows.append({
                "parameter": param,
                "value": value,
                "seeds_run": n,
                "mission_success_rate": round(sum(r["mission_success"] for r in runs) / n, 3),
                "collision_count_mean": round(sum(r["collision_count"] for r in runs) / n, 3),
                "near_miss_count_mean": round(sum(r["near_miss_count"] for r in runs) / n, 3),
                "avg_formation_error_mean": round(
                    sum(r["avg_formation_error"] or 0 for r in runs) / n, 3),
                "handoff_hold_steps_mean": round(
                    sum(r["handoff_hold_steps"] or 0 for r in runs) / n, 3),
                "classification": worst,
            })
            print(f"{param}={value}: {worst}")

    fieldnames = ["parameter", "value", "seeds_run", "mission_success_rate",
                  "collision_count_mean", "near_miss_count_mean",
                  "avg_formation_error_mean", "handoff_hold_steps_mean", "classification"]
    with open(OUT_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {len(rows)} rows to {OUT_PATH}")


if __name__ == "__main__":
    main()