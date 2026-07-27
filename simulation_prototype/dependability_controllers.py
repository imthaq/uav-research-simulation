"""Task 17: five dependability controllers, built by composing the
existing Task 13/14/15 modules rather than reimplementing any of them.

    1. fixed_margin          - Task 13 safety_margin_mode="fixed"
    2. uncertainty_aware     - Task 13 safety_margin_mode="quality_monitor"
    3. abstention            - (2) + Task 14 SelectiveDecisionMaker
    4. handoff               - (2) + Task 15 PerceptionHandoffModel
    5. dynamic_trust_handoff - full radar/track/fusion pipeline with
                               trust_weighted_fusion + trust_adaptation
                               (Task 15 dynamic trust), + handoff on top

Controllers 3-5 need SelectiveDecisionMaker/PerceptionHandoffModel to see a
per-detection quality verdict and to override that step's motion when they
fire. Neither module was ever wired into Simulation's actual decision loop
(both only shipped with their own scripted-timeline CLI demos), so this
file is that missing wiring - not a rebuild of either module.

Integration approach: monkey-patch Simulation._steer (instance attribute,
not a subclass) so this works whether `sim` is a plain Simulation or the
`.sim` RadarLikeModel builds internally for run_radar_track_fusion_pipeline.
_steer already computes this step's uncertainty margin per detection via
_quality_level_for/_compute_safety_margin; attach_dependability_layer reuses
that same per-detection quality verdict rather than re-deriving it, then
lets the abstention/handoff decision override the velocity _steer already
picked.
"""
import copy
import os
import sys

_DEPENDABILITY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dependability")
if _DEPENDABILITY_DIR not in sys.path:
    sys.path.insert(0, _DEPENDABILITY_DIR)

from simple_swarm_sim import Simulation, normalize
from dependability.perception_quality_monitor import GOOD, DEGRADED, CRITICAL
from dependability.selective_swarm_decision import (
    SelectiveDecisionMaker, HOLD_POSITION, MARK_UNSAFE_TO_EXECUTE, REDUCE_SPEED,
    INCREASE_FORMATION_SPACING, REQUEST_PEER_TRACK as SEL_REQUEST_PEER_TRACK,
    TRANSFER_TO_CENTRALIZED_FUSION,
)
from dependability.perception_handoff_model import (
    PerceptionHandoffModel, SAFE_HOLD, REQUEST_PEER_TRACK, CENTRALIZED_FUSION_HANDOFF,
)

_LEVEL_ORDER = {GOOD: 0, DEGRADED: 1, CRITICAL: 2}


def _resources_for(sim):
    """available_resources convention both Task 14/15 modules share.

    radar_available is deliberately False here even though radar is this
    project's actual sensor: RADAR_ONLY_FALLBACK/RADAR_ONLY_FALLBACK-as-
    abstention-tier both mean "drop back to a second, independent local
    source" in the Task 14/15 design - and radar IS the (only) source
    that triggered the quality problem in the first place, so offering
    it back as its own fallback would just silently no-op every time it
    won the priority tie (which it always would, being "available"
    unconditionally) instead of ever reaching the fallbacks that actually
    change something. LiDAR is False for the reason already noted (no
    such sensor model exists at all). That leaves peer/centralized fused
    estimates - approximated from whether cross-UAV fusion is switched on
    for this run, since there's no separate centralized-fusion server
    modeled, only fuse_step() across whichever UAVs are reporting - and
    HOLD as the two real choices this project can actually back up."""
    return {
        "radar_available": False,
        "lidar_available": False,
        "peer_track_available": sim.fusion_mode != "no_fusion" and sim.num_uavs > 1,
        "centralized_fusion_available": sim.fusion_mode != "no_fusion",
        "formation_can_expand": True,
        "radar_update_due_in_steps": 0,
    }


def _worst_quality(sim, perceived):
    """The single worst-quality detection this step, via the same
    per-detection verdict _compute_safety_margin already derives for the
    margin calculation - one bad contact is enough to distrust the step,
    same rule _steer's own critical_action already follows."""
    worst_level, worst_score = None, None
    for d in perceived:
        level, score = sim._quality_level_for(d)
        if level is None:
            continue
        if worst_level is None or _LEVEL_ORDER[level] > _LEVEL_ORDER[worst_level]:
            worst_level, worst_score = level, score
    return worst_level, worst_score


def attach_dependability_layer(sim, abstention=False, handoff=False):
    """Wraps sim._steer to additionally run the Task 14 abstention ladder
    and/or the Task 15 handoff ladder, overriding this step's velocity
    when one of them fires. Call once, right after the Simulation (or
    RadarLikeModel.sim) is constructed, before running it."""
    sim.abstention_maker = SelectiveDecisionMaker() if abstention else None
    sim.handoff_model = PerceptionHandoffModel() if handoff else None
    sim.abstention_hold_count = 0
    sim.handoff_hold_count = 0

    original_steer = sim._steer

    def _resume_cautiously(i, vx, vy, speed_fraction):
        """A CRITICAL-quality contact already made the un-patched _steer
        discard its avoidance geometry entirely (critical_quality_action)
        and return (0, 0) - see simple_swarm_sim._steer. Scaling that zero
        down further is a no-op, which is exactly the case where an
        abstention/handoff fallback that trusts a peer/centralized/lower-
        speed alternative is supposed to differ from plain HOLD: resume
        goal-directed motion at a cautious fraction of speed instead of
        continuing to sit still."""
        if vx == 0.0 and vy == 0.0:
            tgt = sim.targets[i]
            gx, gy = normalize(tgt[0] - sim.pos[i][0], tgt[1] - sim.pos[i][1])
            return gx * sim.speed * speed_fraction, gy * sim.speed * speed_fraction
        return vx * speed_fraction, vy * speed_fraction

    def patched_steer(i, perceived):
        vx, vy, triggered_real, triggered_phantom, safety_info = original_steer(i, perceived)

        level, _score = _worst_quality(sim, perceived)
        if level is None:
            return vx, vy, triggered_real, triggered_phantom, safety_info

        t = getattr(sim, "_current_t", 0)
        signals = {"perception_quality_level": level}
        resources = _resources_for(sim)

        if sim.abstention_maker is not None:
            d = sim.abstention_maker.decide(i, t, signals, available_resources=resources)
            fallback = d["fallback_action"]
            if fallback in (HOLD_POSITION, MARK_UNSAFE_TO_EXECUTE):
                vx, vy = 0.0, 0.0
                sim.abstention_hold_count += 1
            elif fallback == REDUCE_SPEED:
                vx, vy = _resume_cautiously(i, vx, vy, 0.5)
            elif fallback == INCREASE_FORMATION_SPACING:
                cx = sum(p[0] for p in sim.pos) / sim.num_uavs
                cy = sum(p[1] for p in sim.pos) / sim.num_uavs
                ox, oy = normalize(sim.pos[i][0] - cx, sim.pos[i][1] - cy)
                svx, svy = normalize(vx + ox, vy + oy)
                vx, vy = svx * sim.speed, svy * sim.speed
            elif fallback in (SEL_REQUEST_PEER_TRACK, TRANSFER_TO_CENTRALIZED_FUSION):
                # Trusting a peer/centralized track enough to keep moving
                # cautiously rather than sitting in HOLD - same idea as
                # the handoff overlay's REQUEST_PEER_TRACK/CENTRALIZED_
                # FUSION_HANDOFF branch below.
                vx, vy = _resume_cautiously(i, vx, vy, 0.6)
            # WAIT_FOR_RADAR_UPDATE / *_ONLY_FALLBACK: this simplified path
            # has one perception stream per UAV (track-sourcing needs the
            # full radar/track/fusion pipeline - that's controller 5), so
            # these resume cautiously the same way REDUCE_SPEED does
            # rather than switching to a source this path doesn't model.

        if sim.handoff_model is not None:
            hd = sim.handoff_model.decide(i, t, signals, available_resources=resources)
            mode = hd["handoff_mode"]
            if mode == SAFE_HOLD:
                vx, vy = 0.0, 0.0
                sim.handoff_hold_count += 1
            elif mode in (REQUEST_PEER_TRACK, CENTRALIZED_FUSION_HANDOFF):
                # Trusting a peer's or centralized fusion's track enough
                # to keep moving, cautiously, rather than sitting in HOLD
                # while this UAV's own track is the problem.
                vx, vy = _resume_cautiously(i, vx, vy, 0.6)
            # NO_HANDOFF / *_ONLY_FALLBACK: no distinct sensor stream to
            # switch to here (see module docstring) - keep the margin
            # mode's own steering.

        return vx, vy, triggered_real, triggered_phantom, safety_info

    sim._steer = patched_steer


def _dependability_metrics(sim):
    """Metrics Task 17 asks for that only the attached decision modules
    can supply, on top of whatever sim._metrics() already returns.
    correct/failed handoff counts and recovery time are read straight off
    PerceptionHandoffModel.summary() rather than re-derived - that module
    already tracks exactly this (final_outcome_counts, avg resolved
    duration) per logged episode."""
    out = {
        "abstention_episodes": None, "abstention_hold_steps": sim.abstention_hold_count,
        "handoff_episodes": None, "handoff_hold_steps": sim.handoff_hold_count,
        "correct_handoff_count": None, "failed_handoff_count": None,
        "recovery_time_s": None,
    }
    if sim.abstention_maker is not None:
        s = sim.abstention_maker.summary()
        out["abstention_episodes"] = s["abstention_episodes_triggered"]
    if sim.handoff_model is not None:
        s = sim.handoff_model.summary()
        out["handoff_episodes"] = s["handoff_episodes_triggered"]
        out["correct_handoff_count"] = s["final_outcome_counts"].get("handoff_no_longer_needed", 0)
        out["failed_handoff_count"] = s["final_outcome_counts"].get("unresolved_at_simulation_end", 0)
        if s["avg_resolved_duration_steps"] is not None:
            out["recovery_time_s"] = round(s["avg_resolved_duration_steps"] * sim.dt, 3)
    return out


# ----------------------------------------------------------------------
# The five controllers themselves.
# ----------------------------------------------------------------------
CONTROLLERS = {
    "1_fixed_margin": dict(
        safety_margin_mode="fixed", abstention=False, handoff=False, dynamic_trust=False),
    "2_uncertainty_aware": dict(
        safety_margin_mode="quality_monitor", abstention=False, handoff=False, dynamic_trust=False),
    "3_uncertainty_aware_abstention": dict(
        safety_margin_mode="quality_monitor", abstention=True, handoff=False, dynamic_trust=False),
    "4_uncertainty_aware_handoff": dict(
        safety_margin_mode="quality_monitor", abstention=False, handoff=True, dynamic_trust=False),
    "5_dynamic_trust_handoff": dict(
        safety_margin_mode="quality_monitor", abstention=False, handoff=True, dynamic_trust=True),
}


def run_controller(controller_name, config, scenario_name, seed):
    """Runs one (controller, scenario, seed) combination and returns a
    single flat metrics dict - sim._metrics()/pipeline metrics plus the
    Task 17 dependability metrics above."""
    spec = CONTROLLERS[controller_name]
    cfg = copy.deepcopy(config)
    cfg["sim"]["seed"] = seed
    scn = cfg["scenarios"].setdefault(scenario_name, {})
    scn["safety_margin_mode"] = spec["safety_margin_mode"]

    sim = Simulation(cfg, scenario_name)
    attach_dependability_layer(sim, abstention=spec["abstention"], handoff=spec["handoff"])
    metrics = sim.run()
    metrics.update(_dependability_metrics(sim))
    metrics["controller"] = controller_name
    metrics["seed"] = seed
    return metrics


def run_dynamic_trust_controller(config, scenario_name, seed):
    """Controller 5: the real multi-sensor radar/track/trust-weighted-
    fusion pipeline (Task 15 dynamic trust), with the handoff layer
    attached to the same RadarLikeModel-owned Simulation instance the
    pipeline function already builds and drives - see
    run_radar_track_fusion_pipeline in simple_swarm_sim.py."""
    from models.radar_like_model import RadarLikeModel, _range_bearing_radial
    from tracking.radar_track_model import RadarTracker
    from fusion.fusion_model import fuse_step, TrustTracker
    import math

    cfg = copy.deepcopy(config)
    cfg["sim"]["seed"] = seed
    scn = cfg["scenarios"].setdefault(scenario_name, {})
    scn["safety_margin_mode"] = "quality_monitor"
    scn["fusion_mode"] = "trust_weighted_fusion"
    scn["trust_adaptation"] = {"enabled": True}

    model = RadarLikeModel(cfg, scenario_name)
    sim = model.sim
    attach_dependability_layer(sim, abstention=False, handoff=True)
    dt = sim.dt
    fusion_mode = sim.fusion_mode

    trackers = {i: RadarTracker(i) for i in range(sim.num_uavs)}
    obstacle_track_id = {}
    pending_estimates = {}
    trust_tracker = TrustTracker()

    t = 0
    for t in range(sim.max_steps):
        if all(sim.reached_goal):
            break
        model._capture = {}
        model._current_t = t

        true_dets_all, raw_percepts = sim.sense(t)
        active_this_step = list(raw_percepts.keys())
        raw_snapshot = {i: [dict(d) for d in raw_percepts[i]] for i in active_this_step}

        obstacle_track_row_by_uav = {}
        for i in active_this_step:
            dets_for_tracker = [{"x": d["x"], "y": d["y"], "confidence": d.get("confidence")}
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

        fused_clusters = fuse_step(list(obstacle_track_row_by_uav.values()), fusion_mode,
                                    trust_tracker=trust_tracker)
        track_id_to_uav = {tid: uav for uav, tid in obstacle_track_id.items()}
        fused_by_uav = {}
        for cluster in fused_clusters:
            for tid in cluster["source_ids"]:
                uav = track_id_to_uav.get(tid)
                if uav is not None:
                    fused_by_uav[uav] = cluster

        sim.decide_move(t, true_dets_all, raw_percepts, external_estimates=pending_estimates)
        pending_estimates = {i: {"x": c["x"], "y": c["y"], "confidence": c["confidence"],
                                  "num_sources": c["num_sources"],
                                  "position_variance": c.get("position_variance")}
                             for i, c in fused_by_uav.items()}

    metrics = sim._metrics(t)
    metrics.update(_dependability_metrics(sim))
    metrics["controller"] = "5_dynamic_trust_handoff"
    metrics["seed"] = seed
    return metrics


def run_any_controller(controller_name, config, scenario_name, seed):
    """Entry point experiments/controller_comparison.py actually calls -
    routes controller 5 to the real pipeline, everything else to the
    simplified Simulation path."""
    if CONTROLLERS[controller_name]["dynamic_trust"]:
        return run_dynamic_trust_controller(config, scenario_name, seed)
    return run_controller(controller_name, config, scenario_name, seed)
