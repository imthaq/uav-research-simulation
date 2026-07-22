"""Task 14: selective decision-making and abstention.

Sits directly downstream of perception_quality_monitor.py (Task 12). Where
Task 13's safety-margin logic in simple_swarm_sim.py only ever widens/
slows a UAV's *aggressive* (goal-seeking + reactive-avoidance) decision,
this module gives a UAV permission to abstain from that aggressive
decision altogether when perception quality is insufficient, and choose
from a graded menu of fallback behaviors instead of forcing a single
one-size-fits-all response:

    HOLD position
    reduce speed
    increase formation spacing
    wait for another radar update
    use radar-only fallback
    use LiDAR-only fallback at short range
    request another UAV's track
    transfer decision to centralized fusion
    mark decision as unsafe to execute

Severity picks the tier, available resources pick the specific fallback
within that tier, and a stuck-in-HOLD-too-long escalation makes sure the
system doesn't loop on a fallback that isn't actually resolving anything -
it flags the decision as unsafe to execute (for a human/higher-level
authority) instead. Every state transition is logged with what triggered
it, why, what was selected, how long the episode has lasted, and - once
it ends - how it ended.

Like perception_quality_monitor.py, this module NEVER uses true position
error or any other ground-truth field - there is no such parameter
anywhere in its public API, by design (see _self_check).
"""
import argparse
import csv
import json

from perception_quality_monitor import PerceptionQualityMonitor, GOOD, DEGRADED, CRITICAL

# ----------------------------------------------------------------------
# Fallback actions - one constant per bullet in the Task 14 spec, so a
# caller can match on these directly instead of guessing at strings.
# ----------------------------------------------------------------------
HOLD_POSITION = "hold_position"
REDUCE_SPEED = "reduce_speed"
INCREASE_FORMATION_SPACING = "increase_formation_spacing"
WAIT_FOR_RADAR_UPDATE = "wait_for_radar_update"
RADAR_ONLY_FALLBACK = "radar_only_fallback"
LIDAR_ONLY_FALLBACK_SHORT_RANGE = "lidar_only_fallback_short_range"
REQUEST_PEER_TRACK = "request_peer_track"
TRANSFER_TO_CENTRALIZED_FUSION = "transfer_to_centralized_fusion"
MARK_UNSAFE_TO_EXECUTE = "mark_unsafe_to_execute"

FALLBACK_ACTIONS = (
    HOLD_POSITION, REDUCE_SPEED, INCREASE_FORMATION_SPACING, WAIT_FOR_RADAR_UPDATE,
    RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK_SHORT_RANGE, REQUEST_PEER_TRACK,
    TRANSFER_TO_CENTRALIZED_FUSION, MARK_UNSAFE_TO_EXECUTE,
)

# DEGRADED prefers a targeted mitigation (more room, fresher data, a
# corroborating peer) when one is actually available, and only falls back
# to a blanket speed reduction - the guaranteed, resource-free option -
# when none of those apply. reduce_speed must stay last: it's always
# "available", so ordering it first would mean it's chosen unconditionally
# and the more specific fallbacks below it would never fire.
DEGRADED_FALLBACK_PRIORITY = (
    INCREASE_FORMATION_SPACING, WAIT_FOR_RADAR_UPDATE, REQUEST_PEER_TRACK, REDUCE_SPEED,
)

# CRITICAL escalates to fallbacks that change *what the decision is based
# on* (alternate sensor, a peer's track, or handing off to central fusion
# entirely) before falling back to simply not moving. MARK_UNSAFE_TO_EXECUTE
# is deliberately excluded from this list - it is never "the next thing to
# try", only an escalation out of a HOLD that isn't resolving (see
# _select_fallback).
CRITICAL_FALLBACK_PRIORITY = (
    RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK_SHORT_RANGE, REQUEST_PEER_TRACK,
    TRANSFER_TO_CENTRALIZED_FUSION, HOLD_POSITION,
)

# ----------------------------------------------------------------------
# Reasons - derived from *which* PerceptionQualityMonitor signal(s) are
# worst, so "abstention triggered" always comes with a concrete, specific
# cause rather than just "quality was low".
# ----------------------------------------------------------------------
NO_SIGNAL_REASON = "no_perception_signal_available"

REASON_LABELS = {
    "covariance": "high_track_covariance",
    "age": "track_too_young",
    "missed_updates": "too_many_missed_updates",
    "agreement": "low_sensor_agreement",
    "innovation": "high_innovation_residual",
    "calibration": "poor_confidence_calibration",
    "communication_age": "stale_relayed_track",
    "dropout_rate": "high_sensor_dropout_rate",
    "trust": "low_dynamic_trust",
}

# A signal below this (on the monitor's normalized [0, 1] per-signal
# scale) is "concerning enough to mention", independent of whether it
# happens to be the single worst one.
CONCERNING_SIGNAL_THRESHOLD = 0.5
MAX_CONTRIBUTING_SIGNALS = 3

# ----------------------------------------------------------------------
# Final outcomes - the fixed, small set an episode can resolve to. Extra
# per-episode detail (peak severity reached, whether it ever escalated to
# MARK_UNSAFE_TO_EXECUTE) rides along as separate fields on the log line
# rather than being folded into more outcome strings.
# ----------------------------------------------------------------------
OUTCOME_QUALITY_RECOVERED = "quality_recovered"
OUTCOME_UNRESOLVED_AT_END = "unresolved_at_simulation_end"
OUTCOME_SAFETY_EVENT = "safety_event_during_abstention"

# Event types that appear in the structured log (self.log / to_csv).
EVENT_TRIGGERED = "abstention_triggered"
EVENT_ESCALATED = "abstention_escalated"
EVENT_FALLBACK_SELECTED = "fallback_selected"
EVENT_SAFETY_EVENT = "safety_event_during_abstention"
EVENT_RESOLVED = "abstention_resolved"

LOG_FIELDS = [
    "uav_id", "step", "event",
    "abstention_triggered", "reason", "contributing_signals",
    "perception_quality_level", "perception_quality_score",
    "selected_fallback", "duration_steps", "final_outcome",
    "peak_level", "reached_mark_unsafe", "safety_event",
]


def _worst_signals(per_signal, limit=MAX_CONTRIBUTING_SIGNALS, threshold=CONCERNING_SIGNAL_THRESHOLD):
    """Available (non-None) per-signal scores at/below `threshold`,
    worst first, capped at `limit` entries."""
    scored = [(name, score) for name, score in per_signal.items() if score is not None]
    scored.sort(key=lambda kv: kv[1])
    return [(name, score) for name, score in scored if score <= threshold][:limit]


class SelectiveDecisionMaker:
    """Turns a PerceptionQualityMonitor verdict into an abstain/proceed
    decision, and - when abstaining - a specific fallback action, per UAV,
    with full lifecycle logging of the resulting abstention episode.

    One instance is meant to be shared across a whole run (it tracks one
    open episode at most per uav_id), not recreated every step.
    """

    def __init__(self, quality_monitor=None, max_hold_steps=15,
                 wait_horizon_steps=3, resume_threshold=None, weights=None):
        self.quality_monitor = quality_monitor or PerceptionQualityMonitor(weights=weights)
        # How many consecutive steps a HOLD_POSITION fallback is allowed
        # to keep being re-selected within one episode before it's judged
        # stuck and escalated to MARK_UNSAFE_TO_EXECUTE instead.
        self.max_hold_steps = max_hold_steps
        # wait_for_radar_update is only offered as a fallback if a fresh
        # update is actually due within this many steps - waiting for
        # something arbitrarily far off is worse than the alternatives.
        self.wait_horizon_steps = wait_horizon_steps
        # Quality must recover to at least this composite score (default:
        # the monitor's own GOOD threshold) before an episode is closed as
        # "recovered" - kept as its own knob so a caller can require some
        # hysteresis above the monitor's raw GOOD cut if flapping matters.
        self.resume_threshold = (resume_threshold if resume_threshold is not None
                                  else self.quality_monitor.good_threshold)
        self._episodes = {}   # uav_id -> open episode dict, absent if none
        self.log = []         # flat, append-only list of logged lifecycle events

    # -- internal helpers -------------------------------------------------

    def _reason_for(self, composite, per_signal):
        if composite is None:
            return NO_SIGNAL_REASON, []
        worst = _worst_signals(per_signal)
        if not worst:
            # Composite landed below GOOD despite no single signal looking
            # concerning on its own - a diffuse, many-small-issues case.
            return "diffuse_low_quality", []
        primary_name, _ = worst[0]
        reason = REASON_LABELS.get(primary_name, primary_name)
        contributing = [{"signal": name, "score": round(score, 3)} for name, score in worst]
        return reason, contributing

    def _select_fallback(self, level, episode, resources):
        resources = resources or {}

        def available(action):
            if action == REDUCE_SPEED:
                return True
            if action == INCREASE_FORMATION_SPACING:
                return bool(resources.get("formation_can_expand", False))
            if action == WAIT_FOR_RADAR_UPDATE:
                due = resources.get("radar_update_due_in_steps")
                return due is not None and due <= self.wait_horizon_steps
            if action == RADAR_ONLY_FALLBACK:
                return bool(resources.get("radar_available", False))
            if action == LIDAR_ONLY_FALLBACK_SHORT_RANGE:
                return bool(resources.get("lidar_available", False)) and bool(
                    resources.get("lidar_short_range", False))
            if action == REQUEST_PEER_TRACK:
                return bool(resources.get("peer_track_available", False))
            if action == TRANSFER_TO_CENTRALIZED_FUSION:
                return bool(resources.get("centralized_fusion_available", False))
            if action == HOLD_POSITION:
                return True
            return False

        # A HOLD that's been re-selected max_hold_steps times running
        # within this episode isn't resolving anything - re-trying it
        # forever just silently sits there, so escalate to flagging the
        # decision as unsafe to execute instead of continuing to hold.
        if (episode["current_fallback"] == HOLD_POSITION
                and episode["hold_streak"] >= self.max_hold_steps):
            return MARK_UNSAFE_TO_EXECUTE

        priority = DEGRADED_FALLBACK_PRIORITY if level == DEGRADED else CRITICAL_FALLBACK_PRIORITY
        for action in priority:
            if available(action):
                return action
        return HOLD_POSITION  # guaranteed last resort - always "available"

    def _log_event(self, uav_id, t, event, *, reason=None, contributing=None, level=None,
                    composite=None, fallback=None, duration_steps=None, final_outcome=None,
                    peak_level=None, reached_mark_unsafe=None, safety_event=None):
        entry = {
            "uav_id": uav_id,
            "step": t,
            "event": event,
            "abstention_triggered": event == EVENT_TRIGGERED,
            "reason": reason,
            "contributing_signals": contributing or [],
            "perception_quality_level": level,
            "perception_quality_score": round(composite, 3) if composite is not None else None,
            "selected_fallback": fallback,
            "duration_steps": duration_steps,
            "final_outcome": final_outcome,
            "peak_level": peak_level,
            "reached_mark_unsafe": reached_mark_unsafe,
            "safety_event": safety_event,
        }
        self.log.append(entry)
        return entry

    def _close_episode(self, uav_id, t, level, composite, *, forced=False):
        episode = self._episodes.pop(uav_id, None)
        if episode is None:
            return None
        duration = t - episode["start_step"]
        if episode["had_safety_event"]:
            outcome = OUTCOME_SAFETY_EVENT
        elif forced:
            outcome = OUTCOME_UNRESOLVED_AT_END
        else:
            outcome = OUTCOME_QUALITY_RECOVERED
        return self._log_event(
            uav_id, t, EVENT_RESOLVED, reason=episode["reason"], level=level, composite=composite,
            fallback=episode["current_fallback"], duration_steps=duration, final_outcome=outcome,
            peak_level=episode["peak_level"],
            reached_mark_unsafe=episode["current_fallback"] == MARK_UNSAFE_TO_EXECUTE
            or episode["reached_mark_unsafe"])

    # -- public API ---------------------------------------------------

    def decide(self, uav_id, t, signals, available_resources=None, safety_event=None):
        """Evaluates one UAV's current perception signals (same shape
        PerceptionQualityMonitor.evaluate accepts) and returns a decision
        dict for this step:

            {"uav_id", "step", "abstain", "level", "composite_score",
             "reason", "contributing_signals", "fallback_action",
             "duration_steps"}

        abstain=False means perception quality is GOOD: proceed with the
        normal aggressive (goal-seeking + reactive-avoidance) decision,
        fallback_action is None. abstain=True means the caller should
        execute fallback_action instead of its normal decision this step.

        available_resources: optional dict describing what's currently on
        hand to fall back to - radar_available, lidar_available,
        lidar_short_range, peer_track_available,
        centralized_fusion_available, formation_can_expand,
        radar_update_due_in_steps. Anything omitted is treated as
        unavailable, so with no resource info at all the only fallbacks
        ever selected are reduce_speed (DEGRADED) or hold_position
        (CRITICAL) - a safe default rather than assuming resources exist.

        safety_event: optional string ("collision" / "near_miss" / any
        caller-defined label) if one occurred THIS step while already
        abstaining - always logged immediately, and remembered for the
        episode's final_outcome even if quality recovers afterward.
        """
        level, composite, per_signal = self.quality_monitor.evaluate(signals)
        episode = self._episodes.get(uav_id)

        if level == GOOD and composite is not None and composite >= self.resume_threshold:
            if episode is not None:
                self._close_episode(uav_id, t, level, composite, forced=False)
            return {
                "uav_id": uav_id, "step": t, "abstain": False, "level": level,
                "composite_score": composite, "reason": None, "contributing_signals": [],
                "fallback_action": None, "duration_steps": 0,
            }

        reason, contributing = self._reason_for(composite, per_signal)

        if episode is None:
            episode = {
                "start_step": t, "reason": reason, "current_fallback": None,
                "hold_streak": 0, "peak_level": level, "had_safety_event": False,
                "reached_mark_unsafe": False,
            }
            self._episodes[uav_id] = episode
            self._log_event(uav_id, t, EVENT_TRIGGERED, reason=reason, contributing=contributing,
                             level=level, composite=composite, duration_steps=0)
        elif level == CRITICAL and episode["peak_level"] != CRITICAL:
            episode["peak_level"] = CRITICAL
            self._log_event(uav_id, t, EVENT_ESCALATED, reason=reason, contributing=contributing,
                             level=level, composite=composite, fallback=episode["current_fallback"],
                             duration_steps=t - episode["start_step"], peak_level=level)

        fallback = self._select_fallback(level, episode, available_resources)
        episode["hold_streak"] = (episode["hold_streak"] + 1
                                   if fallback == HOLD_POSITION and episode["current_fallback"] == HOLD_POSITION
                                   else (1 if fallback == HOLD_POSITION else 0))
        if fallback == MARK_UNSAFE_TO_EXECUTE:
            episode["reached_mark_unsafe"] = True
        if fallback != episode["current_fallback"]:
            self._log_event(uav_id, t, EVENT_FALLBACK_SELECTED, reason=reason, contributing=contributing,
                             level=level, composite=composite, fallback=fallback,
                             duration_steps=t - episode["start_step"], peak_level=episode["peak_level"])
        episode["current_fallback"] = fallback
        episode["reason"] = reason

        if safety_event is not None:
            episode["had_safety_event"] = True
            self._log_event(uav_id, t, EVENT_SAFETY_EVENT, reason=reason, contributing=contributing,
                             level=level, composite=composite, fallback=fallback,
                             duration_steps=t - episode["start_step"], peak_level=episode["peak_level"],
                             safety_event=safety_event)

        return {
            "uav_id": uav_id, "step": t, "abstain": True, "level": level,
            "composite_score": composite, "reason": reason, "contributing_signals": contributing,
            "fallback_action": fallback, "duration_steps": t - episode["start_step"] + 1,
        }

    def abstaining(self, uav_id):
        """True if uav_id currently has an open abstention episode."""
        return uav_id in self._episodes

    def close_all(self, t):
        """Force-closes every still-open episode as of step t (e.g. at
        the end of a simulation run), logging OUTCOME_UNRESOLVED_AT_END
        (or OUTCOME_SAFETY_EVENT if one occurred during the episode) for
        each. Returns the list of resulting log entries."""
        closed = []
        for uav_id in list(self._episodes):
            episode = self._episodes[uav_id]
            entry = self._close_episode(uav_id, t, episode["peak_level"], None, forced=True)
            if entry is not None:
                closed.append(entry)
        return closed

    def summary(self):
        """Aggregate stats over every logged event so far: counts per
        fallback action actually selected, per reason, per final outcome,
        and the average duration_steps of resolved episodes. Useful for
        comparing abstention behavior across scenarios/runs the same way
        Simulation._metrics() summarizes a run."""
        fallback_counts = {}
        reason_counts = {}
        outcome_counts = {}
        resolved_durations = []
        triggered = 0
        for e in self.log:
            if e["event"] == EVENT_TRIGGERED:
                triggered += 1
                if e["reason"]:
                    reason_counts[e["reason"]] = reason_counts.get(e["reason"], 0) + 1
            if e["event"] == EVENT_FALLBACK_SELECTED and e["selected_fallback"]:
                fallback_counts[e["selected_fallback"]] = fallback_counts.get(e["selected_fallback"], 0) + 1
            if e["event"] == EVENT_RESOLVED:
                if e["final_outcome"]:
                    outcome_counts[e["final_outcome"]] = outcome_counts.get(e["final_outcome"], 0) + 1
                if e["duration_steps"] is not None:
                    resolved_durations.append(e["duration_steps"])
        return {
            "abstention_episodes_triggered": triggered,
            "fallback_action_counts": fallback_counts,
            "trigger_reason_counts": reason_counts,
            "final_outcome_counts": outcome_counts,
            "avg_resolved_duration_steps": (
                round(sum(resolved_durations) / len(resolved_durations), 3)
                if resolved_durations else None),
            "still_open_episode_count": len(self._episodes),
        }

    def to_csv(self, path):
        """Writes self.log to a CSV file with a stable column order
        (LOG_FIELDS), matching the rest of this project's CSV-log style.
        contributing_signals is JSON-encoded into its cell since it's a
        list of dicts, not a scalar."""
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
            writer.writeheader()
            for entry in self.log:
                row = dict(entry)
                row["contributing_signals"] = json.dumps(row["contributing_signals"])
                writer.writerow(row)

    # -- convenience wrapper tied to this project's detection-dict shape --

    def decide_for_detection(self, uav_id, t, detection, num_uavs=None,
                              available_resources=None, safety_event=None):
        """Convenience wrapper mirroring
        PerceptionQualityMonitor.evaluate_track_row / simple_swarm_sim.py's
        Simulation._quality_level_for: pulls what a perceived-detection
        dict from simple_swarm_sim.py's _steer already carries
        (confidence, position_variance/covariance_trace,
        fusion_contributors) into signals, then calls decide() as normal.
        detection may be None (no detection at all this step -> no usable
        perception signal -> abstain via NO_SIGNAL_REASON, same as an
        empty signals dict)."""
        detection = detection or {}
        covariance = detection.get("position_variance", detection.get("covariance_trace"))
        signals = {
            "track_covariance": covariance,
            "current_trust_value": detection.get("confidence"),
            "sensor_agreement": (
                min(detection["fusion_contributors"] / max(num_uavs or 2, 2), 1.0)
                if detection.get("is_fused") and detection.get("fusion_contributors") else None),
        }
        return self.decide(uav_id, t, signals, available_resources=available_resources,
                            safety_event=safety_event)


# ----------------------------------------------------------------------
# CLI demo
# ----------------------------------------------------------------------
# A scripted timeline for one synthetic UAV, walking GOOD -> DEGRADED ->
# CRITICAL -> (stuck holding long enough to escalate) -> back to GOOD, so
# running this file directly demonstrates every event type end to end
# without needing a live simulation wired up. Not test cases (those live
# in _self_check).
def _demo_timeline():
    timeline = []
    # Steps 0-2: healthy.
    for t in range(0, 3):
        timeline.append((t, {
            "track_covariance": 0.2, "track_age": 10, "current_trust_value": 0.95,
        }, {}, None))
    # Steps 3-6: degrading (covariance rising) -> DEGRADED, reduce_speed.
    for t in range(3, 7):
        timeline.append((t, {
            "track_covariance": 6.0, "track_age": 10, "current_trust_value": 0.5,
        }, {}, None))
    # Steps 7-9: quality collapses -> CRITICAL. No alternate resources
    # available at first, so it falls back to hold_position.
    for t in range(7, 10):
        timeline.append((t, {
            "track_covariance": 40.0, "track_age": 10, "current_trust_value": 0.05,
        }, {}, None))
    # Steps 10-11: still CRITICAL, but a peer track becomes available ->
    # request_peer_track instead of continuing to hold.
    for t in range(10, 12):
        timeline.append((t, {
            "track_covariance": 40.0, "track_age": 10, "current_trust_value": 0.05,
        }, {"peer_track_available": True}, None))
    # Step 12: peer track lost again, back to holding, and a near-miss
    # happens on this step while abstaining.
    timeline.append((12, {
        "track_covariance": 40.0, "track_age": 10, "current_trust_value": 0.05,
    }, {}, "near_miss"))
    # Steps 13-27: kept holding long enough (>= default max_hold_steps)
    # to escalate to mark_unsafe_to_execute.
    for t in range(13, 28):
        timeline.append((t, {
            "track_covariance": 40.0, "track_age": 10, "current_trust_value": 0.05,
        }, {}, None))
    # Steps 28-30: perception recovers -> episode resolves.
    for t in range(28, 31):
        timeline.append((t, {
            "track_covariance": 0.2, "track_age": 10, "current_trust_value": 0.95,
        }, {}, None))
    return timeline


def _print_log(entries):
    if not entries:
        print("(no events logged)")
        return
    cols = ["step", "event", "reason", "perception_quality_level",
            "selected_fallback", "duration_steps", "final_outcome"]
    widths = {c: max(len(c), *(len(str(e.get(c, ""))) for e in entries)) + 2 for c in cols}
    header = "".join(f"{c:<{widths[c]}}" for c in cols)
    print(header)
    print("-" * len(header))
    for e in entries:
        print("".join(f"{str(e.get(c, '')):<{widths[c]}}" for c in cols))


def _run_cli():
    parser = argparse.ArgumentParser(
        description="Selective decision-making and abstention (Task 14): drives a scripted "
                     "GOOD->DEGRADED->CRITICAL->recovered timeline through SelectiveDecisionMaker "
                     "and prints the resulting lifecycle log.")
    parser.add_argument("--self-check", action="store_true",
                         help="Run the assert-based self-check instead of the demo.")
    parser.add_argument("--csv", default=None, help="Also write the demo log to this CSV path.")
    parser.add_argument("--summary", action="store_true", help="Print the aggregate summary too.")
    args = parser.parse_args()

    if args.self_check:
        _self_check()
        return

    maker = SelectiveDecisionMaker()
    for t, signals, resources, safety_event in _demo_timeline():
        maker.decide("demo_uav", t, signals, available_resources=resources, safety_event=safety_event)
    maker.close_all(t=31)

    _print_log(maker.log)
    if args.summary:
        print()
        print(json.dumps(maker.summary(), indent=2))
    if args.csv:
        maker.to_csv(args.csv)
        print(f"\nwrote {len(maker.log)} rows to {args.csv}")


def _self_check():
    """Smallest thing that fails if the decision/escalation logic breaks -
    not a full test suite. Run directly:
    python selective_swarm_decision.py --self-check"""
    maker = SelectiveDecisionMaker(max_hold_steps=3)

    # GOOD signals -> no abstention, nothing logged.
    d = maker.decide("u0", 0, {"track_covariance": 0.1, "current_trust_value": 0.99})
    assert d["abstain"] is False and d["fallback_action"] is None
    assert maker.log == []
    assert not maker.abstaining("u0")

    # DEGRADED -> abstains, reduce_speed (always available), episode opens.
    d = maker.decide("u0", 1, {"track_covariance": 6.0, "current_trust_value": 0.5})
    assert d["abstain"] is True and d["level"] == DEGRADED
    assert d["fallback_action"] == REDUCE_SPEED, d["fallback_action"]
    assert d["reason"] is not None
    assert maker.abstaining("u0")
    triggered = [e for e in maker.log if e["event"] == EVENT_TRIGGERED]
    assert len(triggered) == 1 and triggered[0]["abstention_triggered"] is True

    # increase_formation_spacing chosen over reduce_speed once available,
    # since it's earlier in DEGRADED_FALLBACK_PRIORITY.
    d = maker.decide("u0", 2, {"track_covariance": 6.0, "current_trust_value": 0.5},
                      available_resources={"formation_can_expand": True})
    assert d["fallback_action"] == INCREASE_FORMATION_SPACING, d["fallback_action"]

    # CRITICAL with no resources -> hold_position, and an escalation event
    # is logged (peak_level changes DEGRADED -> CRITICAL).
    d = maker.decide("u0", 3, {"track_covariance": 40.0, "current_trust_value": 0.02})
    assert d["level"] == CRITICAL and d["fallback_action"] == HOLD_POSITION
    escalated = [e for e in maker.log if e["event"] == EVENT_ESCALATED]
    assert len(escalated) == 1, escalated

    # CRITICAL with a peer track available -> request_peer_track instead
    # of continuing to hold.
    d = maker.decide("u0", 4, {"track_covariance": 40.0, "current_trust_value": 0.02},
                      available_resources={"peer_track_available": True})
    assert d["fallback_action"] == REQUEST_PEER_TRACK, d["fallback_action"]

    # A safety event while abstaining is logged immediately, distinctly.
    maker.decide("u0", 5, {"track_covariance": 40.0, "current_trust_value": 0.02},
                 safety_event="near_miss")
    safety_events = [e for e in maker.log if e["event"] == EVENT_SAFETY_EVENT]
    assert len(safety_events) == 1 and safety_events[0]["safety_event"] == "near_miss"

    # Holding max_hold_steps (3) steps in a row escalates to
    # mark_unsafe_to_execute instead of continuing to hold forever.
    maker.decide("u0", 6, {"track_covariance": 40.0, "current_trust_value": 0.02})  # hold #1
    maker.decide("u0", 7, {"track_covariance": 40.0, "current_trust_value": 0.02})  # hold #2
    d = maker.decide("u0", 8, {"track_covariance": 40.0, "current_trust_value": 0.02})  # hold #3 -> escalate
    assert d["fallback_action"] == MARK_UNSAFE_TO_EXECUTE, d["fallback_action"]

    # Quality recovering closes the episode with OUTCOME_SAFETY_EVENT
    # (not "recovered") since a safety event occurred earlier in it.
    d = maker.decide("u0", 9, {"track_covariance": 0.1, "current_trust_value": 0.99})
    assert d["abstain"] is False
    resolved = [e for e in maker.log if e["event"] == EVENT_RESOLVED]
    assert len(resolved) == 1
    assert resolved[0]["final_outcome"] == OUTCOME_SAFETY_EVENT, resolved[0]["final_outcome"]
    assert resolved[0]["duration_steps"] == 9 - 1  # start_step was 1
    assert not maker.abstaining("u0")

    # No signals at all -> abstains with NO_SIGNAL_REASON, not silently GOOD.
    d = maker.decide("u1", 0, {})
    assert d["abstain"] is True and d["reason"] == NO_SIGNAL_REASON

    # close_all force-closes a still-open episode as unresolved.
    closed = maker.close_all(t=1)
    assert len(closed) == 1 and closed[0]["final_outcome"] == OUTCOME_UNRESOLVED_AT_END
    assert not maker.abstaining("u1")

    # Every logged entry must be JSON-serializable (it's meant to be
    # written to CSV/JSON logs downstream).
    json.dumps(maker.log)

    # summary() reflects what actually happened.
    s = maker.summary()
    assert s["abstention_episodes_triggered"] == 2, s
    assert s["fallback_action_counts"].get(MARK_UNSAFE_TO_EXECUTE) == 1, s

    # No ground-truth parameter anywhere on the public decision-making API.
    import inspect
    for name in ("decide", "decide_for_detection"):
        params = inspect.signature(getattr(maker, name)).parameters
        assert not any("true" in p.lower() or "ground_truth" in p.lower() for p in params), (
            f"{name} must not accept a ground-truth parameter")

    print("selective_swarm_decision: all self-checks passed")


if __name__ == "__main__":
    _run_cli()
