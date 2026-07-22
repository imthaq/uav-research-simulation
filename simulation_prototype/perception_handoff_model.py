"""Task 15: perception handoff strategies.

Sits alongside selective_swarm_decision.py (Task 14) downstream of
perception_quality_monitor.py (Task 12), but answers a narrower question.
Task 14 decides what a UAV's *motion* should do when perception quality is
insufficient (hold, slow down, widen formation, ...). This module decides
which perception *source* a track should be handed off to - a track-
sourcing decision, not a motion decision - and is driven by seven named
triggers rather than a single composite score:

    CRITICAL quality status
    excessive covariance
    repeated missed detections
    strong sensor disagreement
    stale distributed track
    sensor failure
    communication recovery

Choosing from six handoff modes:

    no handoff
    local radar-only fallback
    local LiDAR fallback
    request neighbouring UAV track
    handoff to centralized fusion
    safe HOLD when no reliable source exists

The two modules are meant to be used together (a caller can run both per
step and act on Task 14's motion fallback plus this module's sourcing
handoff), but neither depends on the other's code - only on the same
perception_quality_monitor.py signals/levels, and on the same
available_resources convention (radar_available, lidar_available,
peer_track_available, centralized_fusion_available) selective_swarm_
decision.py already established, so a caller wiring both up passes the
same resource dict to each.

Like both of those modules, this one NEVER uses true position error or any
other ground-truth field - there is no such parameter anywhere in its
public API, by design (see _self_check).
"""
import argparse
import csv
import json

from perception_quality_monitor import (
    PerceptionQualityMonitor, GOOD, DEGRADED, CRITICAL,
    COVARIANCE_TRACE_REFERENCE, MISSED_UPDATE_CEILING,
    COMMUNICATION_AGE_HALF_LIFE_STEPS,
)

# ----------------------------------------------------------------------
# Handoff modes - one constant per bullet in the Task 15 spec.
# ----------------------------------------------------------------------
NO_HANDOFF = "no_handoff"
RADAR_ONLY_FALLBACK = "local_radar_only_fallback"
LIDAR_ONLY_FALLBACK = "local_lidar_only_fallback"
REQUEST_PEER_TRACK = "request_neighbouring_uav_track"
CENTRALIZED_FUSION_HANDOFF = "handoff_to_centralized_fusion"
SAFE_HOLD = "safe_hold_no_reliable_source"

HANDOFF_MODES = (
    NO_HANDOFF, RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK, REQUEST_PEER_TRACK,
    CENTRALIZED_FUSION_HANDOFF, SAFE_HOLD,
)

# ----------------------------------------------------------------------
# Triggers - one constant per bullet in the Task 15 spec.
# ----------------------------------------------------------------------
TRIGGER_SENSOR_FAILURE = "sensor_failure"
TRIGGER_CRITICAL_QUALITY = "critical_quality_status"
TRIGGER_SENSOR_DISAGREEMENT = "strong_sensor_disagreement"
TRIGGER_EXCESSIVE_COVARIANCE = "excessive_covariance"
TRIGGER_REPEATED_MISSED_DETECTIONS = "repeated_missed_detections"
TRIGGER_STALE_DISTRIBUTED_TRACK = "stale_distributed_track"
TRIGGER_COMMUNICATION_RECOVERY = "communication_recovery"

# Precedence when more than one "something is wrong" trigger fires on the
# same step: most severe / most specific first. TRIGGER_COMMUNICATION_
# RECOVERY is deliberately excluded from this tuple - it is not a "something
# is wrong" trigger, it is a one-shot recovery event handled separately in
# _select_mode (see there for why).
TRIGGER_PRECEDENCE = (
    TRIGGER_SENSOR_FAILURE,
    TRIGGER_CRITICAL_QUALITY,
    TRIGGER_SENSOR_DISAGREEMENT,
    TRIGGER_EXCESSIVE_COVARIANCE,
    TRIGGER_REPEATED_MISSED_DETECTIONS,
    TRIGGER_STALE_DISTRIBUTED_TRACK,
)

# ----------------------------------------------------------------------
# Trigger thresholds. Reused from perception_quality_monitor.py wherever
# that module already established a scale for the same quantity, so a
# caller computing one signal already has what this module needs - not
# reinvented constants that could silently drift apart from that module's.
# ----------------------------------------------------------------------

# A covariance trace this many times perception_quality_monitor's own
# "borderline trustworthy" reference is unambiguously bad on its own,
# regardless of what the other signals say - not just a contributor to a
# lower composite score, but its own dedicated trigger.
EXCESSIVE_COVARIANCE_MULTIPLIER = 3.0
EXCESSIVE_COVARIANCE_TRACE = COVARIANCE_TRACE_REFERENCE * EXCESSIVE_COVARIANCE_MULTIPLIER

# "Repeated" missed detections is exactly the point at which
# perception_quality_monitor.MISSED_UPDATE_CEILING already means the track
# would be deleted anyway (it matches tracking/radar_track_model.MAX_MISSED)
# - reusing that ceiling rather than defining a second "how many is too
# many" number that could disagree with it.
REPEATED_MISSED_DETECTIONS_CEILING = MISSED_UPDATE_CEILING

# Cross-sensor position disagreement (meters) large enough to be a real
# conflict between sources rather than ordinary per-sensor measurement
# noise. No existing module computes a raw cross-sensor disagreement
# distance directly, so this is its own tunable default - kept generous
# relative to the position noise stds of every *_like_model.py sensor
# (all well under 1m) so it only fires on genuine disagreement.
SENSOR_DISAGREEMENT_DISTANCE_M = 2.0

# A relayed/distributed track's communication age at which it has decayed
# to a third of perception_quality_monitor's own scored weight (twice that
# module's COMMUNICATION_AGE_HALF_LIFE_STEPS) - unambiguously stale, not
# merely aging.
STALE_DISTRIBUTED_TRACK_AGE_STEPS = COMMUNICATION_AGE_HALF_LIFE_STEPS * 2.0

# A rolling dropout rate at/above this is treated as an outright sensor
# failure rather than degraded-but-still-reporting (perception_quality_
# monitor.DROPOUT_RATE_CEILING of 0.5 already scores such a sensor 0, but
# "failed" is a stronger, more specific claim than "scores 0").
SENSOR_FAILURE_DROPOUT_RATE = 0.95

TRIGGER_LABELS = {
    TRIGGER_SENSOR_FAILURE: "sensor_failure",
    TRIGGER_CRITICAL_QUALITY: "perception_quality_critical",
    TRIGGER_SENSOR_DISAGREEMENT: "strong_sensor_disagreement",
    TRIGGER_EXCESSIVE_COVARIANCE: "excessive_track_covariance",
    TRIGGER_REPEATED_MISSED_DETECTIONS: "repeated_missed_detections",
    TRIGGER_STALE_DISTRIBUTED_TRACK: "stale_distributed_track",
    TRIGGER_COMMUNICATION_RECOVERY: "communication_recovery",
}

# ----------------------------------------------------------------------
# Mode priority per primary trigger. Severity picks which tuple applies;
# available_resources (radar_available, lidar_available,
# peer_track_available, centralized_fusion_available - same keys
# selective_swarm_decision.py already uses) picks the first entry in it
# that's actually usable right now. SAFE_HOLD is deliberately last in
# every tuple and always "available" - the guaranteed fallback when
# nothing else is.
# ----------------------------------------------------------------------

# Local perception itself is the problem (its own quality collapsed, its
# covariance blew up, or it's missing too many updates): try the other
# local sensor first - cheapest, lowest-latency recovery - before reaching
# off-board.
_LOCAL_QUALITY_PRIORITY = (
    RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK, REQUEST_PEER_TRACK,
    CENTRALIZED_FUSION_HANDOFF, SAFE_HOLD,
)

# A specific local sensor has failed outright: same shape as above. The
# caller is expected to reflect the failed sensor's own unavailability in
# available_resources (e.g. radar_available=False if radar is what
# failed), so this tuple doesn't need to special-case which one it was -
# _select_mode just skips whichever one resources says isn't there.
_SENSOR_FAILURE_PRIORITY = _LOCAL_QUALITY_PRIORITY

# The local sensors disagree strongly with each other: neither one is
# individually trustworthy right now (that's the whole problem), so an
# external arbiter - a peer's independent track, or centralized fusion -
# is tried before falling back to trusting one of the two disagreeing
# local sensors anyway.
_SENSOR_DISAGREEMENT_PRIORITY = (
    REQUEST_PEER_TRACK, CENTRALIZED_FUSION_HANDOFF,
    RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK, SAFE_HOLD,
)

# The problem is specifically that a *relayed* (distributed/peer) track
# has gone stale: the peer source is exactly what's unreliable here, so
# REQUEST_PEER_TRACK is excluded from its own trigger's priority - prefer
# falling back to this UAV's own local sensors, then centralized fusion,
# before considering another peer.
_STALE_DISTRIBUTED_TRACK_PRIORITY = (
    RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK, CENTRALIZED_FUSION_HANDOFF, SAFE_HOLD,
)

_MODE_PRIORITY = {
    TRIGGER_SENSOR_FAILURE: _SENSOR_FAILURE_PRIORITY,
    TRIGGER_CRITICAL_QUALITY: _LOCAL_QUALITY_PRIORITY,
    TRIGGER_SENSOR_DISAGREEMENT: _SENSOR_DISAGREEMENT_PRIORITY,
    TRIGGER_EXCESSIVE_COVARIANCE: _LOCAL_QUALITY_PRIORITY,
    TRIGGER_REPEATED_MISSED_DETECTIONS: _LOCAL_QUALITY_PRIORITY,
    TRIGGER_STALE_DISTRIBUTED_TRACK: _STALE_DISTRIBUTED_TRACK_PRIORITY,
}

# ----------------------------------------------------------------------
# Final outcomes / log event types - same small-fixed-set convention
# selective_swarm_decision.py uses for its own episode log.
# ----------------------------------------------------------------------
OUTCOME_RECOVERED = "handoff_no_longer_needed"
OUTCOME_UNRESOLVED_AT_END = "unresolved_at_simulation_end"

EVENT_TRIGGERED = "handoff_triggered"
EVENT_MODE_SELECTED = "handoff_mode_selected"
EVENT_COMMUNICATION_RECOVERY = "communication_recovery_reassessed"
EVENT_RESOLVED = "handoff_resolved"

LOG_FIELDS = [
    "uav_id", "step", "event",
    "primary_trigger", "active_triggers",
    "handoff_mode", "duration_steps", "final_outcome",
]


def _covariance_trace(covariance):
    """Same acceptance rule as perception_quality_monitor._covariance_trace:
    either a precomputed trace (a number) or a covariance matrix."""
    if covariance is None:
        return None
    if isinstance(covariance, (int, float)):
        return float(covariance)
    try:
        return float(sum(row[i] for i, row in enumerate(covariance)))
    except (TypeError, IndexError):
        return None


def evaluate_triggers(signals):
    """Evaluates all seven Task 15 triggers against one track/sensor's
    current signals and returns (active_triggers, quality_level), where
    active_triggers is a list of (trigger_name, detail_dict) in
    TRIGGER_PRECEDENCE order (communication recovery, if present, is
    appended last since it isn't part of that precedence).

    signals: dict, any subset of:
        track_covariance           - trace or matrix (perception_quality_
                                      monitor convention)
        missed_update_count
        sensor_disagreement_distance - max pairwise position disagreement
                                      (meters) between concurrent sensor
                                      estimates for the same target
        communication_age_steps    - age of a relayed/distributed track
        sensor_dropout_rate
        sensor_failed               - explicit bool, in addition to (not
                                      instead of) the dropout-rate check
        communication_recovered     - explicit one-shot bool event
        perception_quality_level    - precomputed GOOD/DEGRADED/CRITICAL,
                                      e.g. from PerceptionQualityMonitor.
                                      evaluate() or selective_swarm_
                                      decision.SelectiveDecisionMaker,
                                      upstream of this module.

    perception_quality_level is taken as given rather than re-derived from
    these same raw signals here: PerceptionQualityMonitor.evaluate()
    treats "no recognized signal present at all" as CRITICAL by design
    (an unmonitorable track must not read as healthy), which is the right
    call for that module but would be the wrong call to make silently
    inside this one - it would make CRITICAL_QUALITY fire as a side effect
    of, say, a disagreement-only signals dict that was never meant to
    describe quality at all. If a caller wants this module to derive the
    level itself rather than supplying a precomputed one, call
    quality_monitor.evaluate(signals) explicitly and pass the result in as
    signals["perception_quality_level"] (see PerceptionHandoffModel.
    quality_monitor for a ready instance).

    quality_level is returned alongside active_triggers so a caller/logger
    can see what quality status this decision was made against even when
    no CRITICAL trigger fired (None if not supplied)."""
    level = signals.get("perception_quality_level")

    active = []

    if bool(signals.get("sensor_failed", False)) or (
            signals.get("sensor_dropout_rate") is not None
            and signals["sensor_dropout_rate"] >= SENSOR_FAILURE_DROPOUT_RATE):
        active.append((TRIGGER_SENSOR_FAILURE, {
            "sensor_dropout_rate": signals.get("sensor_dropout_rate"),
        }))

    if level == CRITICAL:
        active.append((TRIGGER_CRITICAL_QUALITY, {"perception_quality_level": level}))

    disagreement = signals.get("sensor_disagreement_distance")
    if disagreement is not None and disagreement >= SENSOR_DISAGREEMENT_DISTANCE_M:
        active.append((TRIGGER_SENSOR_DISAGREEMENT, {
            "sensor_disagreement_distance": disagreement,
            "threshold": SENSOR_DISAGREEMENT_DISTANCE_M,
        }))

    trace = _covariance_trace(signals.get("track_covariance"))
    if trace is not None and trace >= EXCESSIVE_COVARIANCE_TRACE:
        active.append((TRIGGER_EXCESSIVE_COVARIANCE, {
            "track_covariance_trace": trace,
            "threshold": EXCESSIVE_COVARIANCE_TRACE,
        }))

    missed = signals.get("missed_update_count")
    if missed is not None and missed >= REPEATED_MISSED_DETECTIONS_CEILING:
        active.append((TRIGGER_REPEATED_MISSED_DETECTIONS, {
            "missed_update_count": missed,
            "threshold": REPEATED_MISSED_DETECTIONS_CEILING,
        }))

    comm_age = signals.get("communication_age_steps")
    if comm_age is not None and comm_age >= STALE_DISTRIBUTED_TRACK_AGE_STEPS:
        active.append((TRIGGER_STALE_DISTRIBUTED_TRACK, {
            "communication_age_steps": comm_age,
            "threshold": STALE_DISTRIBUTED_TRACK_AGE_STEPS,
        }))

    if bool(signals.get("communication_recovered", False)):
        active.append((TRIGGER_COMMUNICATION_RECOVERY, {}))

    return active, level


class PerceptionHandoffModel:
    """Turns the seven Task 15 triggers into a handoff-mode decision, per
    UAV/track, with full lifecycle logging of the resulting handoff
    episode - mirroring selective_swarm_decision.SelectiveDecisionMaker's
    episode-log shape so the two can sit side by side in a combined log.

    One instance is meant to be shared across a whole run (it tracks one
    open episode at most per uav_id), not recreated every step.
    """

    def __init__(self, quality_monitor=None, weights=None):
        self.quality_monitor = quality_monitor or PerceptionQualityMonitor(weights=weights)
        self._episodes = {}   # uav_id -> open episode dict, absent if none
        self.log = []         # flat, append-only list of logged lifecycle events

    # -- internal helpers -------------------------------------------------

    def _select_mode(self, active_triggers, resources):
        """active_triggers: the list evaluate_triggers() returned.
        Returns (mode, primary_trigger). primary_trigger is None when mode
        is NO_HANDOFF (nothing wrong) - communication recovery, when it's
        the *only* active trigger, resolves to exactly that: not a problem
        needing a mode, just confirmation that whatever previously
        justified a handoff no longer applies."""
        resources = resources or {}
        problem_triggers = [name for name, _ in active_triggers if name in _MODE_PRIORITY]

        if not problem_triggers:
            return NO_HANDOFF, None

        # Precedence, not encounter order: TRIGGER_PRECEDENCE always wins
        # regardless of the order active_triggers happens to list them in.
        primary = next(t for t in TRIGGER_PRECEDENCE if t in problem_triggers)

        def available(mode):
            if mode == RADAR_ONLY_FALLBACK:
                return bool(resources.get("radar_available", False))
            if mode == LIDAR_ONLY_FALLBACK:
                return bool(resources.get("lidar_available", False))
            if mode == REQUEST_PEER_TRACK:
                return bool(resources.get("peer_track_available", False))
            if mode == CENTRALIZED_FUSION_HANDOFF:
                return bool(resources.get("centralized_fusion_available", False))
            if mode == SAFE_HOLD:
                return True
            return False

        for mode in _MODE_PRIORITY[primary]:
            if available(mode):
                return mode, primary
        return SAFE_HOLD, primary  # guaranteed last resort

    def _log_event(self, uav_id, t, event, *, primary_trigger=None, active_triggers=None,
                    mode=None, duration_steps=None, final_outcome=None):
        entry = {
            "uav_id": uav_id,
            "step": t,
            "event": event,
            "primary_trigger": primary_trigger,
            "active_triggers": [name for name, _ in (active_triggers or [])],
            "handoff_mode": mode,
            "duration_steps": duration_steps,
            "final_outcome": final_outcome,
        }
        self.log.append(entry)
        return entry

    def _close_episode(self, uav_id, t, *, forced=False):
        episode = self._episodes.pop(uav_id, None)
        if episode is None:
            return None
        duration = t - episode["start_step"]
        outcome = OUTCOME_UNRESOLVED_AT_END if forced else OUTCOME_RECOVERED
        return self._log_event(
            uav_id, t, EVENT_RESOLVED, primary_trigger=episode["primary_trigger"],
            mode=episode["current_mode"], duration_steps=duration, final_outcome=outcome)

    # -- public API ---------------------------------------------------

    def decide(self, uav_id, t, signals, available_resources=None):
        """Evaluates one UAV/track's current signals (same shape
        evaluate_triggers accepts) and returns a decision dict for this
        step:

            {"uav_id", "step", "handoff_mode", "quality_level",
             "primary_trigger", "active_triggers", "duration_steps"}

        handoff_mode == NO_HANDOFF means no trigger fired: use the track's
        normal local source as-is. Any other mode means the caller should
        source this step's track from that mode instead.

        available_resources: same convention selective_swarm_decision.py
        uses - radar_available, lidar_available, peer_track_available,
        centralized_fusion_available. Anything omitted is treated as
        unavailable, so with no resource info at all the only mode ever
        selected (once a trigger fires) is SAFE_HOLD - a safe default
        rather than assuming resources exist.
        """
        active_triggers, level = evaluate_triggers(signals)
        episode = self._episodes.get(uav_id)

        mode, primary_trigger = self._select_mode(active_triggers, available_resources)

        if mode == NO_HANDOFF:
            if episode is not None:
                self._close_episode(uav_id, t, forced=False)
            return {
                "uav_id": uav_id, "step": t, "handoff_mode": NO_HANDOFF,
                "quality_level": level, "primary_trigger": None,
                "active_triggers": [name for name, _ in active_triggers], "duration_steps": 0,
            }

        if episode is None:
            episode = {"start_step": t, "primary_trigger": primary_trigger, "current_mode": None}
            self._episodes[uav_id] = episode
            self._log_event(uav_id, t, EVENT_TRIGGERED, primary_trigger=primary_trigger,
                             active_triggers=active_triggers, duration_steps=0)

        # A communication-recovery event alongside an ongoing problem
        # doesn't end the episode (the problem trigger is still active),
        # but it's worth logging distinctly: it's the moment distributed
        # resources (peer/centralized) become worth re-checking, which is
        # exactly what available_resources reflects on the caller's side.
        if any(name == TRIGGER_COMMUNICATION_RECOVERY for name, _ in active_triggers):
            self._log_event(uav_id, t, EVENT_COMMUNICATION_RECOVERY,
                             primary_trigger=primary_trigger, active_triggers=active_triggers,
                             mode=mode, duration_steps=t - episode["start_step"])

        if mode != episode["current_mode"]:
            self._log_event(uav_id, t, EVENT_MODE_SELECTED, primary_trigger=primary_trigger,
                             active_triggers=active_triggers, mode=mode,
                             duration_steps=t - episode["start_step"])
        episode["current_mode"] = mode
        episode["primary_trigger"] = primary_trigger

        return {
            "uav_id": uav_id, "step": t, "handoff_mode": mode, "quality_level": level,
            "primary_trigger": primary_trigger,
            "active_triggers": [name for name, _ in active_triggers],
            "duration_steps": t - episode["start_step"] + 1,
        }

    def handing_off(self, uav_id):
        """True if uav_id currently has an open handoff episode (mode !=
        NO_HANDOFF as of its last decide() call)."""
        return uav_id in self._episodes

    def close_all(self, t):
        """Force-closes every still-open episode as of step t (e.g. at the
        end of a simulation run), logging OUTCOME_UNRESOLVED_AT_END for
        each. Returns the list of resulting log entries."""
        closed = []
        for uav_id in list(self._episodes):
            entry = self._close_episode(uav_id, t, forced=True)
            if entry is not None:
                closed.append(entry)
        return closed

    def summary(self):
        """Aggregate stats over every logged event so far: counts per
        handoff mode actually selected, per primary trigger, per final
        outcome, and the average duration_steps of resolved episodes -
        same shape convention as SelectiveDecisionMaker.summary()."""
        mode_counts = {}
        trigger_counts = {}
        outcome_counts = {}
        resolved_durations = []
        triggered = 0
        for e in self.log:
            if e["event"] == EVENT_TRIGGERED:
                triggered += 1
                if e["primary_trigger"]:
                    trigger_counts[e["primary_trigger"]] = trigger_counts.get(e["primary_trigger"], 0) + 1
            if e["event"] == EVENT_MODE_SELECTED and e["handoff_mode"]:
                mode_counts[e["handoff_mode"]] = mode_counts.get(e["handoff_mode"], 0) + 1
            if e["event"] == EVENT_RESOLVED:
                if e["final_outcome"]:
                    outcome_counts[e["final_outcome"]] = outcome_counts.get(e["final_outcome"], 0) + 1
                if e["duration_steps"] is not None:
                    resolved_durations.append(e["duration_steps"])
        return {
            "handoff_episodes_triggered": triggered,
            "handoff_mode_counts": mode_counts,
            "primary_trigger_counts": trigger_counts,
            "final_outcome_counts": outcome_counts,
            "avg_resolved_duration_steps": (
                round(sum(resolved_durations) / len(resolved_durations), 3)
                if resolved_durations else None),
            "still_open_episode_count": len(self._episodes),
        }

    def to_csv(self, path):
        """Writes self.log to a CSV file with a stable column order
        (LOG_FIELDS), matching the rest of this project's CSV-log style.
        active_triggers is JSON-encoded into its cell since it's a list,
        not a scalar."""
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
            writer.writeheader()
            for entry in self.log:
                row = dict(entry)
                row["active_triggers"] = json.dumps(row["active_triggers"])
                writer.writerow(row)

    # -- convenience wrapper tied to a radar_track_model-shaped row --------

    def decide_for_track_row(self, uav_id, t, track_row, sensor_disagreement_distance=None,
                              communication_age_steps=None, sensor_dropout_rate=None,
                              sensor_failed=False, communication_recovered=False,
                              available_resources=None):
        """Convenience wrapper mirroring PerceptionQualityMonitor.
        evaluate_track_row: pulls covariance/missed_count straight out of a
        tracking/radar_track_model-shaped row (covariance as the JSON
        string RadarTrack.as_row actually produces); everything else lives
        outside a single track row, so those are taken as explicit keyword
        arguments the same way evaluate_track_row does."""
        cov = track_row.get("covariance")
        if isinstance(cov, str):
            try:
                cov = json.loads(cov)
            except (ValueError, TypeError):
                cov = None
        signals = {
            "track_covariance": cov,
            "missed_update_count": track_row.get("missed_count"),
            "sensor_disagreement_distance": sensor_disagreement_distance,
            "communication_age_steps": communication_age_steps,
            "sensor_dropout_rate": sensor_dropout_rate,
            "sensor_failed": sensor_failed,
            "communication_recovered": communication_recovered,
        }
        return self.decide(uav_id, t, signals, available_resources=available_resources)


# ----------------------------------------------------------------------
# CLI demo
# ----------------------------------------------------------------------
# A scripted timeline for one synthetic UAV/track walking through every
# trigger at least once, so running this file directly demonstrates every
# event type end to end without needing a live simulation wired up. Not
# test cases (those live in _self_check).
def _demo_timeline():
    timeline = []
    # Steps 0-2: healthy, no handoff.
    for t in range(0, 3):
        timeline.append((t, {"track_covariance": 0.2, "missed_update_count": 0}, {}))
    # Steps 3-5: excessive covariance, radar available -> radar-only fallback.
    for t in range(3, 6):
        timeline.append((t, {"track_covariance": 20.0, "missed_update_count": 0},
                          {"radar_available": True, "lidar_available": True}))
    # Steps 6-7: radar itself now fails -> falls back to LiDAR instead.
    for t in range(6, 8):
        timeline.append((t, {"track_covariance": 20.0, "missed_update_count": 0,
                              "sensor_failed": True},
                          {"radar_available": False, "lidar_available": True}))
    # Steps 8-9: strong sensor disagreement, no peer/centralized available yet
    # -> falls through to whichever local sensor is left (LiDAR).
    for t in range(8, 10):
        timeline.append((t, {"sensor_disagreement_distance": 5.0},
                          {"radar_available": False, "lidar_available": True}))
    # Step 10: same disagreement, but a peer track becomes available -> that's
    # preferred over trusting either disagreeing local sensor.
    timeline.append((10, {"sensor_disagreement_distance": 5.0},
                      {"radar_available": False, "lidar_available": True,
                       "peer_track_available": True}))
    # Steps 11-12: that peer track itself goes stale -> back to local (LiDAR),
    # not another peer.
    for t in range(11, 13):
        timeline.append((t, {"communication_age_steps": 20.0},
                          {"lidar_available": True, "peer_track_available": True}))
    # Step 13: nothing available at all -> safe hold.
    timeline.append((13, {"communication_age_steps": 20.0}, {}))
    # Steps 14-15: communication recovers, centralized fusion now reachable
    # -> hands off there instead of continuing to hold.
    for t in range(14, 16):
        timeline.append((t, {"communication_age_steps": 20.0, "communication_recovered": True},
                          {"centralized_fusion_available": True}))
    # Steps 16-18: everything recovers -> episode resolves.
    for t in range(16, 19):
        timeline.append((t, {"track_covariance": 0.2, "missed_update_count": 0}, {}))
    return timeline


def _print_log(entries):
    if not entries:
        print("(no events logged)")
        return
    cols = ["step", "event", "primary_trigger", "handoff_mode", "duration_steps", "final_outcome"]
    widths = {c: max(len(c), *(len(str(e.get(c, ""))) for e in entries)) + 2 for c in cols}
    header = "".join(f"{c:<{widths[c]}}" for c in cols)
    print(header)
    print("-" * len(header))
    for e in entries:
        print("".join(f"{str(e.get(c, '')):<{widths[c]}}" for c in cols))


def _run_cli():
    parser = argparse.ArgumentParser(
        description="Perception handoff strategies (Task 15): drives a scripted trigger "
                    "timeline through PerceptionHandoffModel and prints the resulting "
                    "lifecycle log.")
    parser.add_argument("--self-check", action="store_true",
                        help="Run the assert-based self-check instead of the demo.")
    parser.add_argument("--csv", default=None, help="Also write the demo log to this CSV path.")
    parser.add_argument("--summary", action="store_true", help="Print the aggregate summary too.")
    args = parser.parse_args()

    if args.self_check:
        _self_check()
        return

    model = PerceptionHandoffModel()
    for t, signals, resources in _demo_timeline():
        model.decide("demo_uav", t, signals, available_resources=resources)
    model.close_all(t=19)

    _print_log(model.log)
    if args.summary:
        print()
        print(json.dumps(model.summary(), indent=2))
    if args.csv:
        model.to_csv(args.csv)
        print(f"\nwrote {len(model.log)} rows to {args.csv}")


def _self_check():
    """Smallest thing that fails if the trigger/priority logic breaks - not
    a full test suite. Run directly:
    python perception_handoff_model.py --self-check"""
    model = PerceptionHandoffModel()

    # Healthy signals -> no handoff, nothing logged.
    d = model.decide("u0", 0, {"track_covariance": 0.2, "missed_update_count": 0})
    assert d["handoff_mode"] == NO_HANDOFF and d["primary_trigger"] is None
    assert model.log == []
    assert not model.handing_off("u0")

    # Excessive covariance in isolation (quality_level pinned to DEGRADED so
    # only the dedicated covariance trigger fires, not CRITICAL_QUALITY too)
    # with both local sensors available -> prefers radar (first in
    # _LOCAL_QUALITY_PRIORITY) over LiDAR.
    d = model.decide("u0", 1, {"track_covariance": 50.0, "perception_quality_level": DEGRADED},
                      available_resources={"radar_available": True, "lidar_available": True})
    assert d["handoff_mode"] == RADAR_ONLY_FALLBACK, d["handoff_mode"]
    assert d["primary_trigger"] == TRIGGER_EXCESSIVE_COVARIANCE
    assert model.handing_off("u0")
    triggered = [e for e in model.log if e["event"] == EVENT_TRIGGERED]
    assert len(triggered) == 1

    # Same problem, but radar specifically has failed -> falls to LiDAR
    # instead, and sensor-failure outranks excessive-covariance as the
    # primary trigger per TRIGGER_PRECEDENCE.
    d = model.decide("u0", 2, {"track_covariance": 50.0, "perception_quality_level": DEGRADED,
                                "sensor_failed": True},
                      available_resources={"radar_available": False, "lidar_available": True})
    assert d["handoff_mode"] == LIDAR_ONLY_FALLBACK, d["handoff_mode"]
    assert d["primary_trigger"] == TRIGGER_SENSOR_FAILURE, d["primary_trigger"]

    # No local sensors and no distributed resources at all -> safe hold,
    # the guaranteed last resort.
    d = model.decide("u0", 3, {"track_covariance": 50.0, "perception_quality_level": DEGRADED})
    assert d["handoff_mode"] == SAFE_HOLD, d["handoff_mode"]

    # Quality recovering closes the episode as OUTCOME_RECOVERED.
    d = model.decide("u0", 4, {"track_covariance": 0.2, "missed_update_count": 0})
    assert d["handoff_mode"] == NO_HANDOFF
    resolved = [e for e in model.log if e["event"] == EVENT_RESOLVED]
    assert len(resolved) == 1 and resolved[0]["final_outcome"] == OUTCOME_RECOVERED
    assert resolved[0]["duration_steps"] == 4 - 1  # start_step was 1
    assert not model.handing_off("u0")

    # Strong sensor disagreement prefers an external arbiter (peer track)
    # over either disagreeing local sensor, even when both locals are
    # nominally available.
    d = model.decide("u1", 0, {"sensor_disagreement_distance": 6.0},
                      available_resources={"radar_available": True, "lidar_available": True,
                                            "peer_track_available": True})
    assert d["handoff_mode"] == REQUEST_PEER_TRACK, d["handoff_mode"]

    # ...but falls through to local sensors if no peer/centralized is
    # actually reachable, rather than blindly picking one of the two
    # disagreeing sensors as a first choice.
    d = model.decide("u1", 1, {"sensor_disagreement_distance": 6.0},
                      available_resources={"radar_available": True, "lidar_available": True})
    assert d["handoff_mode"] == RADAR_ONLY_FALLBACK, d["handoff_mode"]

    # A stale distributed track excludes REQUEST_PEER_TRACK from its own
    # priority (the peer source is exactly what's stale) even when a peer
    # is nominally marked available.
    d = model.decide("u2", 0, {"communication_age_steps": 30.0},
                      available_resources={"peer_track_available": True, "lidar_available": True})
    assert d["handoff_mode"] == LIDAR_ONLY_FALLBACK, d["handoff_mode"]
    d = model.decide("u2", 1, {"communication_age_steps": 30.0},
                      available_resources={"peer_track_available": True})
    assert d["handoff_mode"] == SAFE_HOLD, d["handoff_mode"]

    # A CRITICAL perception_quality_level (precomputed upstream, e.g. by
    # PerceptionQualityMonitor or a full-signals evaluate() call) is its
    # own trigger even with no other signal present.
    d = model.decide("u3", 0, {"perception_quality_level": CRITICAL})
    assert d["primary_trigger"] == TRIGGER_CRITICAL_QUALITY, d["primary_trigger"]
    assert d["handoff_mode"] == SAFE_HOLD

    # communication_recovered alongside an ongoing problem doesn't end the
    # episode by itself (the problem trigger is still active) but is
    # logged as its own event, and can unlock a mode that wasn't available
    # before (centralized fusion) without needing the episode to close.
    model.decide("u4", 0, {"communication_age_steps": 30.0})  # -> SAFE_HOLD, episode opens
    d = model.decide("u4", 1, {"communication_age_steps": 30.0, "communication_recovered": True},
                      available_resources={"centralized_fusion_available": True})
    assert d["handoff_mode"] == CENTRALIZED_FUSION_HANDOFF, d["handoff_mode"]
    assert model.handing_off("u4")
    comm_events = [e for e in model.log if e["event"] == EVENT_COMMUNICATION_RECOVERY]
    assert len(comm_events) == 1, comm_events

    # close_all force-closes a still-open episode as unresolved.
    closed = model.close_all(t=2)
    assert any(e["uav_id"] == "u4" and e["final_outcome"] == OUTCOME_UNRESOLVED_AT_END
               for e in closed)
    assert not model.handing_off("u4")

    # Every logged entry must be JSON-serializable (it's meant to be
    # written to CSV/JSON logs downstream).
    json.dumps(model.log)

    # summary() reflects what actually happened.
    s = model.summary()
    assert s["handoff_episodes_triggered"] >= 5, s
    assert s["handoff_mode_counts"].get(SAFE_HOLD, 0) >= 1, s

    # No ground-truth parameter anywhere on the public decision-making API.
    import inspect
    for name in ("decide", "decide_for_track_row"):
        params = inspect.signature(getattr(model, name)).parameters
        assert not any("true" in p.lower() or "ground_truth" in p.lower() for p in params), (
            f"{name} must not accept a ground-truth parameter")

    print("perception_handoff_model: all self-checks passed")


if __name__ == "__main__":
    _run_cli()
