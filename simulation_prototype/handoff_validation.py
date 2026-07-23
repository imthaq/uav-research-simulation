"""
handoff_validation.py

Task 27: validates perception_handoff_model.py against deterministic,
hand-computable cases.  Every expected output follows directly from the
trigger thresholds and mode-priority tables defined in that module.

  T01 – evaluate_triggers: no signal -> no triggers fire
  T02 – evaluate_triggers: explicit sensor_failed=True -> SENSOR_FAILURE fires
  T03 – evaluate_triggers: dropout_rate >= ceiling -> SENSOR_FAILURE fires
  T04 – evaluate_triggers: perception_quality_level=CRITICAL -> CRITICAL_QUALITY
  T05 – evaluate_triggers: sensor_disagreement >= threshold -> SENSOR_DISAGREEMENT
  T06 – evaluate_triggers: covariance trace >= EXCESSIVE threshold -> EXCESSIVE_COV
  T07 – evaluate_triggers: missed_updates >= ceiling -> REPEATED_MISSED_DETECTIONS
  T08 – evaluate_triggers: comm_age >= STALE threshold -> STALE_DISTRIBUTED_TRACK
  T09 – evaluate_triggers: communication_recovered=True -> COMMUNICATION_RECOVERY
  T10 – evaluate_triggers: multiple triggers fire; TRIGGER_PRECEDENCE order kept
  T11 – mode selection: SAFE_HOLD when no resources available
  T12 – mode selection: radar-only fallback when radar is the only resource
  T13 – mode selection: sensor-disagreement trigger prefers peer/centralized over local
  T14 – mode selection: stale-distributed-track trigger excludes REQUEST_PEER_TRACK
  T15 – PerceptionHandoffModel.decide: healthy track -> NO_HANDOFF
  T16 – PerceptionHandoffModel.decide: failed sensor -> triggers SAFE_HOLD with no resources
  T17 – PerceptionHandoffModel.decide: failed sensor + radar available -> RADAR_ONLY_FALLBACK
  T18 – PerceptionHandoffModel.decide: episode lifecycle — triggered, then resolved
  T19 – PerceptionHandoffModel.decide: duration_steps increments each step
  T20 – PerceptionHandoffModel.handing_off / close_all lifecycle
  T21 – PerceptionHandoffModel.summary: correct episode/mode/trigger counts
  T22 – no ground-truth parameter on any public method (API safety check)

Run directly:
    python handoff_validation.py
"""

import inspect
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from perception_handoff_model import (
    PerceptionHandoffModel, evaluate_triggers,
    NO_HANDOFF, RADAR_ONLY_FALLBACK, LIDAR_ONLY_FALLBACK,
    REQUEST_PEER_TRACK, CENTRALIZED_FUSION_HANDOFF, SAFE_HOLD,
    TRIGGER_SENSOR_FAILURE, TRIGGER_CRITICAL_QUALITY,
    TRIGGER_SENSOR_DISAGREEMENT, TRIGGER_EXCESSIVE_COVARIANCE,
    TRIGGER_REPEATED_MISSED_DETECTIONS, TRIGGER_STALE_DISTRIBUTED_TRACK,
    TRIGGER_COMMUNICATION_RECOVERY,
    SENSOR_FAILURE_DROPOUT_RATE, SENSOR_DISAGREEMENT_DISTANCE_M,
    EXCESSIVE_COVARIANCE_TRACE, REPEATED_MISSED_DETECTIONS_CEILING,
    STALE_DISTRIBUTED_TRACK_AGE_STEPS,
    OUTCOME_RECOVERED, OUTCOME_UNRESOLVED_AT_END,
    EVENT_TRIGGERED, EVENT_RESOLVED,
)
from perception_quality_monitor import CRITICAL, GOOD
from validation_common import Checker

_c = Checker()


def check(task, desc, cond, detail=""):
    return _c.check(task, desc, cond, detail)


def _trigger_names(active):
    """extract trigger name strings from evaluate_triggers output"""
    return [name for name, _ in active]


# ---------------------------------------------------------------------------
# T01: empty signals -> no triggers
# ---------------------------------------------------------------------------
def test_no_triggers_for_healthy_track():
    active, level = evaluate_triggers({})
    check("T01", "empty signals -> no triggers",
          len(active) == 0, f"got {active!r}")
    check("T01", "level is None when perception_quality_level not supplied",
          level is None)


# ---------------------------------------------------------------------------
# T02: explicit sensor_failed=True -> SENSOR_FAILURE fires
# ---------------------------------------------------------------------------
def test_sensor_failed_explicit():
    active, _ = evaluate_triggers({"sensor_failed": True})
    check("T02", "sensor_failed=True fires SENSOR_FAILURE",
          TRIGGER_SENSOR_FAILURE in _trigger_names(active))


# ---------------------------------------------------------------------------
# T03: dropout_rate >= SENSOR_FAILURE_DROPOUT_RATE -> SENSOR_FAILURE fires
# ---------------------------------------------------------------------------
def test_sensor_failed_via_dropout_rate():
    # exactly at the boundary
    active, _ = evaluate_triggers({"sensor_dropout_rate": SENSOR_FAILURE_DROPOUT_RATE})
    check("T03", "dropout_rate=CEILING fires SENSOR_FAILURE",
          TRIGGER_SENSOR_FAILURE in _trigger_names(active))
    # below boundary -> must not fire
    active_below, _ = evaluate_triggers({"sensor_dropout_rate": SENSOR_FAILURE_DROPOUT_RATE - 0.01})
    check("T03", "dropout_rate just below CEILING does NOT fire SENSOR_FAILURE",
          TRIGGER_SENSOR_FAILURE not in _trigger_names(active_below))


# ---------------------------------------------------------------------------
# T04: perception_quality_level=CRITICAL -> CRITICAL_QUALITY fires
# ---------------------------------------------------------------------------
def test_critical_quality_trigger():
    active, level = evaluate_triggers({"perception_quality_level": CRITICAL})
    check("T04", "perception_quality_level=CRITICAL fires CRITICAL_QUALITY",
          TRIGGER_CRITICAL_QUALITY in _trigger_names(active))
    check("T04", "level returned matches the supplied value",
          level == CRITICAL)
    # GOOD level must not fire CRITICAL_QUALITY
    active_good, _ = evaluate_triggers({"perception_quality_level": GOOD})
    check("T04", "perception_quality_level=GOOD does NOT fire CRITICAL_QUALITY",
          TRIGGER_CRITICAL_QUALITY not in _trigger_names(active_good))


# ---------------------------------------------------------------------------
# T05: sensor_disagreement_distance >= threshold -> SENSOR_DISAGREEMENT fires
# ---------------------------------------------------------------------------
def test_sensor_disagreement_trigger():
    active, _ = evaluate_triggers({"sensor_disagreement_distance": SENSOR_DISAGREEMENT_DISTANCE_M})
    check("T05", "disagreement=threshold fires SENSOR_DISAGREEMENT",
          TRIGGER_SENSOR_DISAGREEMENT in _trigger_names(active))
    below, _ = evaluate_triggers({"sensor_disagreement_distance": SENSOR_DISAGREEMENT_DISTANCE_M - 0.01})
    check("T05", "disagreement just below threshold does NOT fire",
          TRIGGER_SENSOR_DISAGREEMENT not in _trigger_names(below))


# ---------------------------------------------------------------------------
# T06: track_covariance trace >= EXCESSIVE_COVARIANCE_TRACE -> fires
# ---------------------------------------------------------------------------
def test_excessive_covariance_trigger():
    active, _ = evaluate_triggers({"track_covariance": EXCESSIVE_COVARIANCE_TRACE})
    check("T06", "trace=EXCESSIVE_COVARIANCE_TRACE fires EXCESSIVE_COVARIANCE",
          TRIGGER_EXCESSIVE_COVARIANCE in _trigger_names(active))
    below, _ = evaluate_triggers({"track_covariance": EXCESSIVE_COVARIANCE_TRACE - 0.01})
    check("T06", "trace just below threshold does NOT fire",
          TRIGGER_EXCESSIVE_COVARIANCE not in _trigger_names(below))
    # matrix form: 3x3 diagonal with trace = EXCESSIVE_COVARIANCE_TRACE
    d = EXCESSIVE_COVARIANCE_TRACE / 3
    matrix = [[d, 0, 0], [0, d, 0], [0, 0, d]]
    active_m, _ = evaluate_triggers({"track_covariance": matrix})
    check("T06", "matrix covariance with matching trace also fires",
          TRIGGER_EXCESSIVE_COVARIANCE in _trigger_names(active_m))


# ---------------------------------------------------------------------------
# T07: missed_update_count >= ceiling -> REPEATED_MISSED_DETECTIONS fires
# ---------------------------------------------------------------------------
def test_repeated_missed_detections_trigger():
    active, _ = evaluate_triggers(
        {"missed_update_count": REPEATED_MISSED_DETECTIONS_CEILING})
    check("T07", "missed=CEILING fires REPEATED_MISSED_DETECTIONS",
          TRIGGER_REPEATED_MISSED_DETECTIONS in _trigger_names(active))
    below, _ = evaluate_triggers(
        {"missed_update_count": REPEATED_MISSED_DETECTIONS_CEILING - 1})
    check("T07", "missed=CEILING-1 does NOT fire",
          TRIGGER_REPEATED_MISSED_DETECTIONS not in _trigger_names(below))


# ---------------------------------------------------------------------------
# T08: communication_age_steps >= STALE threshold -> STALE_DISTRIBUTED_TRACK
# ---------------------------------------------------------------------------
def test_stale_distributed_track_trigger():
    active, _ = evaluate_triggers(
        {"communication_age_steps": STALE_DISTRIBUTED_TRACK_AGE_STEPS})
    check("T08", "comm_age=STALE_THRESHOLD fires STALE_DISTRIBUTED_TRACK",
          TRIGGER_STALE_DISTRIBUTED_TRACK in _trigger_names(active))
    below, _ = evaluate_triggers(
        {"communication_age_steps": STALE_DISTRIBUTED_TRACK_AGE_STEPS - 1})
    check("T08", "comm_age just below threshold does NOT fire",
          TRIGGER_STALE_DISTRIBUTED_TRACK not in _trigger_names(below))


# ---------------------------------------------------------------------------
# T09: communication_recovered=True -> COMMUNICATION_RECOVERY appended
# ---------------------------------------------------------------------------
def test_communication_recovery_trigger():
    active, _ = evaluate_triggers({"communication_recovered": True})
    check("T09", "communication_recovered=True fires COMMUNICATION_RECOVERY",
          TRIGGER_COMMUNICATION_RECOVERY in _trigger_names(active))
    # Must not fire when False or absent
    active_f, _ = evaluate_triggers({"communication_recovered": False})
    check("T09", "communication_recovered=False does NOT fire",
          TRIGGER_COMMUNICATION_RECOVERY not in _trigger_names(active_f))


# ---------------------------------------------------------------------------
# T10: multiple triggers — precedence order is TRIGGER_PRECEDENCE, not
#      encounter order.  Fire sensor_failure + critical_quality together;
#      SENSOR_FAILURE must come first in active_triggers.
# ---------------------------------------------------------------------------
def test_multi_trigger_precedence():
    signals = {
        "sensor_failed": True,
        "perception_quality_level": CRITICAL,
        "sensor_disagreement_distance": SENSOR_DISAGREEMENT_DISTANCE_M,
    }
    active, _ = evaluate_triggers(signals)
    names = _trigger_names(active)
    # At least all three must be present
    check("T10", "all three triggers fire",
          TRIGGER_SENSOR_FAILURE in names and
          TRIGGER_CRITICAL_QUALITY in names and
          TRIGGER_SENSOR_DISAGREEMENT in names,
          str(names))
    # SENSOR_FAILURE must appear before CRITICAL_QUALITY (precedence order)
    check("T10", "SENSOR_FAILURE precedes CRITICAL_QUALITY in active_triggers list",
          names.index(TRIGGER_SENSOR_FAILURE) < names.index(TRIGGER_CRITICAL_QUALITY),
          str(names))


# ---------------------------------------------------------------------------
# T11–T14: mode selection (_select_mode, exercised via PerceptionHandoffModel.decide)
# ---------------------------------------------------------------------------

def _decide_mode(signals, available_resources=None):
    """Helper: single decide() call, returns the mode string."""
    model = PerceptionHandoffModel()
    d = model.decide(0, 0, signals, available_resources)
    return d["handoff_mode"]


def test_mode_safe_hold_when_no_resources():
    # SENSOR_FAILURE with no resources -> SAFE_HOLD (guaranteed last resort)
    mode = _decide_mode({"sensor_failed": True}, available_resources={})
    check("T11", "sensor failure + no resources -> SAFE_HOLD",
          mode == SAFE_HOLD, f"got {mode!r}")


def test_mode_radar_only_fallback():
    # SENSOR_FAILURE + only radar available -> RADAR_ONLY_FALLBACK
    mode = _decide_mode(
        {"sensor_failed": True},
        available_resources={"radar_available": True})
    check("T12", "sensor failure + radar available -> RADAR_ONLY_FALLBACK",
          mode == RADAR_ONLY_FALLBACK, f"got {mode!r}")


def test_mode_sensor_disagreement_prefers_peer():
    # SENSOR_DISAGREEMENT prefers peer/centralized over local sensors
    # Only peer available:
    mode = _decide_mode(
        {"sensor_disagreement_distance": SENSOR_DISAGREEMENT_DISTANCE_M},
        available_resources={"peer_track_available": True,
                   "radar_available": True, "lidar_available": True})
    check("T13", "sensor disagreement + peer available -> REQUEST_PEER_TRACK (over local sensors)",
          mode == REQUEST_PEER_TRACK, f"got {mode!r}")


def test_mode_stale_distributed_track_excludes_peer():
    # STALE_DISTRIBUTED_TRACK must not pick REQUEST_PEER_TRACK
    # (peer IS the stale source); radar is available -> RADAR_ONLY_FALLBACK
    mode = _decide_mode(
        {"communication_age_steps": STALE_DISTRIBUTED_TRACK_AGE_STEPS},
        available_resources={"radar_available": True,
                   "peer_track_available": True})
    check("T14", "stale distributed track + radar+peer -> RADAR_ONLY (peer excluded)",
          mode == RADAR_ONLY_FALLBACK, f"got {mode!r}")
    # Without radar: falls through to lidar
    mode_lidar = _decide_mode(
        {"communication_age_steps": STALE_DISTRIBUTED_TRACK_AGE_STEPS},
        available_resources={"lidar_available": True, "peer_track_available": True})
    check("T14", "stale distributed track + lidar+peer -> LIDAR_ONLY (peer excluded)",
          mode_lidar == LIDAR_ONLY_FALLBACK, f"got {mode_lidar!r}")


# ---------------------------------------------------------------------------
# T15: healthy track -> NO_HANDOFF
# ---------------------------------------------------------------------------
def test_decide_healthy_no_handoff():
    model = PerceptionHandoffModel()
    d = model.decide(0, 0, {}, available_resources={"radar_available": True})
    check("T15", "healthy track -> NO_HANDOFF",
          d["handoff_mode"] == NO_HANDOFF, f"got {d['handoff_mode']!r}")
    check("T15", "primary_trigger is None for NO_HANDOFF",
          d["primary_trigger"] is None)


# ---------------------------------------------------------------------------
# T16: failed sensor + no resources -> SAFE_HOLD (via decide)
# ---------------------------------------------------------------------------
def test_decide_failed_sensor_safe_hold():
    model = PerceptionHandoffModel()
    d = model.decide(1, 0, {"sensor_failed": True}, available_resources={})
    check("T16", "decide: failed sensor + no resources -> SAFE_HOLD",
          d["handoff_mode"] == SAFE_HOLD, f"got {d['handoff_mode']!r}")
    check("T16", "primary_trigger is SENSOR_FAILURE",
          d["primary_trigger"] == TRIGGER_SENSOR_FAILURE,
          f"got {d['primary_trigger']!r}")


# ---------------------------------------------------------------------------
# T17: failed sensor + radar available -> RADAR_ONLY_FALLBACK
# ---------------------------------------------------------------------------
def test_decide_failed_sensor_radar_fallback():
    model = PerceptionHandoffModel()
    d = model.decide(2, 0, {"sensor_failed": True},
                     available_resources={"radar_available": True})
    check("T17", "decide: failed sensor + radar -> RADAR_ONLY_FALLBACK",
          d["handoff_mode"] == RADAR_ONLY_FALLBACK, f"got {d['handoff_mode']!r}")


# ---------------------------------------------------------------------------
# T18: episode lifecycle — trigger opens, then recovery closes it
# ---------------------------------------------------------------------------
def test_episode_lifecycle_trigger_then_resolve():
    model = PerceptionHandoffModel()
    # Step 5: trigger fires (sensor failed)
    model.decide(0, 5, {"sensor_failed": True}, available_resources={})
    check("T18", "handing_off=True right after trigger",
          model.handing_off(0))
    # Step 10: healthy again
    model.decide(0, 10, {}, available_resources={})
    check("T18", "handing_off=False after healthy step",
          not model.handing_off(0))
    # Log should contain TRIGGERED and RESOLVED events
    events = [e["event"] for e in model.log]
    check("T18", "EVENT_TRIGGERED in log",
          EVENT_TRIGGERED in events, str(events))
    check("T18", "EVENT_RESOLVED in log",
          EVENT_RESOLVED in events, str(events))
    resolved_entry = next(e for e in model.log if e["event"] == EVENT_RESOLVED)
    check("T18", "resolved entry final_outcome = OUTCOME_RECOVERED",
          resolved_entry["final_outcome"] == OUTCOME_RECOVERED,
          f"got {resolved_entry['final_outcome']!r}")
    check("T18", "resolved episode duration_steps = 10 - 5 = 5",
          resolved_entry["duration_steps"] == 5,
          f"got {resolved_entry['duration_steps']!r}")


# ---------------------------------------------------------------------------
# T19: duration_steps increments correctly across consecutive steps
# ---------------------------------------------------------------------------
def test_duration_increments():
    model = PerceptionHandoffModel()
    for t in range(3, 8):
        d = model.decide(0, t, {"sensor_failed": True}, available_resources={})
        expected_duration = t - 3 + 1  # started at step 3
        check("T19", f"step {t} duration_steps = {expected_duration}",
              d["duration_steps"] == expected_duration,
              f"got {d['duration_steps']!r}")


# ---------------------------------------------------------------------------
# T20: handing_off / close_all lifecycle
# ---------------------------------------------------------------------------
def test_handing_off_and_close_all():
    model = PerceptionHandoffModel()
    model.decide(0, 0, {"sensor_failed": True}, available_resources={})
    model.decide(1, 0, {"sensor_failed": True}, available_resources={})
    check("T20", "both uav_ids are handing_off after trigger",
          model.handing_off(0) and model.handing_off(1))
    closed = model.close_all(10)
    check("T20", "close_all returns 2 entries",
          len(closed) == 2, f"got {len(closed)}")
    check("T20", "neither uav_id is handing_off after close_all",
          not model.handing_off(0) and not model.handing_off(1))
    outcomes = {e["final_outcome"] for e in closed}
    check("T20", "final_outcome is OUTCOME_UNRESOLVED_AT_END after force-close",
          outcomes == {OUTCOME_UNRESOLVED_AT_END}, str(outcomes))


# ---------------------------------------------------------------------------
# T21: summary() aggregation
# ---------------------------------------------------------------------------
def test_summary_counts():
    model = PerceptionHandoffModel()
    # UAV 0: trigger at step 0, resolve at step 5
    model.decide(0, 0, {"sensor_failed": True}, available_resources={})
    for t in range(1, 5):
        model.decide(0, t, {"sensor_failed": True}, available_resources={})
    model.decide(0, 5, {}, available_resources={})   # resolve

    # UAV 1: trigger at step 0, force-close at step 8 (unresolved)
    model.decide(1, 0, {"sensor_failed": True}, available_resources={})
    model.close_all(8)

    s = model.summary()
    check("T21", "2 handoff episodes triggered",
          s["handoff_episodes_triggered"] == 2,
          f"got {s['handoff_episodes_triggered']!r}")
    check("T21", "SENSOR_FAILURE is the only primary_trigger seen",
          list(s["primary_trigger_counts"].keys()) == [TRIGGER_SENSOR_FAILURE],
          str(s["primary_trigger_counts"]))
    check("T21", "OUTCOME_RECOVERED counted once",
          s["final_outcome_counts"].get(OUTCOME_RECOVERED, 0) == 1,
          str(s["final_outcome_counts"]))
    check("T21", "OUTCOME_UNRESOLVED_AT_END counted once",
          s["final_outcome_counts"].get(OUTCOME_UNRESOLVED_AT_END, 0) == 1,
          str(s["final_outcome_counts"]))
    check("T21", "avg_resolved_duration_steps = 5 (UAV 0 resolved at step5-0=5)",
          s["avg_resolved_duration_steps"] == 5.0,
          f"got {s['avg_resolved_duration_steps']!r}")


# ---------------------------------------------------------------------------
# T22: no ground-truth parameter on any public method (API safety check)
# ---------------------------------------------------------------------------
def test_no_ground_truth_params():
    model = PerceptionHandoffModel()
    for method_name in ("decide", "decide_for_track_row", "handing_off", "close_all", "summary"):
        if not hasattr(model, method_name):
            continue
        params = inspect.signature(getattr(model, method_name)).parameters
        bad = [p for p in params if "true" in p.lower() or "ground_truth" in p.lower()]
        check("T22", f"{method_name}() has no ground-truth parameter",
              not bad, f"found: {bad}")
    # Same check for the module-level evaluate_triggers function
    params = inspect.signature(evaluate_triggers).parameters
    bad = [p for p in params if "true" in p.lower() or "ground_truth" in p.lower()]
    check("T22", "evaluate_triggers() has no ground-truth parameter",
          not bad, f"found: {bad}")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def main():
    test_no_triggers_for_healthy_track()
    test_sensor_failed_explicit()
    test_sensor_failed_via_dropout_rate()
    test_critical_quality_trigger()
    test_sensor_disagreement_trigger()
    test_excessive_covariance_trigger()
    test_repeated_missed_detections_trigger()
    test_stale_distributed_track_trigger()
    test_communication_recovery_trigger()
    test_multi_trigger_precedence()
    test_mode_safe_hold_when_no_resources()
    test_mode_radar_only_fallback()
    test_mode_sensor_disagreement_prefers_peer()
    test_mode_stale_distributed_track_excludes_peer()
    test_decide_healthy_no_handoff()
    test_decide_failed_sensor_safe_hold()
    test_decide_failed_sensor_radar_fallback()
    test_episode_lifecycle_trigger_then_resolve()
    test_duration_increments()
    test_handing_off_and_close_all()
    test_summary_counts()
    test_no_ground_truth_params()

    _c.print_summary()
    _c.write_markdown(
        "results/handoff_validation_results.md",
        "Handoff Validation Results",
        "Deterministic checks for trigger evaluation, mode selection, "
        "episode lifecycle, and API safety in perception_handoff_model.py.",
    )
    total, passed, _ = _c.summary()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
