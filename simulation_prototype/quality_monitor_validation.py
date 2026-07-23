"""
quality_monitor_validation.py

Task 27: validates perception_quality_monitor.py against deterministic,
hand-computable cases.  Every expected value is derived analytically from
the module's published formulas and thresholds (COVARIANCE_TRACE_REFERENCE,
GOOD_THRESHOLD, CRITICAL_THRESHOLD, SIGNAL_WEIGHTS) so a reviewer can
cross-check by substitution.

  T01 – _score_covariance: known trace values
  T02 – _score_age: 0/partial/mature
  T03 – _score_missed_updates: 0/ceiling/over-ceiling
  T04 – _score_agreement: clamp to [0,1]
  T05 – _score_innovation: zero/gate boundary
  T06 – _score_calibration: zero error / max error
  T07 – _score_communication_age: half-life decay
  T08 – _score_dropout_rate: 0 / ceiling / above
  T09 – _score_trust: clamp to [0,1]
  T10 – None inputs -> None score (every scorer)
  T11 – composite score = weighted average (2-signal case, hand-computed)
  T12 – evaluate: all-good signals -> GOOD
  T13 – evaluate: all-bad signals -> CRITICAL
  T14 – evaluate: no signals at all -> CRITICAL / score=None (fail-safe)
  T15 – evaluate: partial signals -> normalized score, not 0
  T16 – evaluate_track_row: pulls covariance/age/missed_count from row dict
  T17 – evaluate_track_row: JSON-string covariance parsed correctly
  T18 – GOOD > DEGRADED > CRITICAL score ordering for canonical tracks
  T19 – custom weights change composite score in expected direction
  T20 – no ground-truth parameter on any public method (API safety check)

Run directly:
    python quality_monitor_validation.py
"""

import inspect
import json
import math
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from perception_quality_monitor import (
    PerceptionQualityMonitor,
    GOOD, DEGRADED, CRITICAL,
    COVARIANCE_TRACE_REFERENCE, AGE_MATURITY_STEPS,
    MISSED_UPDATE_CEILING, INNOVATION_GATE,
    COMMUNICATION_AGE_HALF_LIFE_STEPS, DROPOUT_RATE_CEILING,
    GOOD_THRESHOLD, CRITICAL_THRESHOLD, SIGNAL_WEIGHTS,
    _score_covariance, _score_age, _score_missed_updates,
    _score_agreement, _score_innovation, _score_calibration,
    _score_communication_age, _score_dropout_rate, _score_trust,
)
from validation_common import Checker

_c = Checker()


def check(task, desc, cond, detail=""):
    return _c.check(task, desc, cond, detail)


def close(a, b, tol=1e-9):
    return _c.close(a, b, tol)


# ---------------------------------------------------------------------------
# T01: _score_covariance
#   score = 1 / (1 + trace / REF)
#   trace = 0   -> 1.0
#   trace = REF -> 0.5
#   trace = 3*REF -> 0.25
# ---------------------------------------------------------------------------
def test_score_covariance():
    REF = COVARIANCE_TRACE_REFERENCE
    check("T01", "trace=0 -> score=1.0",
          close(_score_covariance(0.0), 1.0))
    check("T01", "trace=REF -> score=0.5",
          close(_score_covariance(REF), 0.5))
    check("T01", "trace=3*REF -> score=0.25",
          close(_score_covariance(3 * REF), 0.25))
    check("T01", "negative trace -> None",
          _score_covariance(-1.0) is None)
    check("T01", "None trace -> None",
          _score_covariance(None) is None)
    # matrix form: trace of 2x2 identity = 2
    check("T01", "matrix form: [[1,0],[0,1]] -> same as trace=2",
          close(_score_covariance([[1, 0], [0, 1]]),
                1.0 / (1.0 + 2.0 / REF)))


# ---------------------------------------------------------------------------
# T02: _score_age
#   score = clamp(age / MATURITY, 0, 1)
# ---------------------------------------------------------------------------
def test_score_age():
    check("T02", "age=0 -> score=0.0",
          close(_score_age(0), 0.0))
    check("T02", "age=MATURITY -> score=1.0",
          close(_score_age(AGE_MATURITY_STEPS), 1.0))
    check("T02", "age=MATURITY//2 -> score=0.5",
          close(_score_age(AGE_MATURITY_STEPS // 2),
                (AGE_MATURITY_STEPS // 2) / AGE_MATURITY_STEPS))
    check("T02", "age > MATURITY -> clamped to 1.0",
          close(_score_age(AGE_MATURITY_STEPS * 10), 1.0))
    check("T02", "None age -> None",
          _score_age(None) is None)


# ---------------------------------------------------------------------------
# T03: _score_missed_updates
#   score = max(0, 1 - missed / CEILING)
# ---------------------------------------------------------------------------
def test_score_missed_updates():
    check("T03", "missed=0 -> score=1.0",
          close(_score_missed_updates(0), 1.0))
    check("T03", "missed=CEILING -> score=0.0",
          close(_score_missed_updates(MISSED_UPDATE_CEILING), 0.0))
    check("T03", "missed > CEILING -> clamped to 0.0",
          close(_score_missed_updates(MISSED_UPDATE_CEILING + 5), 0.0))
    check("T03", "missed=1 -> correct partial score",
          close(_score_missed_updates(1),
                max(0.0, 1.0 - 1 / MISSED_UPDATE_CEILING)))
    check("T03", "None missed -> None",
          _score_missed_updates(None) is None)


# ---------------------------------------------------------------------------
# T04: _score_agreement — straight pass-through, clamped [0,1]
# ---------------------------------------------------------------------------
def test_score_agreement():
    check("T04", "agreement=1.0 -> 1.0",
          close(_score_agreement(1.0), 1.0))
    check("T04", "agreement=0.0 -> 0.0",
          close(_score_agreement(0.0), 0.0))
    check("T04", "agreement=0.7 -> 0.7",
          close(_score_agreement(0.7), 0.7))
    check("T04", "agreement > 1 -> clamped to 1.0",
          close(_score_agreement(1.5), 1.0))
    check("T04", "agreement < 0 -> clamped to 0.0",
          close(_score_agreement(-0.3), 0.0))
    check("T04", "None agreement -> None",
          _score_agreement(None) is None)


# ---------------------------------------------------------------------------
# T05: _score_innovation
#   score = max(0, 1 - innovation / GATE)
# ---------------------------------------------------------------------------
def test_score_innovation():
    check("T05", "innovation=0 -> score=1.0",
          close(_score_innovation(0.0), 1.0))
    check("T05", "innovation=GATE -> score=0.0",
          close(_score_innovation(INNOVATION_GATE), 0.0))
    check("T05", "innovation > GATE -> clamped to 0.0",
          close(_score_innovation(INNOVATION_GATE * 2), 0.0))
    check("T05", "innovation=GATE/2 -> score=0.5",
          close(_score_innovation(INNOVATION_GATE / 2), 0.5))
    check("T05", "None -> None",
          _score_innovation(None) is None)
    check("T05", "negative -> None",
          _score_innovation(-1.0) is None)


# ---------------------------------------------------------------------------
# T06: _score_calibration
#   score = max(0, 1 - error)
# ---------------------------------------------------------------------------
def test_score_calibration():
    check("T06", "error=0.0 -> score=1.0",
          close(_score_calibration(0.0), 1.0))
    check("T06", "error=1.0 -> score=0.0",
          close(_score_calibration(1.0), 0.0))
    check("T06", "error=0.3 -> score=0.7",
          close(_score_calibration(0.3), 0.7))
    check("T06", "error > 1 -> clamped to 0.0",
          close(_score_calibration(1.5), 0.0))
    check("T06", "None -> None",
          _score_calibration(None) is None)
    check("T06", "negative -> None",
          _score_calibration(-0.1) is None)


# ---------------------------------------------------------------------------
# T07: _score_communication_age
#   score = 1 / (1 + age / HALF_LIFE)
#   age=0     -> 1.0
#   age=HALF_LIFE -> 0.5
#   age=2*HALF_LIFE -> 1/3
# ---------------------------------------------------------------------------
def test_score_communication_age():
    HL = COMMUNICATION_AGE_HALF_LIFE_STEPS
    check("T07", "age=0 -> score=1.0",
          close(_score_communication_age(0), 1.0))
    check("T07", "age=HALF_LIFE -> score=0.5",
          close(_score_communication_age(HL), 0.5))
    check("T07", "age=2*HALF_LIFE -> score=1/3",
          close(_score_communication_age(2 * HL), 1.0 / 3.0))
    check("T07", "None -> None",
          _score_communication_age(None) is None)


# ---------------------------------------------------------------------------
# T08: _score_dropout_rate
#   score = max(0, 1 - rate / CEILING)  (CEILING=0.5)
# ---------------------------------------------------------------------------
def test_score_dropout_rate():
    check("T08", "rate=0.0 -> score=1.0",
          close(_score_dropout_rate(0.0), 1.0))
    check("T08", "rate=CEILING -> score=0.0",
          close(_score_dropout_rate(DROPOUT_RATE_CEILING), 0.0))
    check("T08", "rate > CEILING -> clamped to 0.0",
          close(_score_dropout_rate(DROPOUT_RATE_CEILING + 0.3), 0.0))
    check("T08", "rate=0.25 -> score=0.5 (half of ceiling)",
          close(_score_dropout_rate(DROPOUT_RATE_CEILING / 2), 0.5))
    check("T08", "None -> None",
          _score_dropout_rate(None) is None)


# ---------------------------------------------------------------------------
# T09: _score_trust — straight clamp [0,1]
# ---------------------------------------------------------------------------
def test_score_trust():
    check("T09", "trust=1.0 -> 1.0",
          close(_score_trust(1.0), 1.0))
    check("T09", "trust=0.0 -> 0.0",
          close(_score_trust(0.0), 0.0))
    check("T09", "trust=0.75 -> 0.75",
          close(_score_trust(0.75), 0.75))
    check("T09", "trust > 1 -> clamped to 1.0",
          close(_score_trust(1.2), 1.0))
    check("T09", "trust < 0 -> clamped to 0.0",
          close(_score_trust(-0.1), 0.0))
    check("T09", "None -> None",
          _score_trust(None) is None)


# ---------------------------------------------------------------------------
# T10: None inputs -> None scores across all scorers (already covered per
#      scorer above, but this one explicit grouped check catches new scorers
#      added without a None guard)
# ---------------------------------------------------------------------------
def test_none_inputs_yield_none():
    # Calling each scorer with an all-None signals dict
    monitor = PerceptionQualityMonitor()
    composite, per_signal = monitor.score({
        "track_covariance": None,
        "track_age": None,
        "missed_update_count": None,
        "sensor_agreement": None,
        "innovation_magnitude": None,
        "confidence_calibration_error": None,
        "communication_age_steps": None,
        "sensor_dropout_rate": None,
        "current_trust_value": None,
    })
    check("T10", "all-None signals -> composite=None",
          composite is None)
    check("T10", "every per-signal score is None",
          all(v is None for v in per_signal.values()),
          str({k: v for k, v in per_signal.items() if v is not None}))


# ---------------------------------------------------------------------------
# T11: composite score = weighted average (2-signal case, hand-computed)
#   signals: trust=1.0 (weight 1.0), missed_updates=0 (weight 1.0)
#   expected = (1.0*1.0 + 1.0*1.0) / (1.0+1.0) = 1.0
# ---------------------------------------------------------------------------
def test_composite_weighted_average():
    monitor = PerceptionQualityMonitor()
    composite, _ = monitor.score({
        "current_trust_value": 1.0,
        "missed_update_count": 0,
    })
    check("T11", "2-signal composite = 1.0 when both signals are perfect",
          composite is not None and close(composite, 1.0),
          f"got {composite!r}")

    # asymmetric: trust=1.0 (w=1.0), age=0 (score=0, w=0.5)
    # composite = (1.0*1.0 + 0.5*0) / (1.0+0.5) = 1.0/1.5 = 2/3
    composite2, _ = monitor.score({
        "current_trust_value": 1.0,
        "track_age": 0,
    })
    expected2 = (1.0 * 1.0 + 0.5 * 0.0) / (1.0 + 0.5)
    check("T11", "trust=1 age=0 composite = 2/3 per SIGNAL_WEIGHTS",
          composite2 is not None and close(composite2, expected2, tol=1e-9),
          f"got {composite2!r}, expected {expected2!r}")


# ---------------------------------------------------------------------------
# T12: evaluate -> GOOD for all-perfect signals
# ---------------------------------------------------------------------------
def test_evaluate_good():
    monitor = PerceptionQualityMonitor()
    level, score, _ = monitor.evaluate({
        "track_covariance": 0.01,
        "track_age": 100,
        "missed_update_count": 0,
        "sensor_agreement": 1.0,
        "innovation_magnitude": 0.0,
        "confidence_calibration_error": 0.0,
        "communication_age_steps": 0,
        "sensor_dropout_rate": 0.0,
        "current_trust_value": 1.0,
    })
    check("T12", "all-perfect signals -> GOOD",
          level == GOOD, f"got {level!r}, score={score!r}")
    check("T12", "all-perfect score >= GOOD_THRESHOLD",
          score is not None and score >= GOOD_THRESHOLD,
          f"score={score!r}")


# ---------------------------------------------------------------------------
# T13: evaluate -> CRITICAL for all-bad signals
# ---------------------------------------------------------------------------
def test_evaluate_critical():
    monitor = PerceptionQualityMonitor()
    level, score, _ = monitor.evaluate({
        "track_covariance": 500.0,
        "track_age": 0,
        "missed_update_count": MISSED_UPDATE_CEILING,
        "sensor_agreement": 0.0,
        "innovation_magnitude": INNOVATION_GATE * 10,
        "confidence_calibration_error": 1.0,
        "communication_age_steps": 1000,
        "sensor_dropout_rate": 1.0,
        "current_trust_value": 0.0,
    })
    check("T13", "all-bad signals -> CRITICAL",
          level == CRITICAL, f"got {level!r}, score={score!r}")
    check("T13", "all-bad score < CRITICAL_THRESHOLD",
          score is not None and score < CRITICAL_THRESHOLD,
          f"score={score!r}")


# ---------------------------------------------------------------------------
# T14: evaluate -> CRITICAL / score=None when no signals at all
# ---------------------------------------------------------------------------
def test_evaluate_no_signals():
    monitor = PerceptionQualityMonitor()
    level, score, per_signal = monitor.evaluate({})
    check("T14", "no signals -> CRITICAL (fail-safe, not GOOD)",
          level == CRITICAL, f"got {level!r}")
    check("T14", "no signals -> score is None",
          score is None, f"got {score!r}")


# ---------------------------------------------------------------------------
# T15: partial signals -> composite is normalized to [0,1], not 0
#   Providing only trust=1.0 should give composite=1.0, not 0
#   (the weight total must be non-zero for the average to work)
# ---------------------------------------------------------------------------
def test_partial_signals_normalized():
    monitor = PerceptionQualityMonitor()
    composite, _ = monitor.score({"current_trust_value": 1.0})
    check("T15", "single perfect signal -> composite=1.0, not diluted by absent signals",
          composite is not None and close(composite, 1.0, tol=1e-9),
          f"got {composite!r}")


# ---------------------------------------------------------------------------
# T16: evaluate_track_row — pulls track fields from row dict
# ---------------------------------------------------------------------------
def test_evaluate_track_row_dict():
    monitor = PerceptionQualityMonitor()
    row = {
        "covariance": 0.5,      # scalar trace
        "age": 10,
        "missed_count": 0,
    }
    level, score, per_signal = monitor.evaluate_track_row(
        row,
        sensor_agreement=1.0,
        current_trust_value=0.9,
    )
    check("T16", "evaluate_track_row with scalar covariance returns a level",
          level in (GOOD, DEGRADED, CRITICAL), f"got {level!r}")
    check("T16", "covariance signal is not None",
          per_signal["covariance"] is not None)
    check("T16", "age signal is not None",
          per_signal["age"] is not None)
    check("T16", "missed_updates signal is not None",
          per_signal["missed_updates"] is not None)


# ---------------------------------------------------------------------------
# T17: evaluate_track_row — JSON-string covariance parsed correctly
#   [[0.1,0,0,0],[0,0.1,0,0],[0,0,1,0],[0,0,0,1]] -> trace = 2.2
# ---------------------------------------------------------------------------
def test_evaluate_track_row_json_covariance():
    cov_matrix = [[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    expected_trace = 0.1 + 0.1 + 1.0 + 1.0  # 2.2
    monitor = PerceptionQualityMonitor()
    row = {"covariance": json.dumps(cov_matrix), "age": 5, "missed_count": 0}
    _, _, per_signal = monitor.evaluate_track_row(row)
    expected_cov_score = 1.0 / (1.0 + expected_trace / COVARIANCE_TRACE_REFERENCE)
    check("T17", "JSON-string covariance parsed: cov_score matches manual trace",
          per_signal["covariance"] is not None and
          close(per_signal["covariance"], expected_cov_score, tol=1e-9),
          f"got {per_signal['covariance']!r}, expected {expected_cov_score!r}")


# ---------------------------------------------------------------------------
# T18: DEMO_TRACKS canonical ordering: healthy > new > coasting > stale > unreliable
# ---------------------------------------------------------------------------
def test_canonical_track_ordering():
    from perception_quality_monitor import DEMO_TRACKS
    monitor = PerceptionQualityMonitor()
    scores = {name: monitor.score(signals)[0]
              for name, signals in DEMO_TRACKS.items()
              if signals}  # exclude no_signals_available (score=None)
    # "healthy_confirmed_track" must be the highest
    check("T18", "healthy_confirmed_track has highest composite score",
          scores["healthy_confirmed_track"] == max(scores.values()),
          str({k: round(v, 4) for k, v in scores.items()}))
    # "unreliable_sensor" must be the lowest
    check("T18", "unreliable_sensor has lowest composite score",
          scores["unreliable_sensor"] == min(scores.values()),
          str({k: round(v, 4) for k, v in scores.items()}))


# ---------------------------------------------------------------------------
# T19: custom weights shift composite in expected direction
#   If we zero out every weight except "trust" then composite == trust_score
# ---------------------------------------------------------------------------
def test_custom_weights():
    only_trust = {k: 0.0 for k in SIGNAL_WEIGHTS}
    only_trust["trust"] = 1.0
    monitor = PerceptionQualityMonitor(weights=only_trust)
    trust_val = 0.65
    composite, _ = monitor.score({
        "current_trust_value": trust_val,
        "track_covariance": 100.0,   # would tank composite with default weights
        "track_age": 0,              # ditto
    })
    check("T19", "custom weights (trust-only) -> composite == trust_score",
          composite is not None and close(composite, trust_val, tol=1e-9),
          f"got {composite!r}, expected {trust_val!r}")


# ---------------------------------------------------------------------------
# T20: no ground-truth parameter on any public method
# ---------------------------------------------------------------------------
def test_no_ground_truth_params():
    monitor = PerceptionQualityMonitor()
    for method_name in ("evaluate", "score", "evaluate_track_row"):
        params = inspect.signature(getattr(monitor, method_name)).parameters
        bad = [p for p in params if "true" in p.lower() or "ground_truth" in p.lower()]
        check("T20", f"{method_name}() has no ground-truth parameter",
              not bad, f"found: {bad}")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------
def main():
    test_score_covariance()
    test_score_age()
    test_score_missed_updates()
    test_score_agreement()
    test_score_innovation()
    test_score_calibration()
    test_score_communication_age()
    test_score_dropout_rate()
    test_score_trust()
    test_none_inputs_yield_none()
    test_composite_weighted_average()
    test_evaluate_good()
    test_evaluate_critical()
    test_evaluate_no_signals()
    test_partial_signals_normalized()
    test_evaluate_track_row_dict()
    test_evaluate_track_row_json_covariance()
    test_canonical_track_ordering()
    test_custom_weights()
    test_no_ground_truth_params()

    _c.print_summary()
    _c.write_markdown(
        "results/quality_monitor_validation_results.md",
        "Quality Monitor Validation Results",
        "Deterministic checks for every scoring function and the "
        "composite evaluator in perception_quality_monitor.py.",
    )
    total, passed, _ = _c.summary()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
