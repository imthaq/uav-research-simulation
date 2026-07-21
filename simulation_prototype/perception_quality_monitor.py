"""Task 12: runtime perception-quality monitor.

Estimates how much a downstream consumer (fusion, planning, control)
should currently trust a track's perception right now, using only
signals a real system already has on hand at runtime: track covariance,
track age, missed-update count, sensor agreement, innovation/residual
magnitude, confidence calibration error, communication age, sensor
dropout rate, and the existing dynamic trust value (see
fusion/fusion_model.py's TrustTracker, which already computes several of
these per step).

It NEVER uses true position error. There is no ground-truth parameter
anywhere in this module's public API - not "unused for now", structurally
absent - so a caller cannot even accidentally plumb truth into the score.
_self_check() asserts this directly by inspecting the public method
signatures.

Output is one of GOOD / DEGRADED / CRITICAL per track per step.
"""
import argparse
import json

GOOD = "GOOD"
DEGRADED = "DEGRADED"
CRITICAL = "CRITICAL"

# Reference scales reuse the conventions already established for these
# same quantities elsewhere in the codebase, rather than inventing fresh
# ones - a covariance trace of COVARIANCE_TRACE_REFERENCE already means
# "borderline trustworthy" in fusion/fusion_model.py's TrustTracker, and
# an innovation of INNOVATION_GATE is exactly the chi-square gate
# tracking/radar_track_model.py already association-gates matches
# against - so they mean the same thing here.
COVARIANCE_TRACE_REFERENCE = 4.0       # matches fusion_model.TRUST_COVARIANCE_REFERENCE
AGE_MATURITY_STEPS = 3                 # a track needs about this many hits to be past "brand new"
MISSED_UPDATE_CEILING = 3              # matches tracking/radar_track_model.MAX_MISSED (track is deleted at this point anyway)
INNOVATION_GATE = 9.21                 # matches tracking/radar_track_model.GATE_CHI2 (2-DOF, ~99%)
COMMUNICATION_AGE_HALF_LIFE_STEPS = 8.0
DROPOUT_RATE_CEILING = 0.5             # a rolling dropout rate at/above this scores 0

# How the nine signals are weighted into one composite score. Kept close
# to equal (matching fusion_model.TRUST_SIGNAL_WEIGHTS's "equal unless
# tuned" convention) with two mild exceptions: a young track is
# inherently uncertain but shouldn't alone tank the score the way a
# clearly-bad signal should, and communication age matters less than the
# radar's own local signals when a track is being scored by its own
# sensor rather than relayed from a peer.
SIGNAL_WEIGHTS = {
    "covariance": 1.0,
    "age": 0.5,
    "missed_updates": 1.0,
    "agreement": 1.0,
    "innovation": 1.0,
    "calibration": 1.0,
    "communication_age": 0.75,
    "dropout_rate": 1.0,
    "trust": 1.0,
}

# Composite-score thresholds: >= GOOD_THRESHOLD is GOOD, >=
# CRITICAL_THRESHOLD (and below GOOD_THRESHOLD) is DEGRADED, below
# CRITICAL_THRESHOLD is CRITICAL.
GOOD_THRESHOLD = 0.7
CRITICAL_THRESHOLD = 0.4


def _covariance_trace(covariance):
    """Accepts either a precomputed trace (a number) or a covariance
    matrix (list of lists, e.g. RadarTrack.as_row's parsed "covariance"
    field), and returns the trace. Plain diagonal sum - no numpy needed
    for this."""
    if covariance is None:
        return None
    if isinstance(covariance, (int, float)):
        return float(covariance)
    try:
        return float(sum(row[i] for i, row in enumerate(covariance)))
    except (TypeError, IndexError):
        return None


def _score_covariance(covariance):
    trace = _covariance_trace(covariance)
    if trace is None or trace < 0:
        return None
    return 1.0 / (1.0 + trace / COVARIANCE_TRACE_REFERENCE)


def _score_age(track_age):
    if track_age is None:
        return None
    return max(0.0, min(1.0, track_age / AGE_MATURITY_STEPS))


def _score_missed_updates(missed_update_count):
    if missed_update_count is None:
        return None
    return max(0.0, 1.0 - missed_update_count / MISSED_UPDATE_CEILING)


def _score_agreement(sensor_agreement):
    if sensor_agreement is None:
        return None
    return max(0.0, min(1.0, sensor_agreement))


def _score_innovation(innovation_magnitude):
    """innovation_magnitude is expected on the same scale as
    RadarTrack.mahalanobis_sq (squared Mahalanobis distance - already
    normalized by measurement covariance, not a raw meters residual)."""
    if innovation_magnitude is None or innovation_magnitude < 0:
        return None
    return max(0.0, 1.0 - innovation_magnitude / INNOVATION_GATE)


def _score_calibration(confidence_calibration_error):
    """confidence_calibration_error is expected as an ECE/Brier-style
    error in [0, 1], 0 = perfectly calibrated (e.g. from
    radar_confidence_calibration.metrics_for_pairs)."""
    if confidence_calibration_error is None or confidence_calibration_error < 0:
        return None
    return max(0.0, 1.0 - confidence_calibration_error)


def _score_communication_age(communication_age_steps):
    if communication_age_steps is None:
        return None
    return 1.0 / (1.0 + communication_age_steps / COMMUNICATION_AGE_HALF_LIFE_STEPS)


def _score_dropout_rate(sensor_dropout_rate):
    if sensor_dropout_rate is None:
        return None
    return max(0.0, 1.0 - min(1.0, sensor_dropout_rate) / DROPOUT_RATE_CEILING)


def _score_trust(current_trust_value):
    if current_trust_value is None:
        return None
    return max(0.0, min(1.0, current_trust_value))


_SCORERS = {
    "covariance": lambda s: _score_covariance(s.get("track_covariance")),
    "age": lambda s: _score_age(s.get("track_age")),
    "missed_updates": lambda s: _score_missed_updates(s.get("missed_update_count")),
    "agreement": lambda s: _score_agreement(s.get("sensor_agreement")),
    "innovation": lambda s: _score_innovation(s.get("innovation_magnitude")),
    "calibration": lambda s: _score_calibration(s.get("confidence_calibration_error")),
    "communication_age": lambda s: _score_communication_age(s.get("communication_age_steps")),
    "dropout_rate": lambda s: _score_dropout_rate(s.get("sensor_dropout_rate")),
    "trust": lambda s: _score_trust(s.get("current_trust_value")),
}


class PerceptionQualityMonitor:
    """Estimates runtime perception quality (GOOD/DEGRADED/CRITICAL) from
    self-reported track/sensor signals only.

    ponytail: the composite score is a weighted average of independently
    -normalized per-signal scores, not a learned or physically-derived
    fusion of them - same simplification fusion_model.TrustTracker
    already makes with TRUST_SIGNAL_WEIGHTS. Upgrade path: replace the
    weighted average with something that models signal correlation/
    redundancy (e.g. a small calibrated classifier trained on outcome
    labels) if the flat average misclassifies in practice.

    Never reads or accepts true position, true track error, or any other
    ground-truth field - there is no such parameter anywhere on this
    class, by design (see _self_check for a signature-level assertion of
    this).
    """

    def __init__(self, weights=None, good_threshold=GOOD_THRESHOLD,
                 critical_threshold=CRITICAL_THRESHOLD):
        self.weights = dict(weights or SIGNAL_WEIGHTS)
        self.good_threshold = good_threshold
        self.critical_threshold = critical_threshold

    def score(self, signals):
        """signals: dict, any subset of track_covariance, track_age,
        missed_update_count, sensor_agreement, innovation_magnitude,
        confidence_calibration_error, communication_age_steps,
        sensor_dropout_rate, current_trust_value. Missing/None entries
        are simply left out of the weighted average (both numerator and
        weight total), so partial information still yields a normalized
        [0, 1] score instead of silently skewing toward whichever
        signals happen to be present.

        Returns (composite_score, per_signal_scores). composite_score is
        None if no usable signal was supplied at all."""
        per_signal = {}
        weighted_sum = 0.0
        weight_total = 0.0
        for name, scorer in _SCORERS.items():
            value = scorer(signals)
            per_signal[name] = value
            if value is None:
                continue
            w = self.weights.get(name, 0.0)
            weighted_sum += w * value
            weight_total += w
        composite = weighted_sum / weight_total if weight_total > 0 else None
        return composite, per_signal

    def evaluate(self, signals):
        """Returns (level, composite_score, per_signal_scores), level in
        {GOOD, DEGRADED, CRITICAL}. No usable signal at all is treated as
        CRITICAL, not GOOD - an unmonitorable track must not be reported
        as a healthy one just because every input happened to be
        missing."""
        composite, per_signal = self.score(signals)
        if composite is None:
            return CRITICAL, None, per_signal
        if composite >= self.good_threshold:
            level = GOOD
        elif composite >= self.critical_threshold:
            level = DEGRADED
        else:
            level = CRITICAL
        return level, composite, per_signal

    def evaluate_track_row(self, track_row, sensor_agreement=None,
                            innovation_magnitude=None,
                            confidence_calibration_error=None,
                            communication_age_steps=None,
                            sensor_dropout_rate=None,
                            current_trust_value=None):
        """Convenience wrapper: pulls track_covariance/track_age/
        missed_update_count straight out of a tracking/radar_track_model.py
        row (RadarTrack.as_row / RadarTracker.update output). Every other
        signal lives outside a single track row - fusion agreement,
        comms/dropout stats, the dynamic trust score - so those are taken
        as explicit keyword arguments, making it obvious at the call site
        which other system each one has to come from (e.g.
        current_trust_value from a fusion_model.TrustTracker.get(radar_id)
        call, sensor_agreement from that same tracker's
        last_signals(radar_id)["agreement"])."""
        cov = track_row.get("covariance")
        if isinstance(cov, str):
            try:
                cov = json.loads(cov)
            except (ValueError, TypeError):
                cov = None
        signals = {
            "track_covariance": cov,
            "track_age": track_row.get("age"),
            "missed_update_count": track_row.get("missed_count"),
            "sensor_agreement": sensor_agreement,
            "innovation_magnitude": innovation_magnitude,
            "confidence_calibration_error": confidence_calibration_error,
            "communication_age_steps": communication_age_steps,
            "sensor_dropout_rate": sensor_dropout_rate,
            "current_trust_value": current_trust_value,
        }
        return self.evaluate(signals)


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
# A handful of representative synthetic tracks, printed as a table by
# default so `python perception_quality_monitor.py` shows something
# concrete rather than just a pass/fail line. Not test cases (those live
# in _self_check) - these exist purely to demonstrate the per-signal
# breakdown end to end without needing a live simulation run wired up.
DEMO_TRACKS = {
    "healthy_confirmed_track": {
        "track_covariance": 0.2, "track_age": 20, "missed_update_count": 0,
        "sensor_agreement": 1.0, "innovation_magnitude": 0.5,
        "confidence_calibration_error": 0.02, "communication_age_steps": 0,
        "sensor_dropout_rate": 0.0, "current_trust_value": 1.0,
    },
    "new_tentative_track": {
        "track_covariance": 1.5, "track_age": 1, "missed_update_count": 0,
        "sensor_agreement": 0.8, "innovation_magnitude": 2.0,
        "confidence_calibration_error": 0.1, "communication_age_steps": 1,
        "sensor_dropout_rate": 0.0, "current_trust_value": 0.6,
    },
    "coasting_after_misses": {
        "track_covariance": 3.0, "track_age": 15, "missed_update_count": 2,
        "sensor_agreement": 0.5, "innovation_magnitude": 6.0,
        "confidence_calibration_error": 0.2, "communication_age_steps": 4,
        "sensor_dropout_rate": 0.2, "current_trust_value": 0.5,
    },
    "stale_relayed_track": {
        "track_covariance": 1.0, "track_age": 10, "missed_update_count": 0,
        "sensor_agreement": 0.7, "innovation_magnitude": 1.0,
        "confidence_calibration_error": 0.05, "communication_age_steps": 30,
        "sensor_dropout_rate": 0.1, "current_trust_value": 0.7,
    },
    "unreliable_sensor": {
        "track_covariance": 50.0, "track_age": 1, "missed_update_count": 3,
        "sensor_agreement": 0.0, "innovation_magnitude": 20.0,
        "confidence_calibration_error": 0.9, "communication_age_steps": 40,
        "sensor_dropout_rate": 0.9, "current_trust_value": 0.05,
    },
    "no_signals_available": {},
}

_SIGNAL_COLUMNS = list(_SCORERS.keys())


_COL_W = max(len(c) for c in _SIGNAL_COLUMNS) + 2


def _fmt(value):
    return "-" if value is None else f"{value:.3f}"


def _print_table(named_signals, monitor=None):
    monitor = monitor or PerceptionQualityMonitor()
    name_w = max([len(n) for n in named_signals] + [len("track")]) + 2

    header = f"{'track':<{name_w}}" + "".join(f"{c:>{_COL_W}}" for c in _SIGNAL_COLUMNS) + f"{'score':>9}{'level':>11}"
    print(header)
    print("-" * len(header))
    for name, signals in named_signals.items():
        level, score, per_signal = monitor.evaluate(signals)
        row = f"{name:<{name_w}}"
        row += "".join(f"{_fmt(per_signal[c]):>{_COL_W}}" for c in _SIGNAL_COLUMNS)
        row += f"{_fmt(score):>9}{level:>11}"
        print(row)


def _run_cli():
    parser = argparse.ArgumentParser(
        description="Evaluate runtime perception quality (GOOD/DEGRADED/CRITICAL) "
                     "from self-reported track/sensor signals - never true position error.")
    parser.add_argument(
        "--signals", default=None,
        help="JSON object of one track's signals to evaluate, e.g. "
             '\'{"track_covariance": 0.5, "track_age": 10, "current_trust_value": 0.9}\'. '
             "Prints a single-row table for it. Omit to print the built-in demo table instead.")
    parser.add_argument(
        "--self-check", action="store_true",
        help="Run the assert-based self-check instead of printing a table.")
    args = parser.parse_args()

    if args.self_check:
        _self_check()
        return

    if args.signals is not None:
        try:
            signals = json.loads(args.signals)
        except json.JSONDecodeError as e:
            parser.error(f"--signals is not valid JSON: {e}")
        _print_table({"input": signals})
        return

    _print_table(DEMO_TRACKS)


def _self_check():
    """ponytail: smallest thing that fails if the scoring logic breaks -
    not a full test suite. Run directly: python perception_quality_monitor.py"""
    monitor = PerceptionQualityMonitor()

    good_signals = {
        "track_covariance": 0.2,
        "track_age": 20,
        "missed_update_count": 0,
        "sensor_agreement": 1.0,
        "innovation_magnitude": 0.5,
        "confidence_calibration_error": 0.02,
        "communication_age_steps": 0,
        "sensor_dropout_rate": 0.0,
        "current_trust_value": 1.0,
    }
    level, score, _ = monitor.evaluate(good_signals)
    assert level == GOOD, f"expected GOOD, got {level} (score={score})"

    bad_signals = {
        "track_covariance": 50.0,
        "track_age": 1,
        "missed_update_count": 3,
        "sensor_agreement": 0.0,
        "innovation_magnitude": 20.0,
        "confidence_calibration_error": 0.9,
        "communication_age_steps": 40,
        "sensor_dropout_rate": 0.9,
        "current_trust_value": 0.05,
    }
    level, score, _ = monitor.evaluate(bad_signals)
    assert level == CRITICAL, f"expected CRITICAL, got {level} (score={score})"

    # No signals at all -> CRITICAL/None, not GOOD.
    level, score, _ = monitor.evaluate({})
    assert level == CRITICAL and score is None, (level, score)

    # A mixed case should land strictly between the good and bad scores.
    mixed_signals = {
        "track_covariance": 4.0,
        "track_age": 3,
        "missed_update_count": 1,
        "sensor_agreement": 0.6,
    }
    _, mixed_score, _ = monitor.evaluate(mixed_signals)
    _, good_score, _ = monitor.evaluate(good_signals)
    _, bad_score, _ = monitor.evaluate(bad_signals)
    assert bad_score < mixed_score < good_score, (bad_score, mixed_score, good_score)

    # evaluate_track_row pulls covariance/age/missed_count out of a
    # radar_track_model-shaped row (covariance as the JSON string
    # RadarTrack.as_row actually produces).
    track_row = {
        "covariance": json.dumps([[0.1, 0.0, 0.0, 0.0],
                                   [0.0, 0.1, 0.0, 0.0],
                                   [0.0, 0.0, 1.0, 0.0],
                                   [0.0, 0.0, 0.0, 1.0]]),
        "age": 10,
        "missed_count": 0,
    }
    level, score, per_signal = monitor.evaluate_track_row(
        track_row, sensor_agreement=0.9, current_trust_value=0.95)
    assert level in (GOOD, DEGRADED), (level, score)
    assert per_signal["covariance"] is not None

    # No ground-truth parameter anywhere on the public API.
    import inspect
    for name in ("evaluate", "score", "evaluate_track_row"):
        params = inspect.signature(getattr(monitor, name)).parameters
        assert not any("true" in p.lower() or "ground_truth" in p.lower() for p in params), (
            f"{name} must not accept a ground-truth parameter")

    print("perception_quality_monitor: all self-checks passed")


if __name__ == "__main__":
    _run_cli()