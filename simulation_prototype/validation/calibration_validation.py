import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from metrics_analysis import (
    confidence_calibration_metrics,
    _calibration_pairs,
    _reliability_bins,
)
from models.radar_like_model import calibration_pairs as radar_calibration_pairs
from validation.validation_common import Checker

_c = Checker()


def check(task, desc, cond, detail=""):
    return _c.check(task, desc, cond, detail)


def close(a, b, tol=1e-9):
    return _c.close(a, b, tol)


# ---------------------------------------------------------------------------
# helpers to build minimal row dicts
# ---------------------------------------------------------------------------

def _row(status, pd=0.8, false_alarm=False, dropout=False, pd_miss=False,
         confidence_correct=None, confidence_score=None):
    """Minimal row that _calibration_pairs and radar_calibration_pairs read."""
    return {
        "detection_status": status,
        "probability_of_detection": pd,
        "false_alarm_flag": false_alarm,
        "dropout_flag": dropout,
        "radar_pd_miss_flag": pd_miss,
        # fields for radar_like_model.calibration_pairs
        "confidence_score": confidence_score,
        "confidence_correct": confidence_correct,
    }


# ---------------------------------------------------------------------------
# T01: _calibration_pairs filtering rules
# ---------------------------------------------------------------------------

def test_calibration_pair_filtering():
    rows = [
        _row("detected",  pd=0.9),               # -> included
        _row("missed",    pd=0.7),               # -> included
        _row("detected",  pd=0.8, false_alarm=True),  # excluded: false alarm
        _row("detected",  pd=0.8, dropout=True),     # excluded: dropout
        _row("detected",  pd=0.8, pd_miss=True),     # excluded: pd_miss
        _row("false_alarm", pd=0.5),              # excluded: bad status
        _row("unknown",   pd=0.5),               # excluded: bad status
        _row("detected",  pd=None),              # excluded: pd is None
    ]
    pairs = _calibration_pairs(rows)
    check("T01", "detected+missed included; bad rows excluded",
          len(pairs) == 2, f"got {len(pairs)}")
    if len(pairs) == 2:
        check("T01", "first pair is (0.9, True)",
              pairs[0] == (0.9, True))
        check("T01", "second pair is (0.7, False)",
              pairs[1] == (0.7, False))


# ---------------------------------------------------------------------------
# T02: Brier score — hand-computable with 2 pairs
# ---------------------------------------------------------------------------
#   pair1: conf=0.8, correct=True  -> (0.8-1)^2 = 0.04
#   pair2: conf=0.4, correct=False -> (0.4-0)^2 = 0.16
#   brier = mean(0.04, 0.16) = 0.10
def test_brier_score():
    rows = [
        _row("detected", pd=0.8),   # correct=True
        _row("missed",   pd=0.4),   # correct=False
    ]
    m = confidence_calibration_metrics(rows, num_bins=1)
    check("T02", "Brier score = 0.10 for known 2-pair case",
          close(m["brier_score"], 0.10, tol=1e-9),
          f"got {m['brier_score']!r}")


# ---------------------------------------------------------------------------
# T03: negative log-likelihood — hand-computable with 2 pairs
# ---------------------------------------------------------------------------
#   pair1: conf=0.8, y=1 -> -log(0.8) = 0.22314…
#   pair2: conf=0.4, y=0 -> -log(1-0.4)= -log(0.6) = 0.51082…
#   nll = mean = 0.36698…
def test_nll():
    rows = [
        _row("detected", pd=0.8),
        _row("missed",   pd=0.4),
    ]
    expected_nll = round((-math.log(0.8) + -math.log(0.6)) / 2.0, 6)
    m = confidence_calibration_metrics(rows, num_bins=1)
    check("T03", "NLL matches hand computation",
          close(m["negative_log_likelihood"], expected_nll, tol=1e-9),
          f"got {m['negative_log_likelihood']!r}, expected {expected_nll!r}")


# ---------------------------------------------------------------------------
# T04: ECE / MCE in a symmetric 2-bin case
# ---------------------------------------------------------------------------
#  10 pairs at conf=0.05 (bin 0, acc=0.0) -> gap 0.05
#  10 pairs at conf=0.95 (bin 9, acc=1.0) -> gap 0.05
#  ECE = weighted average of |gap| by fraction of samples = 0.05
#  MCE = max(|gap|) = 0.05
def test_ece_mce_symmetric():
    n = 10
    rows = (
        [_row("missed",   pd=0.05)] * n +   # bin 0: all wrong -> acc=0
        [_row("detected", pd=0.95)] * n      # bin 9: all right -> acc=1
    )
    m = confidence_calibration_metrics(rows, num_bins=10)
    # bin 0: conf~=0.05, acc=0  -> gap=0.05
    # bin 9: conf~=0.95, acc=1  -> gap=0.05
    check("T04", "ECE = 0.05 for symmetric 2-bin case",
          close(m["expected_calibration_error"], 0.05, tol=1e-9),
          f"got {m['expected_calibration_error']!r}")
    check("T04", "MCE = 0.05 for symmetric 2-bin case",
          close(m["maximum_calibration_error"], 0.05, tol=1e-9),
          f"got {m['maximum_calibration_error']!r}")


# ---------------------------------------------------------------------------
# T05: perfectly calibrated -> ECE ~= 0, Brier score = p(1-p) in expectation
# ---------------------------------------------------------------------------
#   pairs all at conf=0.5: 50% detected, 50% missed
#   ECE -> 0 (conf == acc in that bin)
#   Brier = (0.5-1)^2*50% + (0.5-0)^2*50% = 0.25
def test_perfect_calibration():
    n = 100
    rows = (
        [_row("detected", pd=0.5)] * (n // 2) +
        [_row("missed",   pd=0.5)] * (n // 2)
    )
    m = confidence_calibration_metrics(rows, num_bins=10)
    check("T05", "ECE ~= 0 for perfectly-calibrated bin",
          close(m["expected_calibration_error"], 0.0, tol=1e-9),
          f"got {m['expected_calibration_error']!r}")
    check("T05", "Brier score = 0.25 at conf=0.5, 50/50 outcomes",
          close(m["brier_score"], 0.25, tol=1e-9),
          f"got {m['brier_score']!r}")


# ---------------------------------------------------------------------------
# T06: severely overconfident sensor
#   all pairs: conf=0.95, but always missed (y=0)
#   ECE = 0.95, MCE = 0.95, overconfidence_rate = 1.0
# ---------------------------------------------------------------------------
def test_overconfident():
    n = 50
    rows = [_row("missed", pd=0.95)] * n
    m = confidence_calibration_metrics(rows, num_bins=10)
    check("T06", "ECE = 0.95 for maximally overconfident sensor",
          close(m["expected_calibration_error"], 0.95, tol=1e-9),
          f"got {m['expected_calibration_error']!r}")
    check("T06", "overconfidence_rate = 1.0",
          close(m["overconfidence_rate"], 1.0, tol=1e-9),
          f"got {m['overconfidence_rate']!r}")
    check("T06", "underconfidence_rate = 0.0",
          close(m["underconfidence_rate"], 0.0, tol=1e-9),
          f"got {m['underconfidence_rate']!r}")


# ---------------------------------------------------------------------------
# T07: overconfidence / underconfidence rates
#   bin A (conf=0.8, acc=1.0): underconfident (acc > conf)
#   bin B (conf=0.9, acc=0.0): overconfident  (conf > acc)
# ---------------------------------------------------------------------------
def test_over_under_confidence_rates():
    n = 10
    rows = (
        [_row("detected", pd=0.85)] * n +      # bin 8: all detected -> acc=1 > 0.85 -> under
        [_row("missed",   pd=0.95)] * n         # bin 9: all missed  -> acc=0 < 0.95 -> over
    )
    m = confidence_calibration_metrics(rows, num_bins=10)
    # half the samples are overconfident, half are underconfident
    check("T07", "overconfidence_rate = 0.5",
          close(m["overconfidence_rate"], 0.5, tol=1e-9),
          f"got {m['overconfidence_rate']!r}")
    check("T07", "underconfidence_rate = 0.5",
          close(m["underconfidence_rate"], 0.5, tol=1e-9),
          f"got {m['underconfidence_rate']!r}")


# ---------------------------------------------------------------------------
# T08: n_samples reported correctly
# ---------------------------------------------------------------------------
def test_n_samples():
    rows = [
        _row("detected", pd=0.8),
        _row("missed",   pd=0.6),
        _row("detected", pd=0.9, false_alarm=True),  # excluded
        _row("missed",   pd=0.7, dropout=True),       # excluded
    ]
    m = confidence_calibration_metrics(rows, num_bins=10)
    check("T08", "n_samples = 2 (2 filtered-in pairs)",
          m["n_samples"] == 2, f"got {m['n_samples']!r}")


# ---------------------------------------------------------------------------
# T09: empty rows -> all-None, n_samples=0
# ---------------------------------------------------------------------------
def test_empty_rows():
    m = confidence_calibration_metrics([], num_bins=10)
    check("T09", "n_samples = 0 for empty input",
          m["n_samples"] == 0)
    check("T09", "ECE is None for empty input",
          m["expected_calibration_error"] is None)
    check("T09", "Brier score is None for empty input",
          m["brier_score"] is None)
    check("T09", "reliability_bins is empty list",
          m["reliability_bins"] == [])


# ---------------------------------------------------------------------------
# T10: radar_like_model.calibration_pairs — confidence_correct filtering
# ---------------------------------------------------------------------------
#   True  -> (confidence_score, True)  included
#   False -> (confidence_score, False) included
#   None  -> excluded (missed detection / dropout, no confidence issued)
def test_radar_calibration_pairs():
    rows = [
        _row("detected", confidence_score=0.9, confidence_correct=True),
        _row("detected", confidence_score=0.4, confidence_correct=False),
        _row("missed",   confidence_score=None, confidence_correct=None),  # excluded
        _row("detected", confidence_score=0.7, confidence_correct=None),  # excluded
    ]
    pairs = radar_calibration_pairs(rows)
    check("T10", "2 pairs extracted (None rows excluded)",
          len(pairs) == 2, f"got {len(pairs)}")
    if len(pairs) == 2:
        check("T10", "first pair: (0.9, True)",
              pairs[0] == (0.9, True))
        check("T10", "second pair: (0.4, False)",
              pairs[1] == (0.4, False))


# ---------------------------------------------------------------------------
# T11: _reliability_bins — boundary assignment
#   With num_bins=10: conf=1.0 must land in bin 9, not an IndexError
#   (the min(..., num_bins-1) guard in _reliability_bins)
# ---------------------------------------------------------------------------
def test_reliability_bin_boundary():
    pairs = [(1.0, True), (0.0, False), (0.99999, True)]
    bins = _reliability_bins(pairs, num_bins=10)
    check("T11", "conf=1.0 lands in last bin (no IndexError)",
          len(bins[9]) >= 1, f"bin9 has {len(bins[9])} entries")
    check("T11", "conf=0.0 lands in first bin",
          len(bins[0]) >= 1, f"bin0 has {len(bins[0])} entries")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def main():
    test_calibration_pair_filtering()
    test_brier_score()
    test_nll()
    test_ece_mce_symmetric()
    test_perfect_calibration()
    test_overconfident()
    test_over_under_confidence_rates()
    test_n_samples()
    test_empty_rows()
    test_radar_calibration_pairs()
    test_reliability_bin_boundary()

    _c.print_summary()
    _c.write_markdown(
        "results/calibration_validation_results.md",
        "Calibration Validation Results",
        "Deterministic checks for confidence-calibration math "
        "(metrics_analysis.confidence_calibration_metrics and "
        "radar_like_model.calibration_pairs).",
    )
    _, _, _ = _c.summary()
    total, passed, _ = _c.summary()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
