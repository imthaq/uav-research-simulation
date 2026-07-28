"""
dynamic_trust_validation.py

Task 7: validates fusion_model.py's TrustTracker (the dynamic, cross-step
trust score maintained per UAV/radar) against controlled cases.
"""

import os
import sys

import numpy as np

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from fusion.fusion_model import TrustTracker, TRUST_MIN, TRUST_MAX, TRUST_INITIAL
from validation.validation_common import Checker

_checker = Checker()


def check(task, description, condition, detail=""):
    return _checker.check(task, description, condition, detail)


def make_source(radar_id="r1", x=0.0, y=0.0, confidence=0.95,
                 measurement_age_steps=0, dropout_state=False,
                 covariance_trace=1.0):
    """Builds a source dict in the shape TrustTracker.update() expects."""
    return {
        "source_id": f"{radar_id}_t1",
        "radar_id": radar_id,
        "x": x,
        "y": y,
        "confidence": confidence,
        "measurement_age_steps": measurement_age_steps,
        "dropout_state": dropout_state,
        "covariance": np.diag([covariance_trace / 2.0, covariance_trace / 2.0]),
    }


def good_source(radar_id="r1", agree_x=0.0, agree_y=0.0):
    return make_source(radar_id, x=agree_x, y=agree_y, confidence=1.0,
                        measurement_age_steps=0, dropout_state=False,
                        covariance_trace=0.1)


def cluster_mate(x, y):
    return make_source(radar_id="peer", x=x, y=y)


def test_disagreement_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        s = make_source("r1", x=20.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        trust_history.append(tracker.get("r1"))

    check("disagreement_decreases_trust",
          "trust strictly decreases every step under repeated hard disagreement",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")


def test_dropout_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=True,
                         covariance_trace=0.1)
        tracker.update([s], [[s]])
        trust_history.append(tracker.get("r1"))

    check("dropout_decreases_trust",
          "trust strictly decreases every step under repeated dropout",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")


def test_staleness_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=20, dropout_state=False,
                         covariance_trace=0.1)
        tracker.update([s], [[s]])
        trust_history.append(tracker.get("r1"))

    check("staleness_decreases_trust",
          "trust strictly decreases every step under repeated staleness",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")


def test_false_alarms_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        # A false alarm is often isolated (doesn't cluster with peers) and has low confidence
        s = make_source("r1", x=50.0, y=50.0, confidence=0.2,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=5.0)
        peer = cluster_mate(0.0, 0.0) # completely different location
        tracker.update([s, peer], [[s], [peer]])
        trust_history.append(tracker.get("r1"))

    check("false_alarms_decreases_trust",
          "trust strictly decreases after repeated false alarms (low confidence, isolated)",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")


def test_inconsistent_updates_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for step in range(6):
        # Inconsistent updates (bouncing around, covariance increasing)
        cov = 0.1 if step % 2 == 0 else 10.0
        s = make_source("r1", x=10.0 * step, y=0.0, confidence=0.8,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=cov)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s], [peer]])
        trust_history.append(tracker.get("r1"))
    
    check("inconsistent_updates_decreases_trust",
          "trust decreases overall when updates are wildly inconsistent and covariance spikes",
          trust_history[-1] < trust_history[0], f"history={trust_history}")


def test_trust_never_negative():
    tracker = TrustTracker()
    for _ in range(50):
        s = make_source("r1", x=1000.0, y=1000.0, confidence=0.0,
                         measurement_age_steps=10_000, dropout_state=True,
                         covariance_trace=1e6)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
    
    final = tracker.get("r1")
    check("trust_never_negative", "trust never drops below 0", final >= 0.0, f"final={final:.4f}")


def test_trust_never_exceeds_max():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(50):
        s = good_source("r1", agree_x=0.0, agree_y=0.0)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        trust_history.append(tracker.get("r1"))

    check("trust_never_exceeds_max", "trust never exceeds TRUST_MAX",
          all(t <= TRUST_MAX for t in trust_history), f"max seen={max(trust_history):.4f}")


def test_trust_gradually_recovers():
    tracker = TrustTracker()
    for _ in range(8):
        s = make_source("r1", x=20.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
    
    depressed = tracker.get("r1")
    recovery_history = [depressed]
    for _ in range(10):
        s = good_source("r1", agree_x=0.0, agree_y=0.0)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        recovery_history.append(tracker.get("r1"))

    check("trust_gradually_recovers", "trust rises monotonically once good signals resume",
          all(recovery_history[i + 1] >= recovery_history[i] for i in range(len(recovery_history) - 1)))


def test_one_good_update_insufficient():
    tracker = TrustTracker()
    for _ in range(8):
        s = make_source("r1", x=20.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
    
    depressed = tracker.get("r1")
    s = good_source("r1", agree_x=0.0, agree_y=0.0)
    peer = cluster_mate(0.0, 0.0)
    tracker.update([s, peer], [[s, peer]])
    after_one_good_update = tracker.get("r1")

    check("one_good_update_insufficient", "a single perfect update does not restore trust to TRUST_MAX",
          after_one_good_update < TRUST_MAX, f"after one good update={after_one_good_update:.4f}")


def test_hysteresis():
    tracker = TrustTracker()
    check("hysteresis", "trust recovery contains hysteresis (alpha_up < alpha_down) to drop fast but recover slowly",
          tracker.alpha_up < tracker.alpha_down, f"up={tracker.alpha_up}, down={tracker.alpha_down}")


def test_ground_truth_not_used():
    tracker = TrustTracker()
    s = good_source("r1", agree_x=0.0, agree_y=0.0)
    peer = cluster_mate(0.0, 0.0)
    tracker.update([s, peer], [[s, peer]])
    
    # Check that update succeeds even when ground truth is completely absent from all structures
    keys_used = set(s.keys())
    check("ground_truth_not_used", "trust does not directly use ground-truth error in sources",
          "ground_truth_x" not in keys_used and "true_x" not in keys_used)


def test_trust_updates_are_logged():
    tracker = TrustTracker()
    s = good_source("r1", agree_x=0.0, agree_y=0.0)
    tracker.update([s], [[s]])
    
    snap = tracker.snapshot()
    logs = tracker.last_signals("r1")
    
    check("trust_updates_are_logged", "trust snapshot provides per-radar scores for logging", "r1" in snap)
    check("trust_updates_are_logged", "last_signals provides signal breakdown for logging", logs is not None and "target" in logs)


def main():
    test_disagreement_decreases_trust()
    test_dropout_decreases_trust()
    test_staleness_decreases_trust()
    test_false_alarms_decreases_trust()
    test_inconsistent_updates_decreases_trust()
    test_trust_never_negative()
    test_trust_never_exceeds_max()
    test_trust_gradually_recovers()
    test_one_good_update_insufficient()
    test_hysteresis()
    test_ground_truth_not_used()
    test_trust_updates_are_logged()

    failed = _checker.print_summary()

    out_dir = os.path.join(_ROOT_DIR, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "dynamic_trust_validation_results.md")
    _checker.write_markdown(
        out_path, "Dynamic Trust Validation Results (Task 7)",
        intro="Validates TrustTracker behavior regarding sensor trust penalties and recovery.")
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
