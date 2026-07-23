"""
dynamic_trust_validation.py

Task 7: validates fusion_model.py's TrustTracker (the dynamic, cross-step
trust score maintained per UAV/radar) against controlled cases:

  - trust decreases after repeated disagreement
  - trust decreases after dropout
  - trust decreases for stale data
  - trust does not become negative
  - trust does not exceed maximum
  - trust gradually recovers
  - one correct update does not immediately restore full trust

TrustTracker.update(sources, clusters) is called directly with small,
hand-built "source" dicts (the same shape _as_source produces), so each
case can isolate exactly one signal (agreement, freshness, dropout,
confidence, covariance) while holding the others at their best value.

Usage:
    python dynamic_trust_validation.py
"""

import json
import os
import sys

import numpy as np

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from fusion.fusion_model import TrustTracker, TRUST_MIN, TRUST_MAX, TRUST_INITIAL

RESULTS = []


def check(task, description, condition, detail=""):
    RESULTS.append({"task": task, "description": description, "passed": bool(condition), "detail": detail})
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {task}: {description}" + (f" ({detail})" if detail else ""))


def make_source(radar_id="r1", x=0.0, y=0.0, confidence=0.95,
                 measurement_age_steps=0, dropout_state=False,
                 covariance_trace=1.0):
    """Builds a source dict in the shape TrustTracker.update() expects
    (same fields _as_source produces)."""
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
    """A source with the best possible value on every signal: perfect
    agreement with a cluster-mate, zero age, no dropout, full confidence,
    tight covariance."""
    return make_source(radar_id, x=agree_x, y=agree_y, confidence=1.0,
                        measurement_age_steps=0, dropout_state=False,
                        covariance_trace=0.1)


def cluster_mate(x, y):
    """A second, unrelated source placed exactly at (x, y) so a test
    source at the same position gets a perfect agreement score."""
    return make_source(radar_id="peer", x=x, y=y)


# ---------------------------------------------------------------------
# 1. Trust decreases after repeated disagreement
# ---------------------------------------------------------------------
def test_disagreement_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        # Source sits far (residual 20 >> hard distance 5) from its
        # cluster-mate every step, but is otherwise perfect (fresh, no
        # dropout, full confidence, tight covariance) - isolates agreement.
        s = make_source("r1", x=20.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        peer = cluster_mate(0.0, 0.0)
        cluster = [s, peer]
        tracker.update([s, peer], [cluster])
        trust_history.append(tracker.get("r1"))

    check("disagreement_decreases_trust",
          "trust strictly decreases every step under repeated hard disagreement",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")
    check("disagreement_decreases_trust",
          "trust after repeated disagreement drops well below the initial value",
          trust_history[-1] < TRUST_INITIAL - 0.15, f"final={trust_history[-1]:.4f}")


# ---------------------------------------------------------------------
# 2. Trust decreases after dropout
# ---------------------------------------------------------------------
def test_dropout_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        # dropout_state=True every step, otherwise perfect and no peers to
        # disagree with (single source this step -> neutral agreement).
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=True,
                         covariance_trace=0.1)
        tracker.update([s], [[s]])
        trust_history.append(tracker.get("r1"))

    check("dropout_decreases_trust",
          "trust strictly decreases every step under repeated dropout",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")

    # Compare against a twin tracker seeing dropout_state=False every step,
    # otherwise identical - isolates dropout's effect from the neutral
    # single-source agreement score both share.
    tracker_no_dropout = TrustTracker()
    for _ in range(6):
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        tracker_no_dropout.update([s], [[s]])
    check("dropout_decreases_trust",
          "repeated dropout leaves trust lower than an otherwise-identical no-dropout run",
          trust_history[-1] < tracker_no_dropout.get("r1"),
          f"dropout={trust_history[-1]:.4f} vs no_dropout={tracker_no_dropout.get('r1'):.4f}")


# ---------------------------------------------------------------------
# 3. Trust decreases for stale data
# ---------------------------------------------------------------------
def test_staleness_decreases_trust():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(6):
        # Large measurement_age_steps every step (stale), otherwise
        # perfect and no peers to disagree with.
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=20, dropout_state=False,
                         covariance_trace=0.1)
        tracker.update([s], [[s]])
        trust_history.append(tracker.get("r1"))

    check("staleness_decreases_trust",
          "trust strictly decreases every step under repeated staleness",
          all(trust_history[i + 1] < trust_history[i] for i in range(len(trust_history) - 1)),
          f"history={trust_history}")

    tracker_fresh = TrustTracker()
    for _ in range(6):
        s = make_source("r1", x=0.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        tracker_fresh.update([s], [[s]])
    check("staleness_decreases_trust",
          "repeated staleness leaves trust lower than an otherwise-identical fresh-data run",
          trust_history[-1] < tracker_fresh.get("r1"),
          f"stale={trust_history[-1]:.4f} vs fresh={tracker_fresh.get('r1'):.4f}")


# ---------------------------------------------------------------------
# 4. Trust does not become negative
# ---------------------------------------------------------------------
def test_trust_never_negative():
    tracker = TrustTracker()
    for _ in range(200):
        # Worst possible signal on every axis at once: hard disagreement,
        # max staleness, dropout, zero confidence, huge covariance.
        s = make_source("r1", x=1000.0, y=1000.0, confidence=0.0,
                         measurement_age_steps=10_000, dropout_state=True,
                         covariance_trace=1e6)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        assert tracker.get("r1") >= 0.0

    final = tracker.get("r1")
    check("trust_never_negative", "trust never drops below 0 even under sustained worst-case signals for 200 steps",
          final >= 0.0, f"final={final:.4f}")
    check("trust_never_negative", "trust floors exactly at TRUST_MIN under sustained worst-case signals",
          abs(final - TRUST_MIN) < 1e-6, f"final={final:.4f}, TRUST_MIN={TRUST_MIN}")


# ---------------------------------------------------------------------
# 5. Trust does not exceed maximum
# ---------------------------------------------------------------------
def test_trust_never_exceeds_max():
    tracker = TrustTracker()
    trust_history = [tracker.get("r1")]
    for _ in range(200):
        s = good_source("r1", agree_x=0.0, agree_y=0.0)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        trust_history.append(tracker.get("r1"))
        assert tracker.get("r1") <= TRUST_MAX

    check("trust_never_exceeds_max", "trust never exceeds TRUST_MAX (1.0) even under sustained best-case signals",
          all(t <= TRUST_MAX for t in trust_history), f"max seen={max(trust_history):.4f}")
    check("trust_never_exceeds_max", "trust starts at TRUST_INITIAL (already at the max) and simply stays there",
          trust_history[0] == TRUST_INITIAL == TRUST_MAX, f"initial={trust_history[0]}")


# ---------------------------------------------------------------------
# 6. Trust gradually recovers
# ---------------------------------------------------------------------
def test_trust_gradually_recovers():
    tracker = TrustTracker()
    # First, drive trust down with sustained disagreement.
    for _ in range(8):
        s = make_source("r1", x=20.0, y=0.0, confidence=1.0,
                         measurement_age_steps=0, dropout_state=False,
                         covariance_trace=0.1)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
    depressed = tracker.get("r1")
    check("trust_gradually_recovers", "trust was successfully driven below its initial value before testing recovery",
          depressed < TRUST_INITIAL - 0.15, f"depressed={depressed:.4f}")

    # Now switch to perfect signals and watch it climb back, step by step.
    recovery_history = [depressed]
    for _ in range(10):
        s = good_source("r1", agree_x=0.0, agree_y=0.0)
        peer = cluster_mate(0.0, 0.0)
        tracker.update([s, peer], [[s, peer]])
        recovery_history.append(tracker.get("r1"))

    check("trust_gradually_recovers", "trust rises monotonically once good signals resume",
          all(recovery_history[i + 1] >= recovery_history[i] for i in range(len(recovery_history) - 1)),
          f"history={[round(v,4) for v in recovery_history]}")
    check("trust_gradually_recovers", "trust is still below max after 10 good steps (recovery is gradual, not instant)",
          recovery_history[-1] < TRUST_MAX, f"after 10 good steps={recovery_history[-1]:.4f}")
    check("trust_gradually_recovers", "recovery takes multiple steps to make meaningful progress back up (not a 1-step jump)",
          (recovery_history[1] - recovery_history[0]) < (TRUST_MAX - depressed) * 0.5,
          f"first-step gain={recovery_history[1] - recovery_history[0]:.4f}, "
          f"total gap={TRUST_MAX - depressed:.4f}")


# ---------------------------------------------------------------------
# 7. One correct update does not immediately restore full trust
# ---------------------------------------------------------------------
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

    check("one_good_update_insufficient", "a single perfect update after being depressed does not restore trust to TRUST_MAX",
          after_one_good_update < TRUST_MAX, f"after one good update={after_one_good_update:.4f}")
    check("one_good_update_insufficient", "a single perfect update moves trust up, but only by the slow alpha_up step",
          depressed < after_one_good_update < TRUST_MAX,
          f"depressed={depressed:.4f} -> after_one_good_update={after_one_good_update:.4f}")
    expected_gain = tracker.alpha_up * (tracker.last_signals("r1")["target"] - depressed)
    check("one_good_update_insufficient", "the single-step gain matches alpha_up*(target-current) (asymmetric EWMA, slow climb)",
          abs((after_one_good_update - depressed) - expected_gain) < 1e-3,
          f"observed gain={after_one_good_update - depressed:.4f}, expected~={expected_gain:.4f}")


def main():
    test_disagreement_decreases_trust()
    test_dropout_decreases_trust()
    test_staleness_decreases_trust()
    test_trust_never_negative()
    test_trust_never_exceeds_max()
    test_trust_gradually_recovers()
    test_one_good_update_insufficient()

    total = len(RESULTS)
    passed = sum(1 for r in RESULTS if r["passed"])
    failed = total - passed

    by_task = {}
    for r in RESULTS:
        by_task.setdefault(r["task"], []).append(r["passed"])

    print("\n=== Summary by task ===")
    for task, outcomes in by_task.items():
        print(f"  {task}: {sum(outcomes)}/{len(outcomes)} passed")
    print(f"\nTotal: {passed}/{total} checks passed" + (f", {failed} FAILED" if failed else ""))

    out_dir = os.path.join(_ROOT_DIR, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "dynamic_trust_validation_results.json")
    with open(out_path, "w") as f:
        json.dump({
            "total": total, "passed": passed, "failed": failed,
            "by_task": {t: {"passed": sum(o), "total": len(o)} for t, o in by_task.items()},
            "checks": RESULTS,
        }, f, indent=2)
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())