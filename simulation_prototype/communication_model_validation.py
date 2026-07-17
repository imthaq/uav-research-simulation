"""
communication_model_validation.py

Task 6: validates communication_model.py's CommunicationChannel against
controlled cases:

  - zero delay
  - fixed delay
  - random delay
  - zero packet loss
  - low packet loss
  - high packet loss
  - limited range
  - temporary outage
  - stale-message rejection
  - corrupted confidence value

Each check either asserts an exact/deterministic outcome or, for
probabilistic behavior (packet loss, corruption), runs many trials with a
seeded RNG and checks the observed rate against the configured probability
within a statistical tolerance. Results are printed and written to
results/communication_model_validation_results.json.

Usage:
    python communication_model_validation.py
"""

import json
import os
import random
import statistics
import sys

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from models.communication_model import CommunicationChannel, PRESETS, from_config

RESULTS = []


def check(task, description, condition, detail=""):
    RESULTS.append({
        "task": task,
        "description": description,
        "passed": bool(condition),
        "detail": detail,
    })
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {task}: {description}" + (f" ({detail})" if detail else ""))


MSG = {"confidence": 0.9, "reliability": 0.8}


# ---------------------------------------------------------------------
# 1. Zero delay
# ---------------------------------------------------------------------
def test_zero_delay():
    ch = CommunicationChannel(base_latency_steps=0, rng=random.Random(1))
    out, outcome = ch.transmit(dict(MSG))
    check("zero_delay", "message with no base latency delivers with latency_steps=0",
          outcome == "delivered" and out["latency_steps"] == 0, f"outcome={outcome} latency={out.get('latency_steps')}")

    # A message that already carries its own upstream latency (e.g. sensor
    # latency) should pass through unchanged when the channel adds 0.
    out2, _ = CommunicationChannel(base_latency_steps=0, rng=random.Random(1)).transmit(
        {**MSG, "latency_steps": 4})
    check("zero_delay", "zero channel delay leaves a message's pre-existing latency untouched",
          out2["latency_steps"] == 4, f"got {out2['latency_steps']}")


# ---------------------------------------------------------------------
# 2. Fixed delay
# ---------------------------------------------------------------------
def test_fixed_delay():
    ch = CommunicationChannel(base_latency_steps=5, rng=random.Random(2))
    latencies = []
    for _ in range(200):
        out, outcome = ch.transmit(dict(MSG))
        assert outcome == "delivered"
        latencies.append(out["latency_steps"])
    check("fixed_delay", "base_latency_steps=5 adds exactly 5 to every delivered message, every time",
          all(L == 5 for L in latencies), f"distinct values seen={sorted(set(latencies))}")

    # Additive on top of a message's own pre-existing latency.
    out, _ = CommunicationChannel(base_latency_steps=5, rng=random.Random(2)).transmit(
        {**MSG, "latency_steps": 2})
    check("fixed_delay", "fixed delay adds to (not replaces) a message's existing latency_steps (2+5=7)",
          out["latency_steps"] == 7, f"got {out['latency_steps']}")


# ---------------------------------------------------------------------
# 3. Random delay
# ---------------------------------------------------------------------
def test_random_delay():
    # The channel itself injects no jitter into latency - base_latency_steps
    # is a fixed additive constant, not a distribution. "Random" delay in
    # this pipeline comes from upstream (e.g. varying sensor latency
    # already stamped on the message before it reaches this channel); the
    # channel's job is to compound its own fixed delay on top of whatever
    # variable latency the message already carries, correctly and without
    # adding variance of its own.
    rng = random.Random(3)
    ch = CommunicationChannel(base_latency_steps=4, rng=random.Random(42))
    deltas = []
    totals = []
    for _ in range(500):
        incoming_latency = rng.randint(0, 10)
        out, outcome = ch.transmit({**MSG, "latency_steps": incoming_latency})
        assert outcome == "delivered"
        totals.append(out["latency_steps"])
        deltas.append(out["latency_steps"] - incoming_latency)

    check("random_delay", "total delivered latency = random incoming latency + fixed base_latency_steps, every trial",
          all(d == 4 for d in deltas), f"distinct deltas={sorted(set(deltas))}")
    check("random_delay", "the channel's own contribution to delay has zero variance (no injected jitter)",
          statistics.pvariance(deltas) == 0.0, f"variance={statistics.pvariance(deltas)}")
    check("random_delay", "total latency varies across trials because incoming latency varies (not constant)",
          len(set(totals)) > 1, f"distinct totals seen={len(set(totals))}")


# ---------------------------------------------------------------------
# 4. Zero packet loss
# ---------------------------------------------------------------------
def test_zero_packet_loss():
    ch = CommunicationChannel(packet_loss_probability=0.0, rng=random.Random(4))
    outcomes = [ch.transmit(dict(MSG))[1] for _ in range(1000)]
    check("zero_packet_loss", "packet_loss_probability=0.0 delivers 100% of 1000 messages",
          all(o == "delivered" for o in outcomes), f"delivered={outcomes.count('delivered')}/1000")


# ---------------------------------------------------------------------
# 5. Low packet loss
# ---------------------------------------------------------------------
def test_low_packet_loss():
    ch = CommunicationChannel(packet_loss_probability=0.05, rng=random.Random(5))
    n = 5000
    outcomes = [ch.transmit(dict(MSG))[1] for _ in range(n)]
    loss_rate = outcomes.count("packet_loss") / n
    check("low_packet_loss", "packet_loss_probability=0.05 gives an observed loss rate near 5% over 5000 trials",
          abs(loss_rate - 0.05) < 0.02, f"observed loss rate={loss_rate:.4f}")
    check("low_packet_loss", "every non-lost message is marked delivered (no third outcome for a range-unlimited, non-stale channel)",
          set(outcomes) <= {"delivered", "packet_loss"}, f"outcomes seen={set(outcomes)}")

    preset_ch = CommunicationChannel(rng=random.Random(5), **PRESETS["low_packet_loss"])
    preset_outcomes = [preset_ch.transmit(dict(MSG))[1] for _ in range(n)]
    preset_loss_rate = preset_outcomes.count("packet_loss") / n
    check("low_packet_loss", "'low_packet_loss' preset (0.05) matches its documented loss rate",
          abs(preset_loss_rate - 0.05) < 0.02, f"observed={preset_loss_rate:.4f}")


# ---------------------------------------------------------------------
# 6. High packet loss
# ---------------------------------------------------------------------
def test_high_packet_loss():
    ch = CommunicationChannel(packet_loss_probability=0.4, rng=random.Random(6))
    n = 5000
    outcomes = [ch.transmit(dict(MSG))[1] for _ in range(n)]
    loss_rate = outcomes.count("packet_loss") / n
    check("high_packet_loss", "packet_loss_probability=0.4 gives an observed loss rate near 40% over 5000 trials",
          abs(loss_rate - 0.4) < 0.03, f"observed loss rate={loss_rate:.4f}")

    preset_ch = CommunicationChannel(rng=random.Random(6), **PRESETS["high_packet_loss"])
    preset_outcomes = [preset_ch.transmit(dict(MSG))[1] for _ in range(n)]
    preset_loss_rate = preset_outcomes.count("packet_loss") / n
    check("high_packet_loss", "'high_packet_loss' preset (0.4) matches its documented loss rate",
          abs(preset_loss_rate - 0.4) < 0.03, f"observed={preset_loss_rate:.4f}")


# ---------------------------------------------------------------------
# 7. Limited range
# ---------------------------------------------------------------------
def test_limited_range():
    ch = CommunicationChannel(comm_range=5.0, rng=random.Random(7))

    out, outcome = ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(3.0, 0.0))
    check("limited_range", "receiver within comm_range (distance 3 <= 5) is delivered",
          outcome == "delivered", f"outcome={outcome}")

    out, outcome = ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(10.0, 0.0))
    check("limited_range", "receiver beyond comm_range (distance 10 > 5) is rejected as out_of_range",
          outcome == "out_of_range" and out is None, f"outcome={outcome}")

    # Boundary: exactly at comm_range should still be in range (<=).
    out, outcome = ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(5.0, 0.0))
    check("limited_range", "receiver exactly at comm_range (distance == 5) is delivered (<=, not <)",
          outcome == "delivered", f"outcome={outcome}")

    # comm_range=None means unlimited: an arbitrarily far receiver still delivers.
    unlimited_ch = CommunicationChannel(comm_range=None, rng=random.Random(7))
    out, outcome = unlimited_ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(1e6, 0.0))
    check("limited_range", "comm_range=None imposes no range limit at all",
          outcome == "delivered", f"outcome={outcome}")

    # Missing position info can't be range-gated, so it should pass through.
    out, outcome = ch.transmit(dict(MSG), sender_pos=None, receiver_pos=(10.0, 0.0))
    check("limited_range", "missing sender_pos skips range gating (can't gate what isn't known)",
          outcome == "delivered", f"outcome={outcome}")

    preset_ch = CommunicationChannel(rng=random.Random(7), **PRESETS["short_range"])
    out, outcome = preset_ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(4.9, 0.0))
    check("limited_range", "'short_range' preset (comm_range=5.0) delivers just inside its range",
          outcome == "delivered", f"outcome={outcome}")
    out, outcome = preset_ch.transmit(dict(MSG), sender_pos=(0.0, 0.0), receiver_pos=(5.1, 0.0))
    check("limited_range", "'short_range' preset (comm_range=5.0) rejects just outside its range",
          outcome == "out_of_range", f"outcome={outcome}")


# ---------------------------------------------------------------------
# 8. Temporary outage
# ---------------------------------------------------------------------
def test_temporary_outage():
    outage_ch = CommunicationChannel(rng=random.Random(8), **PRESETS["outage"])
    outcomes = [outage_ch.transmit(dict(MSG))[1] for _ in range(50)]
    check("temporary_outage", "'outage' preset (packet_loss_probability=1.0) drops every message while active",
          all(o == "packet_loss" for o in outcomes), f"outcomes seen={set(outcomes)}")

    # Simulate the outage being temporary: the same channel object's
    # packet_loss_probability is restored to 0 afterward (as a scenario
    # script would do when connectivity comes back), and delivery resumes
    # immediately with no lingering effect from the outage period.
    outage_ch.packet_loss_probability = 0.0
    recovered_outcomes = [outage_ch.transmit(dict(MSG))[1] for _ in range(50)]
    check("temporary_outage", "communication recovers fully once the outage condition is lifted",
          all(o == "delivered" for o in recovered_outcomes), f"outcomes seen={set(recovered_outcomes)}")

    # A from_config channel built with the "outage" preset behaves identically.
    cfg_ch = from_config({"preset": "outage"}, rng=random.Random(8))
    check("temporary_outage", "from_config({'preset': 'outage'}) reproduces the outage preset's total loss",
          cfg_ch.transmit(dict(MSG))[1] == "packet_loss")


# ---------------------------------------------------------------------
# 9. Stale-message rejection
# ---------------------------------------------------------------------
def test_stale_message_rejection():
    ch = CommunicationChannel(max_staleness_steps=3, rng=random.Random(9))

    out, outcome = ch.transmit(dict(MSG), measurement_age_steps=1)
    check("stale_message_rejection", "message younger than max_staleness_steps is delivered",
          outcome == "delivered", f"outcome={outcome} age=1 max=3")

    out, outcome = ch.transmit(dict(MSG), measurement_age_steps=3)
    check("stale_message_rejection", "message exactly at max_staleness_steps is NOT stale (age>max, not >=)",
          outcome == "delivered", f"outcome={outcome} age=3 max=3")

    out, outcome = ch.transmit(dict(MSG), measurement_age_steps=4)
    check("stale_message_rejection", "message one step past max_staleness_steps is rejected as stale",
          outcome == "stale" and out is None, f"outcome={outcome} age=4 max=3")

    out, outcome = ch.transmit(dict(MSG), measurement_age_steps=1000)
    check("stale_message_rejection", "a very old message is rejected as stale",
          outcome == "stale", f"outcome={outcome} age=1000 max=3")

    # max_staleness_steps=None disables staleness rejection entirely.
    no_staleness_ch = CommunicationChannel(max_staleness_steps=None, rng=random.Random(9))
    out, outcome = no_staleness_ch.transmit(dict(MSG), measurement_age_steps=10_000)
    check("stale_message_rejection", "max_staleness_steps=None never rejects for staleness, however old",
          outcome == "delivered", f"outcome={outcome}")

    # Staleness is checked ahead of packet loss (per transmit()'s ordering),
    # so a guaranteed-loss channel still reports "stale" for a stale message.
    always_loss_ch = CommunicationChannel(packet_loss_probability=1.0, max_staleness_steps=3,
                                           rng=random.Random(9))
    out, outcome = always_loss_ch.transmit(dict(MSG), measurement_age_steps=100)
    check("stale_message_rejection", "staleness is checked before the packet-loss roll (reports 'stale', not 'packet_loss')",
          outcome == "stale", f"outcome={outcome}")


# ---------------------------------------------------------------------
# 10. Corrupted confidence value
# ---------------------------------------------------------------------
def test_corrupted_confidence_value():
    # corruption_probability=0.0: confidence/reliability always pass through unchanged.
    clean_ch = CommunicationChannel(corruption_probability=0.0, rng=random.Random(10))
    results = [clean_ch.transmit({"confidence": 0.9, "reliability": 0.8}) for _ in range(200)]
    check("corrupted_confidence_value", "corruption_probability=0.0 never corrupts confidence/reliability",
          all(out["confidence"] == 0.9 and out["reliability"] == 0.8 and out["corrupted"] is False
              for out, _ in results))

    # corruption_probability=1.0: every delivered message is corrupted, and
    # the corrupted confidence equals original*factor clamped to [0, 1] for
    # some factor drawn from [0.2, 1.8].
    dirty_ch = CommunicationChannel(corruption_probability=1.0, rng=random.Random(11))
    n = 2000
    confidences, reliabilities, factors = [], [], []
    for _ in range(n):
        out, outcome = dirty_ch.transmit({"confidence": 0.5, "reliability": 0.5})
        assert outcome == "delivered"
        check_flag = out["corrupted"] is True
        if not check_flag:
            break
        confidences.append(out["confidence"])
        reliabilities.append(out["reliability"])
        # confidence == reliability == 0.5*factor here (both scaled by the
        # same draw), so back out the implied factor for range-checking.
        factors.append(out["confidence"] / 0.5)

    check("corrupted_confidence_value", "corruption_probability=1.0 marks every delivered message corrupted=True",
          len(confidences) == n)
    check("corrupted_confidence_value", "corrupted confidence values stay within the valid [0, 1] probability range",
          all(0.0 <= c <= 1.0 for c in confidences), f"min={min(confidences):.3f} max={max(confidences):.3f}")
    check("corrupted_confidence_value", "corrupted reliability values stay within the valid [0, 1] range",
          all(0.0 <= r <= 1.0 for r in reliabilities), f"min={min(reliabilities):.3f} max={max(reliabilities):.3f}")
    check("corrupted_confidence_value", "confidence and reliability are scaled by the same random factor per message",
          all(close_enough(c, r) for c, r in zip(confidences, reliabilities)),
          "confidence == reliability for every trial since both started at 0.5")
    check("corrupted_confidence_value", "implied corruption factor is drawn from [0.2, 1.8] (before clamping)",
          min(factors) >= 0.2 - 1e-9 and max(factors) <= 1.8 + 1e-9,
          f"observed factor range=[{min(factors):.3f}, {max(factors):.3f}]")
    check("corrupted_confidence_value", "corruption actually changes the value for the large majority of trials (factor != 1)",
          sum(1 for f in factors if abs(f - 1.0) > 1e-9) / n > 0.95,
          f"fraction changed={sum(1 for f in factors if abs(f - 1.0) > 1e-9) / n:.3f}")

    # A message missing confidence/reliability entirely shouldn't error, and
    # shouldn't fabricate those fields.
    bare_ch = CommunicationChannel(corruption_probability=1.0, rng=random.Random(12))
    out, outcome = bare_ch.transmit({"track_id": "t1"})
    check("corrupted_confidence_value", "a message with no confidence/reliability fields is delivered without error and gains no fabricated fields",
          outcome == "delivered" and "confidence" not in out and "reliability" not in out and out["corrupted"] is True)

    # A confidence of exactly 0.0 stays clamped at/above 0 even if the
    # factor would (mathematically) keep it at 0 - sanity check the clamp
    # doesn't push it negative or leave it undefined.
    zero_ch = CommunicationChannel(corruption_probability=1.0, rng=random.Random(13))
    out, _ = zero_ch.transmit({"confidence": 0.0, "reliability": 0.0})
    check("corrupted_confidence_value", "corrupting a confidence of exactly 0.0 stays exactly 0.0 (0*factor=0, already in range)",
          out["confidence"] == 0.0 and out["reliability"] == 0.0)


def close_enough(a, b, tol=1e-9):
    return abs(a - b) <= tol


def main():
    test_zero_delay()
    test_fixed_delay()
    test_random_delay()
    test_zero_packet_loss()
    test_low_packet_loss()
    test_high_packet_loss()
    test_limited_range()
    test_temporary_outage()
    test_stale_message_rejection()
    test_corrupted_confidence_value()

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
    out_path = os.path.join(out_dir, "communication_model_validation_results.json")
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
