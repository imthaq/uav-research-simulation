import math
import random


class CommunicationChannel:
    """One communication link a track message travels over.

    packet_loss_probability - per-message chance a transmission is lost
        outright (independent of range).
    comm_range              - max distance (world units) a message can
        travel; farther sender/receiver pairs simply can't hear each
        other. None = unlimited range.
    base_latency_steps      - fixed delay (sim steps) every delivered
        message incurs, on top of the sender's own sensor latency.
    max_staleness_steps     - a track message older than this (by its
        measurement_age_steps) is rejected outright as stale, regardless
        of whether it was otherwise delivered - a receiver discarding an
        out-of-date report rather than acting on it. None = never reject
        for staleness.
    corruption_probability  - per-message chance the reported
        confidence/reliability value arrives corrupted (scaled by a random
        factor), modeling bit errors/garbled payloads rather than outright
        loss.
    """

    def __init__(self, packet_loss_probability=0.0, comm_range=None,
                 base_latency_steps=0, max_staleness_steps=None,
                 corruption_probability=0.0, rng=None):
        self.packet_loss_probability = packet_loss_probability
        self.comm_range = comm_range
        self.base_latency_steps = base_latency_steps
        self.max_staleness_steps = max_staleness_steps
        self.corruption_probability = corruption_probability
        self.rng = rng or random.Random()

    def in_range(self, sender_pos, receiver_pos):
        if self.comm_range is None or sender_pos is None or receiver_pos is None:
            return True
        return math.hypot(sender_pos[0] - receiver_pos[0],
                           sender_pos[1] - receiver_pos[1]) <= self.comm_range

    def is_stale(self, measurement_age_steps):
        if self.max_staleness_steps is None:
            return False
        return measurement_age_steps > self.max_staleness_steps

    def transmit(self, message, sender_pos=None, receiver_pos=None,
                 measurement_age_steps=0):
        """Attempts to deliver one track message over this channel.

        Returns (delivered_message_or_None, outcome), outcome one of
        "delivered", "out_of_range", "stale", "packet_loss". A delivered
        message is a shallow copy of `message` with its latency bumped by
        base_latency_steps and, with corruption_probability, its
        confidence/reliability fields scaled by a random corruption
        factor (flagged via "corrupted")."""
        if not self.in_range(sender_pos, receiver_pos):
            return None, "out_of_range"
        if self.is_stale(measurement_age_steps):
            return None, "stale"
        if self.rng.random() < self.packet_loss_probability:
            return None, "packet_loss"

        out = dict(message)
        out["latency_steps"] = out.get("latency_steps", 0) + self.base_latency_steps
        if self.rng.random() < self.corruption_probability:
            factor = self.rng.uniform(0.2, 1.8)
            for key in ("confidence", "reliability"):
                if key in out and out[key] is not None:
                    out[key] = max(0.0, min(1.0, out[key] * factor))
            out["corrupted"] = True
        else:
            out["corrupted"] = False
        return out, "delivered"


# Task 13's six required test conditions, as ready-to-use channel presets.
PRESETS = {
    "perfect": dict(packet_loss_probability=0.0, comm_range=None,
                    base_latency_steps=0, max_staleness_steps=None,
                    corruption_probability=0.0),
    "low_packet_loss": dict(packet_loss_probability=0.05, comm_range=None,
                             base_latency_steps=1, max_staleness_steps=None,
                             corruption_probability=0.02),
    "high_packet_loss": dict(packet_loss_probability=0.4, comm_range=None,
                              base_latency_steps=1, max_staleness_steps=None,
                              corruption_probability=0.1),
    "short_range": dict(packet_loss_probability=0.0, comm_range=5.0,
                         base_latency_steps=1, max_staleness_steps=None,
                         corruption_probability=0.0),
    "delayed_sharing": dict(packet_loss_probability=0.0, comm_range=None,
                             base_latency_steps=5, max_staleness_steps=None,
                             corruption_probability=0.0),
    "outage": dict(packet_loss_probability=1.0, comm_range=None,
                    base_latency_steps=0, max_staleness_steps=None,
                    corruption_probability=0.0),
}


def from_config(comm_cfg, rng=None):
    """Builds a CommunicationChannel from a "communication" config block,
    optionally seeded from one of PRESETS via comm_cfg["preset"] and then
    overridden field-by-field by any keys also present in comm_cfg."""
    comm_cfg = comm_cfg or {}
    params = dict(PRESETS.get(comm_cfg.get("preset"), {}))
    for key in ("packet_loss_probability", "comm_range", "base_latency_steps",
                "max_staleness_steps", "corruption_probability"):
        if key in comm_cfg:
            params[key] = comm_cfg[key]
    return CommunicationChannel(rng=rng, **params)


if __name__ == "__main__":
    # Minimal self-check: each preset should behave as its name promises
    # over many trials (deterministic seed for reproducibility).
    msg = {"confidence": 0.9, "reliability": 0.8}
    for name, params in PRESETS.items():
        ch = CommunicationChannel(rng=random.Random(0), **params)
        delivered = sum(1 for _ in range(500)
                         if ch.transmit(msg, measurement_age_steps=0)[1] == "delivered")
        rate = delivered / 500
        print(f"{name:20s} delivered {rate:.2%} of 500 messages")

    perfect = CommunicationChannel(**PRESETS["perfect"])
    assert perfect.transmit(msg)[1] == "delivered"
    outage = CommunicationChannel(**PRESETS["outage"])
    assert outage.transmit(msg)[1] == "packet_loss"
    short = CommunicationChannel(**PRESETS["short_range"])
    assert short.transmit(msg, sender_pos=(0, 0), receiver_pos=(100, 0))[1] == "out_of_range"
    stale_ch = CommunicationChannel(max_staleness_steps=3)
    assert stale_ch.transmit(msg, measurement_age_steps=10)[1] == "stale"
    print("communication_model self-check passed")
