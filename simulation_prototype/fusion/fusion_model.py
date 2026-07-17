"""
fusion_model.py

Combines each UAV's radar track estimates (from radar_track_model.py) into
a single fused position estimate per real-world object, per time step.

Fusion never touches ground truth. Its only input is radar tracks that
have already been through radar_like_model.py + radar_track_model.py. That
mirrors the same rule simple_swarm_sim.py's existing single-obstacle
fuse_obstacle_detections() already follows - this module generalizes that
idea to per-object track fusion across all of the swarm's radars, instead
of a single hardcoded obstacle id. No fusion mode below ever looks at
ground truth to decide which source to trust or how to weight it; every
weight is derived only from what a real UAV would actually have on hand
at runtime - the track's own confidence, status, covariance, and
staleness, plus static, config-known sensor characteristics (latency,
dropout probability). Ground truth is only ever used afterwards, by
metrics_analysis.py / simulation_visualizer.py, to *evaluate* what fusion
produced - never to steer fusion itself.

Before fusing, we first have to know which tracks - one per UAV's radar -
refer to the *same* real-world object. That's done with the same
nearest-neighbor idea radar_track_model.py already uses for
detection-to-track association, one level up: track-to-track.

Fusion architectures
--------------------
Independent of *which weighting scheme* combines tracks (the fusion modes
below), there are two different answers to *where* that combining happens
and *how the tracks get there* - this is the "architecture" axis:

  - "centralized" - every UAV's track is sent to one central fusion node
                    (e.g. a ground station or a designated lead UAV). That
                    node runs the clustering + fuse_group math once and
                    produces a single final world estimate per object,
                    which is then broadcast back out to the swarm. One
                    shared answer, but it costs an uplink message per UAV
                    plus a downlink broadcast, and nothing is usable until
                    that round trip completes (see fuse_centralized).
                    This is what fuse_step already did before this
                    architecture axis existed, so "centralized" is the
                    default and reproduces the old behavior exactly.
  - "distributed"  - there is no central node. Each UAV keeps its own
                      local track(s), broadcasts a lightweight summary of
                      them to the rest of the swarm, and separately
                      receives whatever summaries the others managed to
                      get to it this step (each peer-to-peer broadcast can
                      independently fail - see COMM_DROP_PROBABILITY).
                      Each UAV then runs its *own* local clustering +
                      fuse_group pass over only what it currently has on
                      hand (its own track plus whatever peer summaries
                      arrived). Because delivery isn't guaranteed to be
                      identical for every UAV, different UAVs can end up
                      with slightly different local estimates of the same
                      object in the same step (see fuse_distributed).

Both architectures reuse the exact same clustering (_cluster) and
weighting (fuse_group) math - the fusion *mode* (naive/confidence/trust/
covariance/CI) still decides how sources combine once they're gathered.
The architecture only decides who gathers what, and what that gathering
costs in messages and delay.

Fusion modes
------------
  1. "no_fusion"                      - each UAV's own track stands alone,
                                         nothing is combined across UAVs.
  2. "naive_fusion"                   - plain (unweighted) average across
                                         UAVs whose tracks agree on the
                                         same object.
  3. "confidence_weighted_fusion"     - weighted average, weight = the
                                         track's own reported confidence.
  4. "trust_weighted_fusion"          - weighted average, weight =
                                         confidence * track-status
                                         reliability * a composite
                                         "reliability" score that also
                                         folds in measurement age, sensor
                                         latency, and dropout state (see
                                         "Reliability model" below).
  5. "covariance_weighted_fusion"     - information-filter (inverse-
                                         covariance) fusion: each track's
                                         2x2 position covariance is
                                         inflated by how unreliable it is,
                                         then sources are combined by
                                         weighting each with the inverse
                                         of that effective covariance.
                                         Statistically optimal only when
                                         sources' errors are independent.
  6. "covariance_intersection_fusion" - Covariance Intersection (CI):
                                         same covariance-aware idea as
                                         mode 5, but consistent even when
                                         sources' errors are correlated in
                                         some unknown way (e.g. several
                                         radars degraded together by the
                                         same weather/hardware condition,
                                         or a track re-observed after only
                                         a partial update). This is the
                                         mode to reach for whenever
                                         cross-source correlation can't be
                                         ruled out; modes 3-5 all
                                         implicitly assume independence.

Reliability model
------------------
Every track is turned into a fusion "source" (_as_source) carrying, on
top of its raw position estimate:
  - covariance       - the track's own 2x2 position covariance (top-left
                        block of the Kalman filter's 4x4 state
                        covariance), which already grows every step a
                        track coasts without a real detection - so age
                        and dropout are *already* partly baked into it
                        upstream, before fusion even sees it.
  - status_weight     - STATUS_RELIABILITY lookup (tentative/confirmed/
                        coasting/lost/deleted).
  - measurement_age_steps - the track's own missed_count: how many
                        consecutive steps since this source's last real
                        (non-predicted) detection. 0 means it was
                        genuinely updated this step.
  - dropout_state     - True while a track has gone without a fresh
                        detection this step (coasting/lost/deleted, or
                        missed_count > 0) - the fusion-time signal that a
                        source's radar may currently be dark, without
                        needing to reach into radar_like_model's internal
                        per-scan dropout flag.
  - sensor_latency_steps - static, config-known extra delay (in steps)
                        for this source's radar scan to arrive, passed in
                        by the caller when known (build_fused_log reads
                        it straight off RadarLikeModel); defaults to 0 for
                        callers that don't have it on hand (e.g. the live,
                        per-step pipeline in simple_swarm_sim.py).
  - persistent_trust - this UAV/sensor's *dynamic*, cross-step trust
                        score (see "Dynamic trust adaptation" below);
                        1.0 (no adjustment) for callers that don't pass a
                        TrustTracker.
  - reliability       - single composite score in
                        [MIN_RELIABILITY, 1.0] multiplying: status_weight
                        * confidence * an age-decay factor * a
                        latency-decay factor * a dropout penalty *
                        persistent_trust. Used directly as the
                        trust_weighted_fusion weight multiplier, and also
                        used to inflate each source's covariance
                        (eff_covariance = covariance / reliability)
                        before covariance_weighted_fusion /
                        covariance_intersection_fusion - a less reliable
                        source is treated as if it were noisier, so it
                        naturally gets down-weighted by the same
                        inverse-covariance math that handles genuine
                        measurement uncertainty. Folding persistent_trust
                        in here (rather than bolting it on separately)
                        means every fusion mode that already leans on
                        reliability benefits from dynamic trust
                        automatically, not just trust_weighted_fusion.

Dynamic trust adaptation
------------------------
Everything above (status, age, latency, dropout, confidence) is
*instantaneous* - recomputed from scratch every step with no memory of
what this sensor did last step. TrustTracker adds a second, slow-moving
score per UAV/radar_id (persistent_trust, folded into reliability above)
that accumulates across steps within a run:

  Decreases when:
    - the source's estimate repeatedly disagrees with the other UAVs
      currently tracking the same object (residual to the cluster's
      other members grows) - "measurement error repeatedly increases".
    - the source is repeatedly the odd one out with nobody else
      corroborating it while other UAVs *are* seeing something nearby -
      the best proxy fusion_model.py has for "false alarms increase"
      without reaching into radar_like_model's ground-truth-aware
      internals (see TrustTracker docstring for the honest caveat here).
    - measurement_age_steps keeps climbing - "data becomes stale".
    - dropout_state has been true often lately (a rolling window, not
      just this instant) - "sensor drops out frequently".
  Recovers, gradually (slower than it decays), when:
    - the source's estimate agrees with its cluster-mates again.
    - confidence is high and covariance is tight (or tightening) -
      "confidence and covariance improve".
    - the source has been reporting fresh, non-dropped-out data.

TrustTracker is intentionally *not* wired in by default - build_fused_log
creates one per (scenario, architecture) run when trust_adaptation is
enabled (on by default; see "trust_adaptation" config block / --disable-
adaptive-trust) and carries it across every step of that run, exactly the
way a real onboard trust estimator would persist across a mission rather
than resetting every step. It composes with, and is independent of, both
existing rate/reliability layers: communication.max_staleness_steps still
hard-rejects sources outright before trust ever sees them, and the
per-step communication channel (packet loss / range / corruption) still
governs which distributed broadcasts even arrive - persistent_trust only
adjusts how much a source that *did* arrive and pass staleness rejection
gets weighted.
"""

import argparse
import csv
import json
import math
import random
from collections import deque

import numpy as np

from models.radar_like_model import RadarLikeModel
from tracking.radar_track_model import build_tracks
import models.communication_model

NO_FUSION = "no_fusion"
NAIVE_FUSION = "naive_fusion"
CONFIDENCE_WEIGHTED_FUSION = "confidence_weighted_fusion"
TRUST_WEIGHTED_FUSION = "trust_weighted_fusion"
COVARIANCE_WEIGHTED_FUSION = "covariance_weighted_fusion"
COVARIANCE_INTERSECTION_FUSION = "covariance_intersection_fusion"

FUSION_MODES = (
    NO_FUSION,
    NAIVE_FUSION,
    CONFIDENCE_WEIGHTED_FUSION,
    TRUST_WEIGHTED_FUSION,
    COVARIANCE_WEIGHTED_FUSION,
    COVARIANCE_INTERSECTION_FUSION,
)

# --- fusion architectures --------------------------------------------------
ARCHITECTURE_CENTRALIZED = "centralized"
ARCHITECTURE_DISTRIBUTED = "distributed"
ARCHITECTURES = (ARCHITECTURE_CENTRALIZED, ARCHITECTURE_DISTRIBUTED)

# --- communication-model tuning constants ----------------------------------
# These model the messaging cost/delay of *getting tracks to wherever they
# get fused*, on top of (and separate from) each source's own sensor
# latency. All defaults are overridable per call / via a config
# "communication" block (see build_fused_log).

# Centralized: one step to move each UAV's track uplink to the central
# node, one step to broadcast the fused result back down. Nothing is
# usable by a UAV until both legs complete.
CENTRAL_UPLINK_LATENCY_STEPS = 1
CENTRAL_DOWNLINK_LATENCY_STEPS = 1

# Distributed: one step for a single peer-to-peer broadcast hop (no
# central relay, so no round trip - just the one hop).
DISTRIBUTED_HOP_LATENCY_STEPS = 1

# Distributed: probability that any single UAV-to-UAV broadcast this step
# is lost (independently per ordered pair, per step). 0.0 = perfectly
# reliable mesh; centralized has no equivalent knob today (its single
# uplink/downlink legs are assumed reliable, since a dropped central
# report is already covered by each source's own sensor_dropout_probability).
COMM_DROP_PROBABILITY = 0.0

# How close two UAVs' tracks have to be to be treated as the same
# real-world object. Same spirit as radar_track_model.GATE_DISTANCE.
CLUSTER_DISTANCE = 4.0

# Track-status reliability multiplier. "tentative" tracks (not yet
# confirmed over several hits) and "coasting" tracks (predicted forward
# with no detection this step) are trusted less than "confirmed" ones;
# "lost"/"deleted" tracks are on their way out and get the same low trust
# as tentative rather than the 1.0 default.
STATUS_RELIABILITY = {
    "tentative": 0.6,
    "confirmed": 1.0,
    "coasting": 0.4,
    "lost": 0.1,
    "deleted": 0.1,
}

# --- reliability-model tuning constants -----------------------------------
# How fast trust decays per consecutive step without a real detection
# (measurement age), per step of sensor latency, and the flat multiplier
# applied while a source has no fresh return this step (dropout state).
AGE_DECAY_PER_STEP = 0.12
LATENCY_DECAY_PER_STEP = 0.08
DROPOUT_PENALTY = 0.5
MIN_RELIABILITY = 0.05

# Fallback position variance (world units^2) for a source whose track row
# carries no usable covariance (e.g. malformed/missing JSON) - scaled down
# by confidence so a low-confidence fallback source is still trusted less.
FALLBACK_POSITION_VAR = 4.0

_EPS = 1e-9

# --- dynamic trust adaptation (TrustTracker) tuning constants --------------
# Asymmetric EWMA rates: trust falls quickly on a bad signal but only
# climbs back slowly on good ones, per the "decrease .. / recover
# gradually" requirement.
TRUST_ALPHA_DOWN = 0.30
TRUST_ALPHA_UP = 0.08
TRUST_MIN = 0.05
TRUST_MAX = 1.0
TRUST_INITIAL = 1.0

# How many recent steps the dropout-frequency signal looks back over.
TRUST_DROPOUT_WINDOW_STEPS = 10

# Residual (world units) to this source's cluster-mates below which it
# counts as full agreement, and above which it counts as full
# disagreement (linearly interpolated in between).
TRUST_DISAGREEMENT_SOFT_DISTANCE = 2.0
TRUST_DISAGREEMENT_HARD_DISTANCE = 5.0

# When a source is the only member of its cluster (nothing else nearby to
# compare against) but other sources *did* report elsewhere this step,
# fusion_model.py has no ground-truth-aware way to tell a genuine lone
# detection apart from a false alarm/phantom - that distinction lives
# upstream in radar_like_model.py, which knows which detections it
# generated as phantoms. This is the best available proxy without
# reaching into that ground truth, and is deliberately a moderate (not
# severe) penalty since being alone isn't proof of a false alarm.
TRUST_ISOLATED_AGREEMENT_SCORE = 0.3
# When this source is the *only* one reporting anything at all this step,
# there's nothing to agree or disagree with - neutral, not penalized.
TRUST_NO_PEERS_AGREEMENT_SCORE = 0.6

# Reference covariance trace (world units^2) used to normalize the
# covariance-tightness signal into roughly [0, 1].
TRUST_COVARIANCE_REFERENCE = 4.0

# How the five signals (agreement, freshness, dropout frequency,
# confidence, covariance tightness) are weighted into one target trust
# value each step. Kept equal by default; tune here rather than in the
# update logic if one signal should dominate.
TRUST_SIGNAL_WEIGHTS = {
    "agreement": 1.0,
    "freshness": 1.0,
    "dropout": 1.0,
    "confidence": 1.0,
    "covariance": 1.0,
}


class TrustTracker:
    """Maintains a slowly-adapting, per-UAV/radar trust score across the
    steps of a single run - the "dynamic" half of the reliability model
    (see module docstring's "Dynamic trust adaptation" section). Fully
    optional: code that never constructs one gets persistent_trust=1.0
    (no adjustment) everywhere, exactly the pre-Task-14 behavior.

    Nothing here ever reads ground truth - every signal it uses (residual
    to cluster-mates, confidence, covariance, dropout state, measurement
    age) is something a real UAV would already have on hand from its own
    and its peers' broadcast track summaries.
    """

    def __init__(self, alpha_up=TRUST_ALPHA_UP, alpha_down=TRUST_ALPHA_DOWN,
                 dropout_window_steps=TRUST_DROPOUT_WINDOW_STEPS,
                 disagreement_soft_distance=TRUST_DISAGREEMENT_SOFT_DISTANCE,
                 disagreement_hard_distance=TRUST_DISAGREEMENT_HARD_DISTANCE,
                 signal_weights=None):
        self.alpha_up = alpha_up
        self.alpha_down = alpha_down
        self.dropout_window_steps = dropout_window_steps
        self.disagreement_soft_distance = disagreement_soft_distance
        self.disagreement_hard_distance = disagreement_hard_distance
        self.signal_weights = dict(signal_weights or TRUST_SIGNAL_WEIGHTS)

        self._trust = {}                                    # radar_id -> float
        self._dropout_history = {}                           # radar_id -> deque[bool]
        self._prev_covariance_trace = {}                     # radar_id -> float
        self._last_signals = {}                              # radar_id -> dict, for logging/debugging

    def get(self, radar_id):
        """Current persistent trust for a UAV/radar, defaulting to
        TRUST_INITIAL the first time it's ever seen."""
        return self._trust.get(radar_id, TRUST_INITIAL)

    def last_signals(self, radar_id):
        """Returns the raw per-signal breakdown from this radar's most
        recent update() call (agreement/freshness/dropout/confidence/
        covariance scores plus the resulting target and new trust) -
        useful for debugging or plotting why a trust score moved, without
        needing to re-derive it from the CSV output."""
        return self._last_signals.get(radar_id)

    def snapshot(self):
        """Returns {radar_id: current_trust} for every radar seen so far -
        handy for logging a per-step trust table alongside the fused
        rows."""
        return dict(self._trust)

    def _agreement_score(self, source, cluster):
        """How well this source's position agrees with the rest of its
        cluster this step. 1.0 = perfect agreement, 0.0 = far outlier."""
        others = [m for m in cluster if m is not source]
        if others:
            mean_x = sum(m["x"] for m in others) / len(others)
            mean_y = sum(m["y"] for m in others) / len(others)
            residual = math.hypot(source["x"] - mean_x, source["y"] - mean_y)
            if residual <= self.disagreement_soft_distance:
                return 1.0
            if residual >= self.disagreement_hard_distance:
                return 0.0
            span = self.disagreement_hard_distance - self.disagreement_soft_distance
            return 1.0 - (residual - self.disagreement_soft_distance) / span
        return None  # no cluster-mates - caller decides isolated vs. no-peers-at-all

    def update(self, sources, clusters):
        """Call once per step with every source seen that step (already
        built by _as_source) and how they clustered (_cluster's output),
        to advance each represented radar_id's persistent trust ready for
        *next* step's weighting. Returns {radar_id: new_trust}.

        Deliberately uses *this* step's fused/clustering picture to
        decide *next* step's trust, rather than letting a source's own
        trust influence the very residual used to judge it this step -
        that ordering is what build_fused_log/fuse_step preserve by
        looking up trust before fusing and calling update() after.
        """
        cluster_of = {}
        for cluster in clusters:
            for s in cluster:
                cluster_of[s["source_id"]] = cluster

        total_sources_this_step = len(sources)
        new_trust = {}
        for s in sources:
            radar_id = s["radar_id"]
            cluster = cluster_of.get(s["source_id"], [s])

            agreement = self._agreement_score(s, cluster)
            if agreement is None:
                agreement = (TRUST_NO_PEERS_AGREEMENT_SCORE if total_sources_this_step == 1
                             else TRUST_ISOLATED_AGREEMENT_SCORE)

            freshness = 1.0 / (1.0 + AGE_DECAY_PER_STEP * s["measurement_age_steps"])

            window = self._dropout_history.setdefault(radar_id, deque(maxlen=self.dropout_window_steps))
            window.append(bool(s["dropout_state"]))
            dropout_frequency = sum(window) / len(window)
            dropout_score = 1.0 - dropout_frequency

            confidence_score = max(0.0, min(1.0, s["confidence"]))

            trace = float(np.trace(s["covariance"]))
            covariance_score = 1.0 / (1.0 + trace / TRUST_COVARIANCE_REFERENCE)
            prev_trace = self._prev_covariance_trace.get(radar_id)
            if prev_trace is not None and trace < prev_trace:
                # Tightening covariance (improving) is itself worth a
                # small extra nudge on top of the absolute tightness score.
                covariance_score = min(1.0, covariance_score + 0.1)
            self._prev_covariance_trace[radar_id] = trace

            weights = self.signal_weights
            total_w = sum(weights.values())
            target = (
                weights["agreement"] * agreement
                + weights["freshness"] * freshness
                + weights["dropout"] * dropout_score
                + weights["confidence"] * confidence_score
                + weights["covariance"] * covariance_score
            ) / total_w

            current = self.get(radar_id)
            alpha = self.alpha_down if target < current else self.alpha_up
            updated = current + alpha * (target - current)
            updated = max(TRUST_MIN, min(TRUST_MAX, updated))

            self._trust[radar_id] = updated
            self._last_signals[radar_id] = {
                "agreement": round(agreement, 4),
                "freshness": round(freshness, 4),
                "dropout_score": round(dropout_score, 4),
                "confidence_score": round(confidence_score, 4),
                "covariance_score": round(covariance_score, 4),
                "target": round(target, 4),
                "prev_trust": round(current, 4),
                "new_trust": round(updated, 4),
            }
            new_trust[radar_id] = updated
        return new_trust


def _position_covariance(track):
    """Extracts the 2x2 position block from a track row's 4x4 state
    covariance (over [x, y, vx, vy]), as logged by radar_track_model.py.
    Returns None if the row has no usable covariance."""
    raw = track.get("covariance")
    if raw is None:
        return None
    try:
        mat = json.loads(raw) if isinstance(raw, str) else raw
        P = np.array(mat, dtype=float)[:2, :2]
        if P.shape != (2, 2) or not np.all(np.isfinite(P)):
            return None
        return P
    except (ValueError, TypeError, IndexError):
        return None


def _as_source(track, sensor_latency_steps=0, sensor_dropout_probability=0.0, persistent_trust=1.0):
    """Normalizes a radar_track_model row into the shape fusion works
    with, computing the composite reliability score described in the
    module docstring. Nothing here reads ground truth - every input is
    either on the track row itself, a static, config-known sensor
    characteristic (latency, dropout probability), or this radar's own
    dynamic trust score as tracked by TrustTracker (also never
    ground-truth-derived - see TrustTracker's docstring)."""
    status = track.get("status", "confirmed")
    status_weight = STATUS_RELIABILITY.get(status, 1.0)
    confidence = track.get("confidence")
    confidence = confidence if confidence is not None else 0.5
    missed = track.get("missed_count") or 0
    age_steps = missed
    dropout_state = bool(status in ("coasting", "lost", "deleted") or missed > 0)

    cov = _position_covariance(track)
    if cov is None:
        base_var = FALLBACK_POSITION_VAR / max(confidence, 0.05)
        cov = np.diag([base_var, base_var])

    age_discount = 1.0 / (1.0 + AGE_DECAY_PER_STEP * age_steps)
    latency_discount = 1.0 / (1.0 + LATENCY_DECAY_PER_STEP * sensor_latency_steps)
    dropout_discount = DROPOUT_PENALTY if dropout_state else 1.0
    # Sensors with a higher baseline dropout probability are inherently
    # less trustworthy even on a step where they happen to report - folds
    # the static per-scenario dropout risk in as a further discount.
    dropout_risk_discount = 1.0 - 0.5 * min(max(sensor_dropout_probability, 0.0), 1.0)
    persistent_trust = max(TRUST_MIN, min(TRUST_MAX, persistent_trust))

    reliability = max(
        MIN_RELIABILITY,
        status_weight * confidence * age_discount * latency_discount
        * dropout_discount * dropout_risk_discount * persistent_trust,
    )

    return {
        "source_id": track["track_id"],
        "radar_id": track["radar_id"],
        "x": track["est_x"],
        "y": track["est_y"],
        "confidence": confidence,
        "status": status,
        "status_weight": status_weight,
        "measurement_age_steps": age_steps,
        "sensor_latency_steps": sensor_latency_steps,
        "dropout_state": dropout_state,
        "persistent_trust": round(float(persistent_trust), 4),
        "reliability": round(float(reliability), 4),
        "covariance": cov,
        "eff_covariance": cov / reliability,
    }


def _cluster(sources, cluster_distance=CLUSTER_DISTANCE):
    """Greedy single-linkage clustering by position: groups tracks (from
    different UAVs' radars) that likely refer to the same real-world
    object.

    ponytail: O(n^2) greedy clustering - fine for the handful of tracks per
    step this sim has (one obstacle, a few UAV radars). If the sim grows to
    track several simultaneous targets, this is the spot that would need a
    real multi-target data-association step (e.g. Hungarian assignment)
    instead of "closest thing wins".
    """
    remaining = list(sources)
    clusters = []
    while remaining:
        seed = remaining.pop(0)
        group = [seed]
        rest = []
        for s in remaining:
            if math.hypot(s["x"] - seed["x"], s["y"] - seed["y"]) <= cluster_distance:
                group.append(s)
            else:
                rest.append(s)
        remaining = rest
        clusters.append(group)
    return clusters


def _weighted_average_xy(group, weights):
    total_w = sum(weights)
    if total_w <= _EPS:
        weights = [1.0 for _ in group]
        total_w = len(group)
    fx = sum(s["x"] * w for s, w in zip(group, weights)) / total_w
    fy = sum(s["y"] * w for s, w in zip(group, weights)) / total_w
    return fx, fy


def _safe_inv(P):
    try:
        return np.linalg.inv(P)
    except np.linalg.LinAlgError:
        return np.eye(2) / FALLBACK_POSITION_VAR


def _information_fusion_xy(group):
    """Inverse-covariance (information filter) fusion: statistically
    optimal when sources' errors are independent. Each source's effective
    covariance (already inflated by its reliability score) is inverted
    into an "information" matrix; those are simply summed, which is the
    closed-form update for combining independent Gaussian estimates."""
    P_inv_sum = np.zeros((2, 2))
    x_weighted_sum = np.zeros(2)
    for s in group:
        P_inv = _safe_inv(s["eff_covariance"])
        P_inv_sum += P_inv
        x_weighted_sum += P_inv @ np.array([s["x"], s["y"]])
    P_fused = _safe_inv(P_inv_sum)
    x_fused = P_fused @ x_weighted_sum
    return x_fused, P_fused


def _covariance_intersection_pair(xa, Pa, xb, Pb, n_search=41):
    """Classic two-source Covariance Intersection (Julier & Uhlmann): for
    any mixing weight w in [0, 1],
        P^-1 = w*Pa^-1 + (1-w)*Pb^-1
        x    = P * (w*Pa^-1*xa + (1-w)*Pb^-1*xb)
    is a *consistent* fused estimate no matter how correlated a and b's
    errors actually are (no independence assumption needed, unlike the
    information filter above). We pick w by a small grid search that
    minimizes the fused covariance's trace, which is the usual practical
    stand-in for the full determinant-minimizing optimization."""
    Pa_inv = _safe_inv(Pa)
    Pb_inv = _safe_inv(Pb)
    best = None  # (score, |w - 0.5|, x, P)
    tol = 1e-9
    for i in range(n_search):
        w = min(max(i / (n_search - 1), 1e-3), 1 - 1e-3)
        P_inv = w * Pa_inv + (1 - w) * Pb_inv
        P = _safe_inv(P_inv)
        x = P @ (w * Pa_inv @ xa + (1 - w) * Pb_inv @ xb)
        score = float(np.trace(P))
        balance = abs(w - 0.5)
        # Prefer strictly lower trace; on a (near-)tie, prefer the more
        # balanced weight instead of whichever w happened to be scanned
        # first - otherwise equal-covariance sources fuse asymmetrically
        # toward whichever end of the grid search ran first.
        if best is None or score < best[0] - tol or (abs(score - best[0]) <= tol and balance < best[1]):
            best = (score, balance, x, P)
    return best[2], best[3]


def _covariance_intersection_xy(group):
    """N-source CI by sequential pairwise reduction: fuse the first two
    sources with CI, then fuse that result with the third, and so on.
    Each pairwise step is individually consistency-preserving regardless
    of correlation, which is what CI guarantees; sequential reduction is
    the standard practical way to extend the (inherently two-source) CI
    formula to a group, at the cost of the result depending slightly on
    fusion order."""
    x = np.array([group[0]["x"], group[0]["y"]])
    P = group[0]["eff_covariance"]
    for s in group[1:]:
        x, P = _covariance_intersection_pair(
            x, P, np.array([s["x"], s["y"]]), s["eff_covariance"])
    return x, P


def fuse_group(group, fusion_mode):
    """Fuses one cluster of same-object tracks (from different UAVs) into
    a single estimate."""
    if len(group) == 1:
        s = group[0]
        return {"x": s["x"], "y": s["y"], "confidence": s["confidence"],
                "num_sources": 1, "source_ids": [s["source_id"]],
                "position_variance": round(float(np.trace(s["covariance"])), 4),
                "avg_persistent_trust": s["persistent_trust"]}

    fused_cov = None
    if fusion_mode == NAIVE_FUSION:
        # Every UAV's track counts equally - confidence, covariance, and
        # reliability all ignored on purpose.
        fx, fy = _weighted_average_xy(group, [1.0 for _ in group])
    elif fusion_mode == CONFIDENCE_WEIGHTED_FUSION:
        fx, fy = _weighted_average_xy(group, [s["confidence"] for s in group])
    elif fusion_mode == TRUST_WEIGHTED_FUSION:
        weights = [s["confidence"] * s["status_weight"] * s["reliability"] for s in group]
        fx, fy = _weighted_average_xy(group, weights)
    elif fusion_mode == COVARIANCE_WEIGHTED_FUSION:
        x_fused, fused_cov = _information_fusion_xy(group)
        fx, fy = float(x_fused[0]), float(x_fused[1])
    elif fusion_mode == COVARIANCE_INTERSECTION_FUSION:
        x_fused, fused_cov = _covariance_intersection_xy(group)
        fx, fy = float(x_fused[0]), float(x_fused[1])
    else:
        raise ValueError(f"Unknown fusion_mode: {fusion_mode!r} (expected one of {FUSION_MODES})")

    avg_conf = sum(s["confidence"] for s in group) / len(group)
    # More independent UAVs agreeing raises fused confidence a bit - same
    # idea as simple_swarm_sim.py's existing fuse_obstacle_detections.
    fused_conf = min(1.0, avg_conf + 0.08 * (len(group) - 1))
    avg_persistent_trust = round(sum(s["persistent_trust"] for s in group) / len(group), 4)

    result = {"x": fx, "y": fy, "confidence": round(fused_conf, 3),
              "num_sources": len(group), "source_ids": [s["source_id"] for s in group],
              "avg_persistent_trust": avg_persistent_trust}
    if fused_cov is not None:
        result["position_variance"] = round(float(np.trace(fused_cov)), 4)
    return result


def _fuse_sources(sources, fusion_mode, cluster_distance):
    """Core gather-then-fuse step, shared by both architectures: given
    whatever sources one fusion point currently has on hand (all of them,
    for centralized; one UAV's own view, for distributed), cluster the
    ones that likely refer to the same object and fuse each cluster.
    Contains no notion of *how* those sources got here - that's each
    architecture's job."""
    if not sources:
        return []

    if fusion_mode == NO_FUSION:
        # Nothing gets combined - every source stands on its own, exactly
        # what "no_fusion" scenarios elsewhere in this project mean.
        return [{"x": s["x"], "y": s["y"], "confidence": s["confidence"],
                 "num_sources": 1, "source_ids": [s["source_id"]],
                 "position_variance": round(float(np.trace(s["covariance"])), 4),
                 "avg_persistent_trust": s["persistent_trust"]}
                for s in sources]

    clusters = _cluster(sources, cluster_distance)
    return [fuse_group(g, fusion_mode) for g in clusters]


def _sources_with_trust(radar_tracks, sensor_latency_steps, sensor_dropout_probability, trust_tracker):
    """Builds this step's sources, looking up each radar's current
    persistent trust (as of *before* this step, i.e. from everything
    TrustTracker has seen up through last step) if a tracker was given."""
    return [
        _as_source(t, sensor_latency_steps, sensor_dropout_probability,
                   persistent_trust=trust_tracker.get(t["radar_id"]) if trust_tracker else 1.0)
        for t in radar_tracks
    ]


def _advance_trust(sources, trust_tracker, cluster_distance):
    """Feeds this step's sources back into the tracker so every
    represented radar_id's trust is ready for *next* step - done after
    this step's own weighting already happened, so a source is never
    judged using a trust value derived from the very thing being judged.
    Runs regardless of fusion_mode (including no_fusion): trust is a
    property of the sensor, tracked independent of whether its output is
    currently being combined with anyone else's."""
    if trust_tracker is None or not sources:
        return
    clusters = _cluster(sources, cluster_distance)
    trust_tracker.update(sources, clusters)


def fuse_centralized(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
                      sensor_latency_steps=0, sensor_dropout_probability=0.0,
                      uplink_latency_steps=CENTRAL_UPLINK_LATENCY_STEPS,
                      downlink_latency_steps=CENTRAL_DOWNLINK_LATENCY_STEPS,
                      max_staleness_steps=None, trust_tracker=None):
    """Centralized architecture: every UAV's track is treated as already
    having arrived at one central fusion node (that's what building
    `radar_tracks` as a single pooled list already represents), fused
    once into a single per-object world estimate, then annotated with
    what that round trip actually costs:

      - comm_messages     - one uplink message per contributing UAV, plus
                             one downlink broadcast back out to the swarm
                             (so len(radar_tracks) + 1, not len(radar_tracks)
                             per UAV - a single broadcast reaches everyone).
      - response_time_steps - uplink + downlink legs, on top of whatever
                             latency each source's own sensor already has
                             (the max across sources, since the center
                             can't finish until its slowest input arrives).

    max_staleness_steps, if given, hard-rejects any source whose
    measurement_age_steps exceeds it before fusing - the "latest available
    measurements ... stale-data rejection" half of asynchronous multi-rate
    sensing, on top of the soft age_discount reliability already applies.

    `trust_tracker`, if given, both supplies this step's persistent_trust
    (folded into each source's reliability before fusing) and gets
    updated afterwards from this step's agreement/freshness/dropout/
    confidence/covariance signals - computed only from the sources that
    actually took part in this round (i.e. after max_staleness_steps
    rejection), ready for next step.

    Returns one row per fused object (same shape fuse_step always
    returned), each carrying architecture/comm/response_time fields.
    """
    if fusion_mode not in FUSION_MODES:
        raise ValueError(f"Unknown fusion_mode: {fusion_mode!r} (expected one of {FUSION_MODES})")

    sources = _sources_with_trust(radar_tracks, sensor_latency_steps, sensor_dropout_probability, trust_tracker)
    if max_staleness_steps is not None:
        sources = [s for s in sources if s["measurement_age_steps"] <= max_staleness_steps]
    fused = _fuse_sources(sources, fusion_mode, cluster_distance)

    num_uavs_reporting = len(sources)
    comm_messages = num_uavs_reporting + (1 if num_uavs_reporting else 0)  # uplinks + one broadcast
    slowest_source_latency = max((s["sensor_latency_steps"] for s in sources), default=0)
    response_time_steps = slowest_source_latency + uplink_latency_steps + downlink_latency_steps

    for row in fused:
        row["architecture"] = ARCHITECTURE_CENTRALIZED
        row["local_uav_id"] = None  # one shared answer, not UAV-specific
        row["comm_messages"] = comm_messages
        row["response_time_steps"] = response_time_steps

    _advance_trust(sources, trust_tracker, cluster_distance)
    return fused


def fuse_distributed(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
                      sensor_latency_steps=0, sensor_dropout_probability=0.0,
                      comm_drop_probability=COMM_DROP_PROBABILITY,
                      hop_latency_steps=DISTRIBUTED_HOP_LATENCY_STEPS,
                      channel=None, rng=None, trust_tracker=None):
    """Distributed architecture: no central node. Every UAV that produced
    a track broadcasts it to every other UAV over `channel` (a
    communication_model.CommunicationChannel - built from
    comm_drop_probability/hop_latency_steps if not given, for backward
    compatibility). Each such broadcast can be lost to packet loss, be
    out of range, or be rejected as stale by the channel; each UAV then
    fuses *its own* track together with whatever peers' tracks actually
    arrived - so different UAVs can end up with different local estimates
    of the same object this step.

    `trust_tracker`, if given, supplies persistent_trust for every UAV's
    track (looked up once, shared across every receiver's local fuse -
    trust is a property of the sensor, not of who's currently listening
    to it) and is updated once per step from the full swarm-wide
    agreement picture (every source that produced a track this step, not
    just what a given receiver happened to have delivered to it), not
    separately per receiver.

    Returns one row per (uav_id, fused object) - i.e. potentially several
    rows per object, one per UAV's local view of it - each carrying
    architecture/comm/response_time fields plus which UAV it's the local
    view for.
    """
    if fusion_mode not in FUSION_MODES:
        raise ValueError(f"Unknown fusion_mode: {fusion_mode!r} (expected one of {FUSION_MODES})")

    rng = rng or np.random.default_rng()
    channel = channel or models.communication_model.CommunicationChannel(
        packet_loss_probability=comm_drop_probability,
        base_latency_steps=hop_latency_steps,
        rng=random.Random(int(rng.integers(0, 2**31 - 1))))
    all_sources = _sources_with_trust(radar_tracks, sensor_latency_steps, sensor_dropout_probability, trust_tracker)
    if not all_sources:
        return []

    receivers = sorted({s["radar_id"] for s in all_sources})
    n = len(receivers)
    # Attempted messages: every UAV broadcasts its own track to every
    # *other* UAV - this is the communication load whether or not each
    # individual broadcast is actually delivered.
    attempted_messages = n * (n - 1)
    delivered_messages = 0

    fused_rows = []
    for receiver_id in receivers:
        own = [s for s in all_sources if s["radar_id"] == receiver_id]
        received_from_peers = []
        for s in all_sources:
            if s["radar_id"] == receiver_id:
                continue
            delivered, outcome = channel.transmit(
                s, measurement_age_steps=s["measurement_age_steps"])
            if outcome == "delivered":
                received_from_peers.append(delivered)
                delivered_messages += 1
        local_sources = own + received_from_peers
        for row in _fuse_sources(local_sources, fusion_mode, cluster_distance):
            row["architecture"] = ARCHITECTURE_DISTRIBUTED
            row["local_uav_id"] = receiver_id
            row["comm_messages"] = attempted_messages
            row["response_time_steps"] = sensor_latency_steps + channel.base_latency_steps
            fused_rows.append(row)

    for row in fused_rows:
        row["comm_messages_delivered"] = delivered_messages

    _advance_trust(all_sources, trust_tracker, cluster_distance)
    return fused_rows


def fuse_step(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
              sensor_latency_steps=0, sensor_dropout_probability=0.0,
              architecture=ARCHITECTURE_CENTRALIZED, trust_tracker=None, **architecture_kwargs):
    """Fuses one time step's worth of radar tracks - one list across all of
    the swarm's UAVs, already sensor output, never ground truth - into
    per-object fused estimates, using whichever `architecture` decides how
    those tracks get gathered (see the module docstring's "Fusion
    architectures" section).

    sensor_latency_steps / sensor_dropout_probability are static,
    config-known sensor characteristics (not per-track data); callers that
    have them on hand (build_fused_log below) pass them through so the
    reliability model can account for latency and baseline dropout risk.
    Callers that don't (e.g. simple_swarm_sim.py's live pipeline) simply
    omit them and get the neutral defaults - measurement age, status, and
    covariance already carry most of the useful signal on their own.

    `trust_tracker`, if given, is an already-constructed TrustTracker that
    the caller keeps alive across steps (see "Dynamic trust adaptation" in
    the module docstring); omitting it keeps every source's
    persistent_trust at 1.0 (no adjustment) exactly as before Task 14.

    Defaults to "centralized", which reproduces this function's exact
    pre-architecture behavior (one row per object, no architecture/comm
    fields consumers didn't already expect) for every existing caller.
    """
    if architecture not in ARCHITECTURES:
        raise ValueError(f"Unknown architecture: {architecture!r} (expected one of {ARCHITECTURES})")

    if architecture == ARCHITECTURE_CENTRALIZED:
        return fuse_centralized(radar_tracks, fusion_mode, cluster_distance,
                                 sensor_latency_steps, sensor_dropout_probability,
                                 trust_tracker=trust_tracker, **architecture_kwargs)
    return fuse_distributed(radar_tracks, fusion_mode, cluster_distance,
                             sensor_latency_steps, sensor_dropout_probability,
                             trust_tracker=trust_tracker, **architecture_kwargs)


def build_fused_log(scenario_name, config, architecture=ARCHITECTURE_CENTRALIZED, seed=None,
                     use_adaptive_trust=True):
    """Runs the radar model + tracker for one scenario, then fuses each
    step's tracks across all UAVs using the given fusion architecture.
    Returns the list of fused-estimate rows for that scenario - one row
    per fused object per step for "centralized", or one row per
    (uav_id, fused object) per step for "distributed" (see fuse_step /
    fuse_distributed).

    Two asynchronous-rate knobs, read from config["communication"] /
    config["sim"]:
      - fusion_update_rate (Hz) - fusion only actually recomputes on
        steps due for one (default: every step); steps in between re-serve
        the last computed result, marked `is_stale`/`fusion_age_steps` -
        "latest available measurements" for a fusion stage slower than the
        sim's own step rate, mirroring how radar/vision/LiDAR each hold
        their own last scan between updates.
      - communication.max_staleness_steps - hard-rejects any source too
        old to trust regardless of soft reliability discounting (Task 8's
        "stale-data rejection"); communication.preset/packet_loss_*/
        comm_range/corruption_probability configure the actual channel
        messages travel over (see communication_model.py).

    `seed` seeds the distributed architecture's communication-drop RNG so
    runs are reproducible; ignored for "centralized".

    `use_adaptive_trust` controls whether a TrustTracker is created and
    carried across every step of this run (see "Dynamic trust
    adaptation" in the module docstring). On by default; a
    config["trust_adaptation"] block can further tune it or disable it
    per scenario via {"enabled": false} - set use_adaptive_trust=False to
    disable it for the whole call regardless of config, reproducing
    pre-Task-14 behavior (persistent_trust fixed at 1.0 everywhere). A
    fusion stage that only recomputes every fusion_update_interval_steps
    only feeds the tracker on the steps it actually runs, exactly like a
    real onboard trust estimator that only has something new to learn
    from on those same steps.
    """
    model = RadarLikeModel(config, scenario_name)
    detection_rows = model.run()
    dt = config["sim"]["dt"]
    track_rows = build_tracks(scenario_name, detection_rows, dt)

    fusion_mode = model.sim.scn.get(
        "fusion_mode", config.get("perception_errors", {}).get("fusion_mode", NO_FUSION))

    comm_cfg = model.sim.scn.get("communication", config.get("communication", {}))
    channel_rng = random.Random(seed)
    architecture_kwargs = {"max_staleness_steps": comm_cfg.get("max_staleness_steps")}
    if architecture == ARCHITECTURE_CENTRALIZED:
        architecture_kwargs["uplink_latency_steps"] = comm_cfg.get(
            "central_uplink_latency_steps", CENTRAL_UPLINK_LATENCY_STEPS)
        architecture_kwargs["downlink_latency_steps"] = comm_cfg.get(
            "central_downlink_latency_steps", CENTRAL_DOWNLINK_LATENCY_STEPS)
    else:
        del architecture_kwargs["max_staleness_steps"]  # folded into the channel itself below
        architecture_kwargs["channel"] = models.communication_model.from_config(
            comm_cfg, rng=random.Random(channel_rng.randint(0, 2**31 - 1)))
        architecture_kwargs["rng"] = np.random.default_rng(seed)

    fusion_update_rate = config.get("sim", {}).get("fusion_update_rate", 0) or comm_cfg.get(
        "fusion_update_rate", 0)
    # Distributed peer-to-peer exchange has its own cadence, separate from
    # how often the centralized round trip happens - falls back to
    # fusion_update_rate if not set, since in both cases "not due yet"
    # means the same thing: hold and re-serve the last computed result.
    if architecture == ARCHITECTURE_DISTRIBUTED and comm_cfg.get("communication_update_rate"):
        fusion_update_rate = comm_cfg["communication_update_rate"]
    fusion_update_interval_steps = (
        max(1, round(1.0 / (fusion_update_rate * dt))) if fusion_update_rate else 1)

    trust_tracker = None
    if use_adaptive_trust:
        trust_cfg = model.sim.scn.get("trust_adaptation", config.get("trust_adaptation", {}))
        if trust_cfg.get("enabled", True):
            trust_tracker = TrustTracker(
                alpha_up=trust_cfg.get("alpha_up", TRUST_ALPHA_UP),
                alpha_down=trust_cfg.get("alpha_down", TRUST_ALPHA_DOWN),
                dropout_window_steps=trust_cfg.get("dropout_window_steps", TRUST_DROPOUT_WINDOW_STEPS),
                disagreement_soft_distance=trust_cfg.get(
                    "disagreement_soft_distance", TRUST_DISAGREEMENT_SOFT_DISTANCE),
                disagreement_hard_distance=trust_cfg.get(
                    "disagreement_hard_distance", TRUST_DISAGREEMENT_HARD_DISTANCE),
            )

    by_step = {}
    for row in track_rows:
        by_step.setdefault(row["time_step"], []).append(row)

    fused_rows = []
    held_fused, held_step = None, None
    for step in sorted(by_step):
        if step % fusion_update_interval_steps == 0:
            fused = fuse_step(by_step[step], fusion_mode,
                               sensor_latency_steps=model.radar_latency_steps,
                               sensor_dropout_probability=model.radar_dropout_probability,
                               architecture=architecture, trust_tracker=trust_tracker,
                               **architecture_kwargs)
            held_fused, held_step = fused, step
            is_stale = False
        else:
            fused, is_stale = (held_fused or []), True

        for f in fused:
            fused_rows.append({
                "scenario": scenario_name,
                "time_step": step,
                "fusion_mode": fusion_mode,
                "architecture": f["architecture"],
                "local_uav_id": f["local_uav_id"],
                "fused_x": round(f["x"], 4),
                "fused_y": round(f["y"], 4),
                "fused_confidence": round(f["confidence"], 4),
                "num_sources": f["num_sources"],
                "position_variance": f.get("position_variance"),
                "avg_persistent_trust": f.get("avg_persistent_trust"),
                "comm_messages": f.get("comm_messages"),
                "comm_messages_delivered": f.get("comm_messages_delivered"),
                "response_time_steps": f.get("response_time_steps"),
                "source_track_ids": ";".join(f["source_ids"]),
                "is_stale": is_stale,
                "fusion_age_steps": (step - held_step) if held_step is not None else 0,
            })
    return fused_rows


def estimation_error_against_ground_truth(fused_rows, ground_truth_xy):
    """Evaluation-only helper (never used by fusion itself - see the
    module docstring): given fused rows and a known ground-truth (x, y)
    for the object they estimate, returns the mean and max Euclidean
    error. This is exactly the kind of "check afterwards" use of ground
    truth metrics_analysis.py / simulation_visualizer.py already do;
    fuse_group/fuse_step never see or use it.

    For "distributed" rows (one row per UAV per step), this naturally
    averages over every UAV's local estimate too, not just over time -
    which is the point: it lets per-UAV disagreement show up as error,
    not just get hidden by averaging away.
    """
    if not fused_rows:
        return {"mean_error": None, "max_error": None, "n": 0}
    gx, gy = ground_truth_xy
    def _xy(r):
        return (r["fused_x"], r["fused_y"]) if "fused_x" in r else (r["x"], r["y"])
    errors = [math.hypot(*(v - g for v, g in zip(_xy(r), (gx, gy)))) for r in fused_rows]
    return {
        "mean_error": round(sum(errors) / len(errors), 4),
        "max_error": round(max(errors), 4),
        "n": len(errors),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Fuse radar tracks from all UAVs into per-object estimates")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--fusion-mode", default=None, choices=FUSION_MODES,
                         help="Override the scenario's configured fusion_mode")
    parser.add_argument("--architecture", default=ARCHITECTURE_CENTRALIZED, choices=ARCHITECTURES,
                         help="Where fusion happens: one central node, or each UAV locally")
    parser.add_argument("--compare-architectures", action="store_true",
                         help="Ignore --architecture and instead run every scenario once under "
                              "each architecture, writing both into --log with an 'architecture' "
                              "column so they can be compared directly")
    parser.add_argument("--seed", type=int, default=None,
                         help="Seeds the distributed architecture's communication-drop RNG")
    parser.add_argument("--disable-adaptive-trust", action="store_true",
                         help="Fix persistent_trust at 1.0 for every source (pre-Task-14 behavior) "
                              "instead of tracking dynamic per-UAV trust across the run")
    parser.add_argument("--log", default="logs/fused_track_log.csv")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    if args.fusion_mode:
        for scn in config["scenarios"].values():
            scn["fusion_mode"] = args.fusion_mode

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    architectures = list(ARCHITECTURES) if args.compare_architectures else [args.architecture]

    all_rows = []
    for name in scenario_names:
        for architecture in architectures:
            rows = build_fused_log(name, config, architecture=architecture, seed=args.seed,
                                    use_adaptive_trust=not args.disable_adaptive_trust)
            all_rows.extend(rows)
            print(f"{name} [{architecture}]: {len(rows)} fused rows")

    if all_rows:
        import os
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote {len(all_rows)} rows to {args.log}")


if __name__ == "__main__":
    main()