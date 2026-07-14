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
  - reliability       - single composite score in
                        [MIN_RELIABILITY, 1.0] multiplying: status_weight
                        * confidence * an age-decay factor * a
                        latency-decay factor * a dropout penalty. Used
                        directly as the trust_weighted_fusion weight
                        multiplier, and also used to inflate each
                        source's covariance (eff_covariance = covariance
                        / reliability) before covariance_weighted_fusion
                        / covariance_intersection_fusion - a less
                        reliable source is treated as if it were noisier,
                        so it naturally gets down-weighted by the same
                        inverse-covariance math that handles genuine
                        measurement uncertainty.
"""

import argparse
import csv
import json
import math

import numpy as np

from radar_like_model import RadarLikeModel
from radar_track_model import build_tracks

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


def _as_source(track, sensor_latency_steps=0, sensor_dropout_probability=0.0):
    """Normalizes a radar_track_model row into the shape fusion works
    with, computing the composite reliability score described in the
    module docstring. Nothing here reads ground truth - every input is
    either on the track row itself or a static, config-known sensor
    characteristic (latency, dropout probability) passed in by the
    caller."""
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

    reliability = max(
        MIN_RELIABILITY,
        status_weight * confidence * age_discount * latency_discount
        * dropout_discount * dropout_risk_discount,
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
                "position_variance": round(float(np.trace(s["covariance"])), 4)}

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

    result = {"x": fx, "y": fy, "confidence": round(fused_conf, 3),
              "num_sources": len(group), "source_ids": [s["source_id"] for s in group]}
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
                 "position_variance": round(float(np.trace(s["covariance"])), 4)}
                for s in sources]

    clusters = _cluster(sources, cluster_distance)
    return [fuse_group(g, fusion_mode) for g in clusters]


def fuse_centralized(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
                      sensor_latency_steps=0, sensor_dropout_probability=0.0,
                      uplink_latency_steps=CENTRAL_UPLINK_LATENCY_STEPS,
                      downlink_latency_steps=CENTRAL_DOWNLINK_LATENCY_STEPS):
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

    Returns one row per fused object (same shape fuse_step always
    returned), each carrying architecture/comm/response_time fields.
    """
    if fusion_mode not in FUSION_MODES:
        raise ValueError(f"Unknown fusion_mode: {fusion_mode!r} (expected one of {FUSION_MODES})")

    sources = [_as_source(t, sensor_latency_steps, sensor_dropout_probability)
               for t in radar_tracks]
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
    return fused


def fuse_distributed(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
                      sensor_latency_steps=0, sensor_dropout_probability=0.0,
                      comm_drop_probability=COMM_DROP_PROBABILITY,
                      hop_latency_steps=DISTRIBUTED_HOP_LATENCY_STEPS,
                      rng=None):
    """Distributed architecture: no central node. Every UAV that produced
    a track broadcasts it to every other UAV; each such broadcast is an
    independent peer-to-peer message that can be lost with probability
    comm_drop_probability. Each UAV then fuses *its own* track together
    with whatever peers' tracks actually arrived - so different UAVs can
    end up with different local estimates of the same object this step.

    Returns one row per (uav_id, fused object) - i.e. potentially several
    rows per object, one per UAV's local view of it - each carrying
    architecture/comm/response_time fields plus which UAV it's the local
    view for.
    """
    if fusion_mode not in FUSION_MODES:
        raise ValueError(f"Unknown fusion_mode: {fusion_mode!r} (expected one of {FUSION_MODES})")

    rng = rng or np.random.default_rng()
    all_sources = [_as_source(t, sensor_latency_steps, sensor_dropout_probability)
                   for t in radar_tracks]
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
            if rng.random() >= comm_drop_probability:  # broadcast survives
                received_from_peers.append(s)
                delivered_messages += 1
        local_sources = own + received_from_peers
        for row in _fuse_sources(local_sources, fusion_mode, cluster_distance):
            row["architecture"] = ARCHITECTURE_DISTRIBUTED
            row["local_uav_id"] = receiver_id
            row["comm_messages"] = attempted_messages
            row["response_time_steps"] = sensor_latency_steps + hop_latency_steps
            fused_rows.append(row)

    for row in fused_rows:
        row["comm_messages_delivered"] = delivered_messages
    return fused_rows


def fuse_step(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE,
              sensor_latency_steps=0, sensor_dropout_probability=0.0,
              architecture=ARCHITECTURE_CENTRALIZED, **architecture_kwargs):
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

    Defaults to "centralized", which reproduces this function's exact
    pre-architecture behavior (one row per object, no architecture/comm
    fields consumers didn't already expect) for every existing caller.
    """
    if architecture not in ARCHITECTURES:
        raise ValueError(f"Unknown architecture: {architecture!r} (expected one of {ARCHITECTURES})")

    if architecture == ARCHITECTURE_CENTRALIZED:
        return fuse_centralized(radar_tracks, fusion_mode, cluster_distance,
                                 sensor_latency_steps, sensor_dropout_probability,
                                 **architecture_kwargs)
    return fuse_distributed(radar_tracks, fusion_mode, cluster_distance,
                             sensor_latency_steps, sensor_dropout_probability,
                             **architecture_kwargs)


def build_fused_log(scenario_name, config, architecture=ARCHITECTURE_CENTRALIZED, seed=None):
    """Runs the radar model + tracker for one scenario, then fuses each
    step's tracks across all UAVs using the given fusion architecture.
    Returns the list of fused-estimate rows for that scenario - one row
    per fused object per step for "centralized", or one row per
    (uav_id, fused object) per step for "distributed" (see fuse_step /
    fuse_distributed).

    `seed` seeds the distributed architecture's communication-drop RNG so
    runs are reproducible; ignored for "centralized" (which has no random
    communication model today).
    """
    model = RadarLikeModel(config, scenario_name)
    detection_rows = model.run()
    dt = config["sim"]["dt"]
    track_rows = build_tracks(scenario_name, detection_rows, dt)

    fusion_mode = model.sim.scn.get(
        "fusion_mode", config.get("perception_errors", {}).get("fusion_mode", NO_FUSION))

    comm_cfg = config.get("communication", {})
    architecture_kwargs = {}
    if architecture == ARCHITECTURE_CENTRALIZED:
        architecture_kwargs["uplink_latency_steps"] = comm_cfg.get(
            "central_uplink_latency_steps", CENTRAL_UPLINK_LATENCY_STEPS)
        architecture_kwargs["downlink_latency_steps"] = comm_cfg.get(
            "central_downlink_latency_steps", CENTRAL_DOWNLINK_LATENCY_STEPS)
    else:
        architecture_kwargs["comm_drop_probability"] = comm_cfg.get(
            "comm_drop_probability", COMM_DROP_PROBABILITY)
        architecture_kwargs["hop_latency_steps"] = comm_cfg.get(
            "distributed_hop_latency_steps", DISTRIBUTED_HOP_LATENCY_STEPS)
        architecture_kwargs["rng"] = np.random.default_rng(seed)

    by_step = {}
    for row in track_rows:
        by_step.setdefault(row["time_step"], []).append(row)

    fused_rows = []
    for step in sorted(by_step):
        for f in fuse_step(by_step[step], fusion_mode,
                            sensor_latency_steps=model.radar_latency_steps,
                            sensor_dropout_probability=model.radar_dropout_probability,
                            architecture=architecture, **architecture_kwargs):
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
                "comm_messages": f.get("comm_messages"),
                "comm_messages_delivered": f.get("comm_messages_delivered"),
                "response_time_steps": f.get("response_time_steps"),
                "source_track_ids": ";".join(f["source_ids"]),
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
            rows = build_fused_log(name, config, architecture=architecture, seed=args.seed)
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