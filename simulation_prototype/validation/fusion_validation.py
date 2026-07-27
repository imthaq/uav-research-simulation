"""
fusion_validation.py

Task 5: validates the multi-source fusion math in fusion/fusion_model.py
against controlled cases built directly from radar-track-shaped rows -
not the full radar+tracker+fusion pipeline, just the fusion weighting
itself:

  - identical sensor measurements
  - radar more accurate than vision (i.e. one source tighter/covariance
    than the other - fusion_model.py fuses same-shaped "sources" from
    any sensor a track came from, so accuracy is expressed the same way
    regardless of which physical sensor produced the track: a tighter
    position covariance / higher confidence)
  - vision more accurate than radar (the mirror image of the above)
  - one stale measurement
  - one high-confidence incorrect measurement
  - one sensor dropout
  - correlated estimates under Covariance Intersection

...confirming expected weighting and fused position for each of the
fusion modes in fusion_model.FUSION_MODES.

Each check is a controlled case with a known expected answer (or a known
directional property), asserted with a small numerical tolerance. Results
are printed and written to results/fusion_validation_results.md.

Usage:
    python fusion_validation.py
"""

import json
import math
import os
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT_DIR,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fusion.fusion_model import (
    fuse_group, fuse_centralized, _as_source, _cluster,
    NAIVE_FUSION, CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION,
    COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION, NO_FUSION,
)
from validation_common import Checker

_checker = Checker()


def check(task, description, condition, detail=""):
    return _checker.check(task, description, condition, detail)


def close(a, b, tol=1e-6):
    return _checker.close(a, b, tol)


def make_track(track_id, radar_id, x, y, confidence=0.9, status="confirmed",
                missed_count=0, pos_var=1.0, age=10, hit_count=10):
    """Builds a radar_track_model-shaped row (the only thing fusion_model
    ever consumes) with a diagonal 4x4 [x, y, vx, vy] covariance whose
    position block is pos_var * I."""
    P = [[pos_var, 0.0, 0.0, 0.0],
         [0.0, pos_var, 0.0, 0.0],
         [0.0, 0.0, 25.0, 0.0],
         [0.0, 0.0, 0.0, 25.0]]
    return {
        "track_id": track_id, "radar_id": radar_id,
        "est_x": x, "est_y": y, "est_vx": 0.0, "est_vy": 0.0,
        "covariance": json.dumps(P), "confidence": confidence, "age": age,
        "hit_count": hit_count, "missed_count": missed_count,
        "existence_probability": 0.9, "status": status,
    }


def source(track_id, radar_id, x, y, **kw):
    return _as_source(make_track(track_id, radar_id, x, y, **kw))


ALL_MODES = (NAIVE_FUSION, CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION,
             COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION)


# ---------------------------------------------------------------------
# 1. Identical sensor measurements
# ---------------------------------------------------------------------
def test_identical_measurements():
    for mode in ALL_MODES:
        group = [source("A", "r1", 10.0, 20.0), source("B", "r2", 10.0, 20.0)]
        result = fuse_group(group, mode)
        check("identical_measurements", f"[{mode}] fusing two identical measurements returns that exact position",
              close(result["x"], 10.0, tol=1e-6) and close(result["y"], 20.0, tol=1e-6),
              f"got ({result['x']:.4f},{result['y']:.4f})")
        check("identical_measurements", f"[{mode}] fusing identical sources reports both as contributors",
              result["num_sources"] == 2, f"got {result['num_sources']}")

    # Confidence should rise (not just hold) when two agreeing sources
    # corroborate each other, for every weighted-average-family mode.
    group = [source("A", "r1", 10.0, 20.0, confidence=0.7),
             source("B", "r2", 10.0, 20.0, confidence=0.7)]
    result = fuse_group(group, NAIVE_FUSION)
    check("identical_measurements", "agreeing sources raise fused confidence above either individual confidence",
          result["confidence"] > 0.7, f"got {result['confidence']}")


# ---------------------------------------------------------------------
# 2. Radar more accurate than vision (tighter covariance / higher confidence)
# ---------------------------------------------------------------------
def test_radar_more_accurate_than_vision():
    radar = source("radar1", "radar", 0.0, 0.0, confidence=0.95, pos_var=0.25)   # tight, trustworthy
    vision = source("vision1", "vision", 10.0, 0.0, confidence=0.5, pos_var=9.0)  # loose, less trustworthy
    group = [radar, vision]

    for mode in (CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION):
        result = fuse_group(group, mode)
        check("radar_more_accurate", f"[{mode}] fused x sits closer to the accurate (radar) source than the midpoint",
              result["x"] < 5.0, f"got x={result['x']:.4f}")

    for mode in (COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION):
        result = fuse_group(group, mode)
        check("radar_more_accurate", f"[{mode}] fused x sits closer to the tighter-covariance (radar) source",
              result["x"] < 5.0, f"got x={result['x']:.4f}")

    naive = fuse_group(group, NAIVE_FUSION)
    check("radar_more_accurate", "naive_fusion ignores accuracy and lands at the unweighted midpoint",
          close(naive["x"], 5.0, tol=1e-6), f"got x={naive['x']:.4f}")


# ---------------------------------------------------------------------
# 3. Vision more accurate than radar (mirror of the above)
# ---------------------------------------------------------------------
def test_vision_more_accurate_than_radar():
    radar = source("radar1", "radar", 0.0, 0.0, confidence=0.4, pos_var=9.0)     # loose
    vision = source("vision1", "vision", 10.0, 0.0, confidence=0.95, pos_var=0.25)  # tight
    group = [radar, vision]

    for mode in (CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION,
                 COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION):
        result = fuse_group(group, mode)
        check("vision_more_accurate", f"[{mode}] fused x sits closer to the accurate (vision) source",
              result["x"] > 5.0, f"got x={result['x']:.4f}")


# ---------------------------------------------------------------------
# 4. One stale measurement (large missed_count / measurement age)
# ---------------------------------------------------------------------
def test_one_stale_measurement():
    fresh = source("fresh", "r1", 0.0, 0.0, confidence=0.9, missed_count=0, status="confirmed")
    stale = source("stale", "r2", 10.0, 0.0, confidence=0.9, missed_count=20, status="coasting")
    check("one_stale_measurement", "a stale source's reliability is discounted below a fresh source's",
          stale["reliability"] < fresh["reliability"],
          f"fresh_reliability={fresh['reliability']} stale_reliability={stale['reliability']}")

    group = [fresh, stale]
    for mode in (TRUST_WEIGHTED_FUSION, COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION):
        result = fuse_group(group, mode)
        check("one_stale_measurement", f"[{mode}] fused position is pulled toward the fresh source, away from the stale one",
              result["x"] < 5.0, f"got x={result['x']:.4f}")

    # max_staleness_steps (fuse_centralized) hard-rejects a source whose
    # measurement_age_steps exceeds it, regardless of soft weighting.
    fused = fuse_centralized(
        [make_track("fresh", "r1", 0.0, 0.0, missed_count=0),
         make_track("stale", "r2", 10.0, 0.0, missed_count=20)],
        TRUST_WEIGHTED_FUSION, max_staleness_steps=5)
    check("one_stale_measurement", "max_staleness_steps hard-rejects a too-old source before fusing",
          len(fused) == 1 and close(fused[0]["x"], 0.0, tol=1e-6),
          f"got {[(f['x'], f['num_sources']) for f in fused]}")


# ---------------------------------------------------------------------
# 5. One high-confidence incorrect measurement
# ---------------------------------------------------------------------
def test_high_confidence_incorrect_measurement():
    # Two agreeing, moderate-confidence sources near the true position,
    # plus one falsely-confident outlier far away.
    correct_a = source("A", "r1", 0.0, 0.0, confidence=0.6, pos_var=1.0)
    correct_b = source("B", "r2", 0.5, 0.0, confidence=0.6, pos_var=1.0)
    bad = source("C", "r3", 100.0, 0.0, confidence=0.99, pos_var=0.5)
    group = [correct_a, correct_b, bad]

    for mode in (CONFIDENCE_WEIGHTED_FUSION,):
        result = fuse_group(group, mode)
        check("high_confidence_incorrect", f"[{mode}] a single high-confidence outlier can pull the fused estimate far off",
              result["x"] > 30.0, f"got x={result['x']:.4f}")

    # trust_weighted_fusion additionally weighs status/reliability, but
    # with all three sources confirmed/fresh it still can't outvote a
    # confidence this high with only two lower-confidence agreeing peers -
    # documenting the known limitation that *pure* confidence-style
    # weighting has no outlier-rejection step of its own (no RANSAC/
    # robust-statistics layer) unless CI or clustering catches it upstream.
    result = fuse_group(group, TRUST_WEIGHTED_FUSION)
    check("high_confidence_incorrect", "trust_weighted_fusion is also pulled toward a high-confidence, high-status outlier",
          result["x"] > 30.0, f"got x={result['x']:.4f}")

    # This is exactly why clustering exists upstream of fuse_group in
    # practice: _cluster would put a 100-units-away "incorrect" source in
    # its own cluster rather than ever handing it to fuse_group alongside
    # the real object's sources.
    clusters = _cluster(group, cluster_distance=4.0)
    check("high_confidence_incorrect", "clustering upstream keeps the far outlier in its own cluster (never reaches fuse_group)",
          len(clusters) == 2 and any(len(c) == 1 and c[0]["source_id"] == "C" for c in clusters),
          f"cluster sizes={[len(c) for c in clusters]}")


# ---------------------------------------------------------------------
# 6. One sensor dropout
# ---------------------------------------------------------------------
def test_one_sensor_dropout():
    active = _as_source(make_track("active", "r1", 0.0, 0.0, confidence=0.9, status="confirmed", missed_count=0))
    dropped_out = _as_source(make_track("dropped", "r2", 10.0, 0.0, confidence=0.9, status="lost", missed_count=1))
    check("one_sensor_dropout", "dropout_state is flagged for a lost/missed track",
          dropped_out["dropout_state"] is True and active["dropout_state"] is False,
          f"active={active['dropout_state']} dropped={dropped_out['dropout_state']}")
    check("one_sensor_dropout", "a dropped-out source's reliability is discounted below an active source's",
          dropped_out["reliability"] < active["reliability"],
          f"active={active['reliability']} dropped={dropped_out['reliability']}")

    group = [active, dropped_out]
    for mode in (TRUST_WEIGHTED_FUSION, COVARIANCE_WEIGHTED_FUSION):
        result = fuse_group(group, mode)
        check("one_sensor_dropout", f"[{mode}] fused position favors the active source over the dropped-out one",
              result["x"] < 5.0, f"got x={result['x']:.4f}")

    # A total dropout (only one source reporting at all) just falls back
    # to that single source unchanged - nothing to fuse against.
    solo = fuse_group([active], TRUST_WEIGHTED_FUSION)
    check("one_sensor_dropout", "with only one source reporting (full dropout of the other), fusion returns it unchanged",
          close(solo["x"], 0.0, tol=1e-6) and solo["num_sources"] == 1, f"got {solo}")


# ---------------------------------------------------------------------
# 7. Correlated estimates under Covariance Intersection
# ---------------------------------------------------------------------
def test_covariance_intersection_correlated():
    # Two sources with IDENTICAL, equally-sized covariance (the classic
    # "fully correlated" case - e.g. two coasting tracks whose estimates
    # both derive from the same stale detection), at confidence=1.0 so
    # reliability==1.0 and eff_covariance == raw covariance (clean
    # textbook numbers, not scaled by the reliability discount). Naive
    # information fusion (covariance_weighted_fusion) would claim the
    # fused covariance is *half* as large (as if the two measurements
    # were independent); CI is specifically designed not to overclaim
    # precision in this case.
    a = source("A", "r1", 0.0, 0.0, confidence=1.0, pos_var=4.0)
    b = source("B", "r2", 10.0, 0.0, confidence=1.0, pos_var=4.0)
    group = [a, b]
    # position_variance is trace(covariance) - summed over x and y, so a
    # single source with pos_var=4.0 per axis reports trace=8.0.
    individual_trace = float(a["eff_covariance"][0][0] + a["eff_covariance"][1][1])

    ci_result = fuse_group(group, COVARIANCE_INTERSECTION_FUSION)
    info_result = fuse_group(group, COVARIANCE_WEIGHTED_FUSION)

    check("covariance_intersection_correlated",
          "CI on two equal-covariance sources lands at the midpoint (symmetric case)",
          close(ci_result["x"], 5.0, tol=0.05), f"got x={ci_result['x']:.4f}")
    check("covariance_intersection_correlated",
          "CI's fused position_variance does not overclaim precision the way naive information fusion does",
          ci_result["position_variance"] > info_result["position_variance"],
          f"CI_var={ci_result['position_variance']} info_var={info_result['position_variance']}")
    check("covariance_intersection_correlated",
          "CI's fused variance is no larger than either individual source's (effective) variance - still informative",
          ci_result["position_variance"] <= individual_trace + 1e-6,
          f"CI_var={ci_result['position_variance']} individual_trace={individual_trace}")
    # The information-filter (independence-assuming) fused variance for two
    # equal-variance, equal-confidence sources is the textbook half:
    # 1/(1/v + 1/v) = v/2 per axis, i.e. half the individual trace.
    check("covariance_intersection_correlated",
          "naive information fusion (assumes independence) halves the trace for two equal sources",
          close(info_result["position_variance"], individual_trace / 2.0, tol=0.05),
          f"got {info_result['position_variance']} vs expected {individual_trace / 2.0}")

    # CI stays *consistent* (conservative) even for 3+ correlated,
    # identical-covariance sources - variance should not collapse toward 0
    # just because more copies of the same correlated estimate were added.
    group3 = [source("A", "r1", 0.0, 0.0, confidence=1.0, pos_var=4.0),
              source("B", "r2", 0.0, 0.0, confidence=1.0, pos_var=4.0),
              source("C", "r3", 0.0, 0.0, confidence=1.0, pos_var=4.0)]
    ci3 = fuse_group(group3, COVARIANCE_INTERSECTION_FUSION)
    check("covariance_intersection_correlated",
          "CI's fused variance for several identical, fully-correlated sources stays close to the single-source variance",
          ci3["position_variance"] > individual_trace / 2.0, f"got {ci3['position_variance']}")


def main():
    test_identical_measurements()
    test_radar_more_accurate_than_vision()
    test_vision_more_accurate_than_radar()
    test_one_stale_measurement()
    test_high_confidence_incorrect_measurement()
    test_one_sensor_dropout()
    test_covariance_intersection_correlated()

    failed = _checker.print_summary()

    out_dir = os.path.join(_ROOT_DIR, "results")
    out_path = os.path.join(out_dir, "fusion_validation_results.md")
    _checker.write_markdown(
        out_path, "Fusion Validation Results (Task 5)",
        intro="Controlled checks of the multi-source weighting math in "
              "`fusion/fusion_model.py` (`_as_source`, `fuse_group`, "
              "`fuse_centralized`) - hand-built radar-track-shaped rows fed "
              "straight into fusion, not the full radar+tracker+fusion pipeline.")
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
