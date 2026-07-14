"""
fusion_model.py

Combines each UAV's radar track estimates (from radar_track_model.py) into
a single fused position estimate per real-world object, per time step.

Fusion never touches ground truth. Its only input is radar tracks that
have already been through radar_like_model.py + radar_track_model.py. That
mirrors the same rule simple_swarm_sim.py's existing single-obstacle
fuse_obstacle_detections() already follows - this module generalizes that
idea to per-object track fusion across all of the swarm's radars, instead
of a single hardcoded obstacle id.

Fusion modes (same three as everywhere else in this project):
  - "no_fusion"             - each UAV's own track stands alone
  - "naive_fusion"          - plain (unweighted) average across UAVs whose
                              tracks agree on the same object
  - "trust_weighted_fusion" - weighted average, weight = confidence *
                              track-status reliability

Before fusing, we first have to know which tracks - one per UAV's radar -
refer to the *same* real-world object. That's done with the same
nearest-neighbor idea radar_track_model.py already uses for
detection-to-track association, one level up: track-to-track.
"""

import argparse
import csv
import json
import math

from radar_like_model import RadarLikeModel
from radar_track_model import build_tracks

NO_FUSION = "no_fusion"
NAIVE_FUSION = "naive_fusion"
TRUST_WEIGHTED_FUSION = "trust_weighted_fusion"

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


def _as_source(track):
    """Normalizes a radar_track_model row into the shape fusion works with."""
    status = track.get("status", "confirmed")
    return {
        "source_id": track["track_id"],
        "radar_id": track["radar_id"],
        "x": track["est_x"],
        "y": track["est_y"],
        "confidence": track.get("confidence") or 0.5,
        "status": status,
        "status_weight": STATUS_RELIABILITY.get(status, 1.0),
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


def fuse_group(group, fusion_mode):
    """Fuses one cluster of same-object tracks (from different UAVs) into
    a single estimate."""
    if len(group) == 1:
        s = group[0]
        return {"x": s["x"], "y": s["y"], "confidence": s["confidence"],
                "num_sources": 1, "source_ids": [s["source_id"]]}

    if fusion_mode == TRUST_WEIGHTED_FUSION:
        weights = [s["confidence"] * s["status_weight"] for s in group]
    else:  # naive_fusion: every UAV's track counts equally, confidence ignored
        weights = [1.0 for _ in group]

    total_w = sum(weights)
    if total_w <= 1e-9:
        weights = [1.0 for _ in group]
        total_w = len(group)

    fx = sum(s["x"] * w for s, w in zip(group, weights)) / total_w
    fy = sum(s["y"] * w for s, w in zip(group, weights)) / total_w
    avg_conf = sum(s["confidence"] for s in group) / len(group)
    # More independent UAVs agreeing raises fused confidence a bit - same
    # idea as simple_swarm_sim.py's existing fuse_obstacle_detections.
    fused_conf = min(1.0, avg_conf + 0.08 * (len(group) - 1))

    return {"x": fx, "y": fy, "confidence": round(fused_conf, 3),
            "num_sources": len(group), "source_ids": [s["source_id"] for s in group]}


def fuse_step(radar_tracks, fusion_mode, cluster_distance=CLUSTER_DISTANCE):
    """Fuses one time step's worth of radar tracks - one list across all of
    the swarm's UAVs, already sensor output, never ground truth - into
    per-object fused estimates."""
    sources = [_as_source(t) for t in radar_tracks]

    if not sources:
        return []

    if fusion_mode == NO_FUSION:
        # Nothing gets combined across UAVs - every UAV's track stands on
        # its own, exactly what "no_fusion" scenarios elsewhere in this
        # project mean.
        return [{"x": s["x"], "y": s["y"], "confidence": s["confidence"],
                 "num_sources": 1, "source_ids": [s["source_id"]]} for s in sources]

    clusters = _cluster(sources, cluster_distance)
    return [fuse_group(g, fusion_mode) for g in clusters]


def build_fused_log(scenario_name, config):
    """Runs the radar model + tracker for one scenario, then fuses each
    step's tracks across all UAVs. Returns the list of fused-estimate rows
    for that scenario (one row per fused object per step)."""
    model = RadarLikeModel(config, scenario_name)
    detection_rows = model.run()
    dt = config["sim"]["dt"]
    track_rows = build_tracks(scenario_name, detection_rows, dt)

    fusion_mode = model.sim.scn.get(
        "fusion_mode", config.get("perception_errors", {}).get("fusion_mode", NO_FUSION))

    by_step = {}
    for row in track_rows:
        by_step.setdefault(row["time_step"], []).append(row)

    fused_rows = []
    for step in sorted(by_step):
        for f in fuse_step(by_step[step], fusion_mode):
            fused_rows.append({
                "scenario": scenario_name,
                "time_step": step,
                "fusion_mode": fusion_mode,
                "fused_x": round(f["x"], 4),
                "fused_y": round(f["y"], 4),
                "fused_confidence": round(f["confidence"], 4),
                "num_sources": f["num_sources"],
                "source_track_ids": ";".join(f["source_ids"]),
            })
    return fused_rows


def main():
    parser = argparse.ArgumentParser(
        description="Fuse radar tracks from all UAVs into per-object estimates")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default="logs/fused_track_log.csv")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        rows = build_fused_log(name, config)
        all_rows.extend(rows)
        print(f"{name}: {len(rows)} fused rows")

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