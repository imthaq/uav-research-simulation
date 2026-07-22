"""Task 20: scalability experiments.

Same controller-5 pipeline as Tasks 18/19 (run_stress_pipeline,
instrument=True), just varying swarm size instead of a fault parameter.
start_positions only ships 4 entries in simulation_config.json, so this
generates a grid of them for larger swarms - everything else (obstacle,
world bounds, formation spacing) stays at config defaults.

Metrics not already on sim._metrics()/instrument's output:
  - communication_load   - message_count: total per-UAV track messages
    handed to fuse_step over the whole run (grows with num_uavs*steps).
  - fusion_update_time    - mean wall-clock time per fuse_step call (ms).
  - centralized_fusion_bottleneck - total wall-clock time fuse_step spent
    across the whole run (ms) - this is what would start to dominate
    runtime if centralized fusion doesn't scale.
  - distributed_fusion_consistency - mean spread (std, meters) across
    UAVs' own pre-fusion track estimates of the shared obstacle each
    step; this project has no separate distributed-architecture mission
    pipeline to compare against (fuse_step here is always the
    centralized-style single fusion pass - see fusion/fusion_model.py's
    fuse_centralized/fuse_distributed split), so this is the proxy: how
    much the swarm's raw, unfused per-UAV perception already disagrees
    before any fusion algorithm gets to reconcile it, at each size.
"""
import copy
import csv
import json
import os
import sys
import time

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from experiments.failure_envelope import run_stress_pipeline, SEEDS_PER_POINT

OUT_PATH = os.path.join(_ROOT_DIR, "swarm_scalability_results.csv")
SWARM_SIZES = [3, 5, 10, 20]


def gen_start_positions(n, cols=5, spacing=4.0, origin=5.0):
    return [[origin + (i % cols) * spacing, origin + (i // cols) * spacing] for i in range(n)]


def main():
    config = json.load(open(os.path.join(_ROOT_DIR, "simulation_config.json")))
    seeds = list(range(1, SEEDS_PER_POINT + 1))
    rows = []

    for n in SWARM_SIZES:
        cfg = copy.deepcopy(config)
        cfg["swarm"]["num_uavs"] = n
        cfg["swarm"]["start_positions"] = gen_start_positions(n)

        run_metrics, runtimes = [], []
        for seed in seeds:
            t0 = time.perf_counter()
            m = run_stress_pipeline(cfg, "baseline", seed, instrument=True)
            runtimes.append(time.perf_counter() - t0)
            run_metrics.append(m)

        k = len(run_metrics)
        total_fusion_ms = [m["fusion_update_time_ms"] * m["steps_run"] for m in run_metrics]
        consistency_vals = [m["distributed_consistency_std"] for m in run_metrics
                             if m["distributed_consistency_std"] is not None]
        rows.append({
            "num_uavs": n,
            "runtime_s": round(sum(runtimes) / k, 3),
            "communication_load_messages": round(sum(m["message_count"] for m in run_metrics) / k, 1),
            "fusion_update_time_ms_per_call": round(
                sum(m["fusion_update_time_ms"] for m in run_metrics) / k, 4),
            "number_of_tracks": round(sum(m["tracks_created"] for m in run_metrics) / k, 1),
            "collision_risk_mean": round(sum(m["collision_count"] for m in run_metrics) / k, 3),
            "formation_error_mean": round(sum(m["avg_formation_error"] or 0 for m in run_metrics) / k, 3),
            "mission_success_rate": round(sum(m["mission_success"] for m in run_metrics) / k, 3),
            "centralized_fusion_bottleneck_ms_total": round(sum(total_fusion_ms) / k, 3),
            "distributed_fusion_consistency_std": (
                round(sum(consistency_vals) / len(consistency_vals), 4) if consistency_vals else None),
        })
        print(f"num_uavs={n}: runtime={rows[-1]['runtime_s']}s "
              f"mission_success_rate={rows[-1]['mission_success_rate']}")

    fieldnames = list(rows[0].keys())
    with open(OUT_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {len(rows)} rows to {OUT_PATH}")


if __name__ == "__main__":
    main()
