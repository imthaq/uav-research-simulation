"""
controller_comparison.py

Task 17: dependability-controller comparison. Runs the five controllers
in dependability_controllers.CONTROLLERS across a set of scenarios/trials
and reports:

    collision risk (collision_count), near misses, mission success,
    formation error, mission delay, unnecessary HOLD time,
    unnecessary avoidance, correct/failed handoff count, recovery time

Same CLI/CSV-writing shape as ablation_experiments.py, next to it.

mission_delay_s: this project has no standalone "reference/direct-flight
time" metric to diff against, so mission_delay_s is reported relative to
controller 1 (fixed_margin) on the *same scenario+seed* - extra wall-clock
time (steps_run * dt) each more-cautious controller took to reach the same
outcome. Controller 1's own mission_delay_s is always 0.0 by construction.
"""
import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from dependability_controllers import CONTROLLERS, run_any_controller

# Which scenario each controller is compared on by default. Controller 5
# needs the full radar/track/fusion pipeline's config keys (radar_*,
# faulty_uav_id, ...), so it gets a scenario that already has them; 1-4
# run on the simplified path and reuse the Task 13 safety-margin scenario
# already tuned for a meaningfully imperfect (not trivial, not impossible)
# run.
DEFAULT_SCENARIO_FOR = {
    "1_fixed_margin": "safety_margin_quality_monitor",
    "2_uncertainty_aware": "safety_margin_quality_monitor",
    "3_uncertainty_aware_abstention": "safety_margin_quality_monitor",
    "4_uncertainty_aware_handoff": "safety_margin_quality_monitor",
    "5_dynamic_trust_handoff": "faulty_sensor_trust_weighted_fusion_dynamic",
}

FIELDNAMES = [
    "controller", "scenario", "trial", "seed",
    "mission_success", "collision_count", "near_miss_count",
    "avg_formation_error", "mission_delay_s",
    "unnecessary_avoidance_count", "unnecessary_hold_steps",
    "correct_handoff_count", "failed_handoff_count", "recovery_time_s",
    "avg_response_time_s", "steps_run",
]


def run_comparison(config, controllers=None, trials=5, base_seed=42):
    controllers = controllers or list(CONTROLLERS)
    rows_by_key = {}  # (scenario, seed) -> {controller: metrics}, to compute mission_delay_s

    for controller_name in controllers:
        scenario = DEFAULT_SCENARIO_FOR[controller_name]
        for trial in range(1, trials + 1):
            seed = base_seed + (trial - 1)
            try:
                m = run_any_controller(controller_name, config, scenario, seed)
            except Exception as e:
                print(f"Error in {controller_name} / {scenario} / trial {trial}: {e}")
                continue
            rows_by_key.setdefault((scenario, seed), {})[controller_name] = m

    rows = []
    for (scenario, seed), by_controller in rows_by_key.items():
        baseline = by_controller.get("1_fixed_margin")
        baseline_time = (baseline["steps_run"] * baseline.get("dt", 0.2)
                          if baseline else None)
        for controller_name, m in by_controller.items():
            dt = 0.2  # sim.dt not carried in metrics dict; matches simulation_config.json "sim.dt"
            this_time = m["steps_run"] * dt
            mission_delay_s = (round(this_time - baseline_time, 3)
                                if baseline_time is not None else None)
            unnecessary_hold_steps = (m.get("hold_count", 0)
                                       + m.get("abstention_hold_steps", 0)
                                       + m.get("handoff_hold_steps", 0))
            rows.append({
                "controller": controller_name,
                "scenario": scenario,
                "trial": seed - 42 + 1,
                "seed": seed,
                "mission_success": "Yes" if m["mission_success"] else "No",
                "collision_count": m["collision_count"],
                "near_miss_count": m["near_miss_count"],
                "avg_formation_error": m["avg_formation_error"],
                "mission_delay_s": mission_delay_s,
                "unnecessary_avoidance_count": m["unnecessary_avoidance_count"],
                "unnecessary_hold_steps": unnecessary_hold_steps,
                "correct_handoff_count": m.get("correct_handoff_count"),
                "failed_handoff_count": m.get("failed_handoff_count"),
                "recovery_time_s": m.get("recovery_time_s"),
                "avg_response_time_s": m["avg_response_time_s"],
                "steps_run": m["steps_run"],
            })
    return rows


def main():
    parser = argparse.ArgumentParser(description="Task 17: dependability-controller comparison")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default=os.path.join(_ROOT_DIR, "results", "controller_comparison.csv"))
    args = parser.parse_args()

    with open(args.config) as f:
        base_config = json.load(f)

    start = datetime.now()
    rows = run_comparison(base_config, trials=args.trials, base_seed=args.seed)
    elapsed = (datetime.now() - start).total_seconds()

    out_dir = os.path.dirname(args.output) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Controller comparison complete: {len(rows)} runs in {elapsed:.1f}s")
    print(f"Results written to: {args.output}")

    groups = defaultdict(list)
    for r in rows:
        groups[r["controller"]].append(r)

    print("\n=== Controller Summary ===")
    for name in CONTROLLERS:
        rs = groups.get(name, [])
        if not rs:
            continue
        success = sum(1 for r in rs if r["mission_success"] == "Yes")
        avg_collision = sum(r["collision_count"] for r in rs) / len(rs)
        avg_near_miss = sum(r["near_miss_count"] for r in rs) / len(rs)
        avg_delay = sum(r["mission_delay_s"] for r in rs if r["mission_delay_s"] is not None) / max(
            1, len([r for r in rs if r["mission_delay_s"] is not None]))
        avg_hold = sum(r["unnecessary_hold_steps"] for r in rs) / len(rs)
        print(f"  {name:32} success={success}/{len(rs)} avg_collision={avg_collision:.2f} "
              f"avg_near_miss={avg_near_miss:.2f} avg_delay_s={avg_delay:.2f} avg_hold={avg_hold:.1f}")


if __name__ == "__main__":
    sys.exit(main())
