"""Task 19: combined-fault scenarios.
"""
import json
import os
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from experiments.failure_envelope import run_point, classify, SEEDS_PER_POINT

OUT_PATH = os.path.join(_ROOT_DIR, "combined_fault_results.md")

CROSSING_TARGETS = [
    {"id": "obstacle_0", "kind": "static_obstacle", "x": 50.0, "y": 50.0, "radius": 5.0},
    {"id": "target_0", "kind": "moving_target", "x": 10.0, "y": 90.0,
     "velocity": [0.9, -0.9], "bounce": True},
    {"id": "target_1", "kind": "moving_target", "x": 10.0, "y": 10.0,
     "velocity": [0.9, 0.9], "bounce": True},
]

SCENARIOS = [
    ("low_P_D + high_clutter",
     {"radar_detection_probability": 0.3, "radar_clutter_density": 2.0,
      "radar_false_alarm_probability": 0.3}, {}),
    ("radar_dropout + communication_loss",
     {"radar_dropout_probability": 0.25}, {"packet_loss_prob": 0.5}),
    ("registration_error + overconfident_vision",
     {"radar_confidence_bias": 0.4}, {"registration_bias": 2.0}),
    ("radar_ghost_returns + target_crossing",
     {"radar_false_alarm_probability": 0.3, "radar_clutter_density": 1.5,
      "entities": CROSSING_TARGETS}, {}),
    ("latency + rapidly_moving_obstacle",
     {"radar_latency_steps": 8,
      "entities": [{"id": "obstacle_0", "kind": "moving_obstacle", "x": 95.0, "y": 10.0,
                    "radius": 4.0, "velocity": [-3.0, 0.0], "bounce": True}]}, {}),
    ("sensor_failure + centralized_fusion_unavailable",
     {"fusion_mode": "no_fusion"}, {"num_faulty_uavs": 2, "fault_duration_steps": 300}),
    ("corrupted_confidence + packet_delay",
     {"radar_confidence_error": 0.35}, {"comm_delay_steps": 5}),
    ("two_faulty_UAV_perception_sources",
     {}, {"num_faulty_uavs": 2, "fault_duration_steps": 300}),
]


def main():
    config = json.load(open(os.path.join(_ROOT_DIR, "simulation_config.json")))
    seeds = list(range(1, SEEDS_PER_POINT + 1))

    baseline_runs = run_point(config, {}, {}, seeds)
    baseline_near_miss = sum(r["near_miss_count"] for r in baseline_runs) / len(baseline_runs)
    formation_vals = [r["avg_formation_error"] for r in baseline_runs if r["avg_formation_error"]]
    baseline_formation_error = sum(formation_vals) / len(formation_vals) if formation_vals else None

    lines = [
        "# Combined-fault scenario results\n",
        f"Controller: `5_dynamic_trust_handoff`, {SEEDS_PER_POINT} seeds/scenario, "
        "classified against the same baseline run and thresholds as `swarm_failure_envelope.csv` "
        "(Task 18) - see `experiments/combined_fault_scenarios.py`.\n",
        "| Scenario | Mission success | Collisions (mean) | Near-misses (mean) | "
        "Formation error (mean) | Classification |",
        "|---|---|---|---|---|---|",
    ]
    for name, scn_overrides, stress_kwargs in SCENARIOS:
        runs = run_point(config, scn_overrides, stress_kwargs, seeds)
        classes = [classify(r, baseline_near_miss, baseline_formation_error) for r in runs]
        worst = max(classes, key=["SAFE", "DEGRADED BUT FUNCTIONAL",
                                   "MISSION FAILURE", "SAFETY FAILURE"].index)
        n = len(runs)
        success_rate = sum(r["mission_success"] for r in runs) / n
        collisions = sum(r["collision_count"] for r in runs) / n
        near_misses = sum(r["near_miss_count"] for r in runs) / n
        formation = sum(r["avg_formation_error"] or 0 for r in runs) / n
        lines.append(f"| {name} | {success_rate:.2f} | {collisions:.2f} | {near_misses:.1f} | "
                      f"{formation:.2f} | **{worst}** |")
        print(f"{name}: {worst}")

    lines.append(f"\nBaseline reference: near-miss count {baseline_near_miss:.1f}, "
                 f"formation error {baseline_formation_error:.3f} "
                 "(the 1.5x-of-baseline thresholds Task 18's `classify()` uses).\n")

    with open(OUT_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nwrote {OUT_PATH}")


if __name__ == "__main__":
    main()
