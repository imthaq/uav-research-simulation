import csv
import json
import os
import sys

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from build_experiment_matrix import (
    CORE_SCENARIOS, _radar_configuration_summary, _communication_condition)
from dependability_controllers import CONTROLLERS
from experiments.combined_fault_scenarios import SCENARIOS as COMBINED_FAULT_SCENARIOS
from experiments.scalability_experiments import SWARM_SIZES

OUT_PATH = os.path.join(_ROOT_DIR, "logs/final_dependability_experiment_matrix.csv")

# Task 18 axes that actually reached DEGRADED/FAILURE in
# swarm_failure_envelope.csv - confirmed at full trial count here.
# (scenario_id, scn_overrides)
FAILURE_ENVELOPE_CONFIRMATION = [
    ("envelope_low_P_D", {"radar_detection_probability": 0.3}),
    ("envelope_high_P_FA", {"radar_false_alarm_probability": 0.4}),
    ("envelope_high_clutter", {"radar_clutter_density": 3.0}),
    ("envelope_high_covariance", {"radar_range_noise_std": 3.0}),
    ("envelope_high_calibration_error", {"radar_confidence_error": 0.35}),
    ("envelope_high_dropout", {"radar_dropout_probability": 0.4}),
]

FIELDNAMES = [
    "scenario_id", "seed_range", "uav_count", "target_count", "radar_mode",
    "radar_parameters", "calibration_mode", "fusion_mode", "controller_mode",
    "handoff_mode", "communication_condition", "num_trials", "output_directory",
]


def _target_count(scn):
    entities = scn.get("entities")
    if entities is None:
        return 0
    return sum(1 for e in entities if e.get("kind") == "moving_target")


def _seed_range(base_seed, trials):
    return f"{base_seed}-{base_seed + trials - 1}"


def build_matrix(config):
    scenarios = config["scenarios"]
    base_seed = config["sim"]["seed"]
    comm_defaults = config.get("communication", {})
    default_uav_count = config["swarm"]["num_uavs"]
    rows = []

    # --- A. controller comparison: baseline scenario, all 5 controllers,
    # 50 trials (the most important comparison - Task 22's own "50 trials
    # when feasible" rule). ---
    baseline_scn = scenarios["baseline"]
    for controller_name, spec in CONTROLLERS.items():
        trials = 50
        rows.append({
            "scenario_id": f"controller_comparison__baseline__{controller_name}",
            "seed_range": _seed_range(base_seed, trials),
            "uav_count": default_uav_count,
            "target_count": _target_count(baseline_scn),
            "radar_mode": baseline_scn.get("radar_mode", "normal"),
            "radar_parameters": _radar_configuration_summary(baseline_scn),
            "calibration_mode": spec["safety_margin_mode"],
            "fusion_mode": "trust_weighted_fusion" if spec["dynamic_trust"]
                           else baseline_scn.get("fusion_mode", "no_fusion"),
            "controller_mode": controller_name,
            "handoff_mode": "on" if spec["handoff"] else "off",
            "communication_condition": _communication_condition(baseline_scn, comm_defaults),
            "num_trials": trials,
            "output_directory": f"results/final/controller_comparison/{controller_name}/",
        })

    # --- B. failure-envelope confirmation: controller 5, 20 trials. ---
    controller_name = "5_dynamic_trust_handoff"
    spec = CONTROLLERS[controller_name]
    for scenario_id, overrides in FAILURE_ENVELOPE_CONFIRMATION:
        scn = dict(baseline_scn)
        scn.update(overrides)
        trials = 20
        rows.append({
            "scenario_id": scenario_id,
            "seed_range": _seed_range(base_seed, trials),
            "uav_count": default_uav_count,
            "target_count": _target_count(scn),
            "radar_mode": scn.get("radar_mode", "normal"),
            "radar_parameters": _radar_configuration_summary(scn),
            "calibration_mode": spec["safety_margin_mode"],
            "fusion_mode": "trust_weighted_fusion",
            "controller_mode": controller_name,
            "handoff_mode": "on",
            "communication_condition": _communication_condition(scn, comm_defaults),
            "num_trials": trials,
            "output_directory": f"results/final/failure_envelope/{scenario_id}/",
        })

    # --- C. combined-fault confirmation: controller 5, 20 trials. ---
    for name, scn_overrides, _stress_kwargs in COMBINED_FAULT_SCENARIOS:
        scn = dict(baseline_scn)
        scn.update(scn_overrides)
        scenario_id = "combined__" + name.replace(" + ", "__").replace(" ", "_")
        trials = 20
        rows.append({
            "scenario_id": scenario_id,
            "seed_range": _seed_range(base_seed, trials),
            "uav_count": default_uav_count,
            "target_count": _target_count(scn),
            "radar_mode": scn.get("radar_mode", "normal"),
            "radar_parameters": _radar_configuration_summary(scn),
            "calibration_mode": spec["safety_margin_mode"],
            "fusion_mode": scn.get("fusion_mode", "trust_weighted_fusion"),
            "controller_mode": controller_name,
            "handoff_mode": "on",
            "communication_condition": _communication_condition(scn, comm_defaults),
            "num_trials": trials,
            "output_directory": f"results/final/combined_fault/{scenario_id}/",
        })

    # --- D. scalability: controller 5, baseline scenario, 20 trials. ---
    for n in SWARM_SIZES:
        trials = 20
        scenario_id = f"scalability__{n}_uavs"
        rows.append({
            "scenario_id": scenario_id,
            "seed_range": _seed_range(base_seed, trials),
            "uav_count": n,
            "target_count": _target_count(baseline_scn),
            "radar_mode": baseline_scn.get("radar_mode", "normal"),
            "radar_parameters": _radar_configuration_summary(baseline_scn),
            "calibration_mode": spec["safety_margin_mode"],
            "fusion_mode": "trust_weighted_fusion",
            "controller_mode": controller_name,
            "handoff_mode": "on",
            "communication_condition": _communication_condition(baseline_scn, comm_defaults),
            "num_trials": trials,
            "output_directory": f"results/final/scalability/{scenario_id}/",
        })

    return rows


def main():
    config = json.load(open(os.path.join(_ROOT_DIR, "simulation_config.json")))
    rows = build_matrix(config)
    with open(OUT_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} rows to {OUT_PATH}")


if __name__ == "__main__":
    main()
