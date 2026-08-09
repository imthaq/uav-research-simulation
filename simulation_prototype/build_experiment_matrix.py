import csv
import json
import os
import sys

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

# Radar-relevant fields worth summarizing per scenario, in priority order
# (only the ones actually overridden away from radar.* defaults are shown -
# see _radar_configuration_summary).
_RADAR_FIELDS = [
    ("radar_detection_probability", "P_D"),
    ("radar_false_alarm_probability", "P_FA"),
    ("radar_clutter_density", "clutter_density"),
    ("radar_range_noise_std", "range_noise_std"),
    ("radar_bearing_noise_std", "bearing_noise_std"),
    ("radar_radial_velocity_noise_std", "radial_velocity_noise_std"),
    ("radar_latency_steps", "latency_steps"),
    ("radar_dropout_probability", "dropout_probability"),
    ("radar_confidence_error", "confidence_error"),
]

# The focused core matrix: which scenario id answers each required test
# case, plus the two axes the config itself doesn't decide for us
# (fusion architecture, and how many trials this scenario is worth given
# its cost/variance). Everything else - fusion_mode, communication preset,
# radar overrides, environment - is read from simulation_config.json.
CORE_SCENARIOS = [
    # (scenario_id, architecture, trial_count)
    ("baseline", "centralized", 20),
    ("very_low_P_D", "centralized", 20),
    ("very_high_P_FA", "centralized", 20),
    ("high_clutter", "centralized", 20),
    ("sensor_noise", "centralized", 20),
    ("high_latency", "centralized", 20),
    ("high_dropout", "centralized", 20),
    # Packet loss / outage only have teeth under the distributed
    # architecture - fusion_model.py's own docstring notes centralized's
    # single uplink/downlink round trip is modeled as reliable, with no
    # packet-loss equivalent of its own.
    ("high_packet_loss", "distributed", 20),
    ("communication_outage", "distributed", 20),
    ("overconfident_faulty_sensor", "centralized", 20),
    ("two_crossing_targets", "centralized", 20),
    ("rapidly_moving_obstacle", "centralized", 20),
]

FIELDNAMES = [
    "scenario_id", "radar_configuration", "environment", "fusion_mode",
    "architecture", "communication_condition", "trial_count", "seed_range",
    "output_directory",
]


def _radar_configuration_summary(scn):
    """Human-readable summary of only the radar fields this scenario
    actually overrides away from radar.* defaults in the config."""
    overrides = [f"{label}={scn[key]}" for key, label in _RADAR_FIELDS if key in scn]
    if not overrides:
        return "defaults (no radar-model override)"
    return ", ".join(overrides)


def _communication_condition(scn, comm_defaults):
    comm = scn.get("communication", {})
    preset = comm.get("preset", comm_defaults.get("preset", "perfect"))
    return preset if "communication" in scn else f"{preset} (default)"


def build_matrix(config):
    scenarios = config["scenarios"]
    base_seed = config["sim"]["seed"]
    comm_defaults = config.get("communication", {})
    output_dir = config.get("reproducibility", {}).get("output_location", "results")

    rows = []
    for scenario_id, architecture, trial_count in CORE_SCENARIOS:
        if scenario_id not in scenarios:
            raise KeyError(f"CORE_SCENARIOS references {scenario_id!r}, "
                            f"which is not defined in simulation_config.json")
        scn = scenarios[scenario_id]
        seed_range = f"{base_seed}-{base_seed + trial_count - 1}"
        rows.append({
            "scenario_id": scenario_id,
            "radar_configuration": _radar_configuration_summary(scn),
            "environment": scn.get("environment_mode", "clear (default)"),
            "fusion_mode": scn.get("fusion_mode", "no_fusion"),
            "architecture": architecture,
            "communication_condition": _communication_condition(scn, comm_defaults),
            "trial_count": trial_count,
            "seed_range": seed_range,
            # Matches run_experiments.py's own out_path convention
            # (os.path.join(logs_dir, f"{scenario_name}_run{run_number}.csv")).
            "output_directory": f"logs/ ({scenario_id}_run<N>.csv)",
        })
    return rows


def main():
    config_path = os.path.join(_ROOT_DIR, "simulation_config.json")
    with open(config_path) as f:
        config = json.load(f)

    rows = build_matrix(config)

    out_dir = os.path.join(_ROOT_DIR, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "final_experiment_matrix.csv")
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} scenario rows to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
