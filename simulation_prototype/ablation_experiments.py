"""
ablation_experiments.py

Ablation study: systematically disable one component at a time and measure
the impact on fusion error, collision risk, mission success, response time, 
and formation error.

Ablation targets:
1. no_radar_tracking    - disable radar sensor (detection_probability=0)
2. no_confidence        - disable confidence estimation (force all to 1.0)
3. no_trust_weighting   - use naive_fusion instead of weighted fusion
4. no_covariance        - disable covariance-based weighting
5. no_latency           - remove all sensor latency
6. no_stale_data        - disable stale-data rejection (dropout, age decay)
7. no_communication     - use no_fusion mode (no inter-UAV fusion)
8. no_dynamic_trust     - disable dynamic trust adaptation

Each ablation creates a variant config and runs a subset of scenarios.
Results are written to ablation_results.csv.
"""

import argparse
import copy
import csv
import json
import os
import sys
import random
from datetime import datetime

from simple_swarm_sim import Simulation


def apply_ablation(config, ablation_name):
    """Apply an ablation modification to a config copy."""
    ablated = copy.deepcopy(config)
    
    if ablation_name == "no_radar_tracking":
        # Disable radar detection entirely
        ablated["radar"]["radar_detection_probability"] = 0.0
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_confidence":
        # Force confidence to maximum (no error)
        for scenario in ablated["scenarios"].values():
            scenario["confidence_error_level"] = 0.0
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_trust_weighting":
        # Force all scenarios to naive_fusion instead of weighted modes
        for scenario in ablated["scenarios"].values():
            if scenario.get("fusion_mode") in ["confidence_weighted_fusion", 
                                               "trust_weighted_fusion",
                                               "covariance_weighted_fusion",
                                               "covariance_intersection_fusion"]:
                scenario["fusion_mode"] = "naive_fusion"
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_covariance":
        # Disable covariance by forcing naive or confidence-weighted fusion
        for scenario in ablated["scenarios"].values():
            mode = scenario.get("fusion_mode", "no_fusion")
            if mode in ["covariance_weighted_fusion", "covariance_intersection_fusion"]:
                scenario["fusion_mode"] = "confidence_weighted_fusion"
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_latency":
        # Remove all latency from config
        ablated["radar"]["radar_latency_steps"] = 0
        for scenario in ablated["scenarios"].values():
            scenario["latency_steps"] = 0
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_stale_data":
        # Disable dropout and aging effects
        ablated["radar"]["radar_dropout_probability"] = 0.0
        for scenario in ablated["scenarios"].values():
            scenario["dropout_prob"] = 0.0
        # Note: age decay is baked into track covariance; can't fully disable
        # without code changes, but zero dropout gets most of the effect
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_communication":
        # Disable inter-UAV fusion
        for scenario in ablated["scenarios"].values():
            scenario["fusion_mode"] = "no_fusion"
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    elif ablation_name == "no_dynamic_trust":
        # Can't easily disable dynamic trust via config alone.
        # Best we can do: use a fusion mode that doesn't heavily weight on trust
        # (naive or confidence-weighted instead of trust_weighted)
        for scenario in ablated["scenarios"].values():
            if scenario.get("fusion_mode") == "trust_weighted_fusion":
                scenario["fusion_mode"] = "confidence_weighted_fusion"
        ablated["sim"]["description"] = f"Ablation: {ablation_name}"
    
    else:
        raise ValueError(f"Unknown ablation: {ablation_name}")
    
    return ablated


def run_ablation_experiment(base_config, ablation_name, scenario_names, 
                           num_trials=5, base_seed=42):
    """Run an ablation variant for selected scenarios."""
    ablated_config = apply_ablation(base_config, ablation_name)
    
    rows = []
    for scenario_name in scenario_names:
        if scenario_name not in ablated_config["scenarios"]:
            continue
        
        scn = ablated_config["scenarios"][scenario_name]
        params = {
            "false_positive_rate": scn.get("false_positive_rate", 0.0),
            "false_negative_rate": scn.get("false_negative_rate", 0.0),
            "noise_level": scn.get("position_noise_std", 0.0),
            "latency_steps": scn.get("latency_steps", 0),
            "dropout_probability": scn.get("dropout_prob", 0.0),
            "confidence_error_level": scn.get("confidence_error_level", 0.0),
            "fusion_mode": scn.get("fusion_mode", "no_fusion"),
        }
        
        for trial in range(1, num_trials + 1):
            seed = base_seed + (trial - 1)
            ablated_config["sim"]["seed"] = seed
            
            try:
                sim = Simulation(ablated_config, scenario_name)
                metrics = sim.run()
                collision_risk_count = sum(1 for row in sim.log_rows if row.get("collision_risk_flag"))
                
                rows.append({
                    "ablation": ablation_name,
                    "scenario": scenario_name,
                    "trial": trial,
                    "seed": seed,
                    "fusion_mode": params["fusion_mode"],
                    "false_positive_rate": params["false_positive_rate"],
                    "false_negative_rate": params["false_negative_rate"],
                    "noise_level": params["noise_level"],
                    "latency_steps": params["latency_steps"],
                    "dropout_probability": params["dropout_probability"],
                    "confidence_error_level": params["confidence_error_level"],
                    "collision_risk_count": collision_risk_count,
                    "missed_response_count": metrics["missed_response_count"],
                    "fusion_recovery_count": metrics["fusion_recovery_count"],
                    "mission_success": "Yes" if metrics["mission_success"] else "No",
                    "avg_response_time_s": metrics["avg_response_time_s"],
                    "avg_formation_error": metrics["avg_formation_error"],
                })
            except Exception as e:
                print(f"Error in {ablation_name} / {scenario_name} / trial {trial}: {e}")
                continue
    
    return rows


def main():
    parser = argparse.ArgumentParser(description="Run ablation study on swarm simulation")
    parser.add_argument("--config", default="simulation_config.json",
                       help="Base simulation config")
    parser.add_argument("--scenarios", nargs="+", default=None,
                       help="Scenarios to ablate (default: all non-baseline)")
    parser.add_argument("--trials", type=int, default=5,
                       help="Number of trials per scenario per ablation")
    parser.add_argument("--output", default="results/ablation_results.csv",
                       help="Output CSV file")
    parser.add_argument("--seed", type=int, default=42,
                       help="Base random seed")
    args = parser.parse_args()
    
    with open(args.config) as f:
        base_config = json.load(f)
    
    # Default to all scenarios except baseline
    if not args.scenarios:
        all_scenarios = list(base_config["scenarios"].keys())
        args.scenarios = [s for s in all_scenarios if s != "baseline"]
    
    ablation_names = [
        "no_radar_tracking",
        "no_confidence",
        "no_trust_weighting",
        "no_covariance",
        "no_latency",
        "no_stale_data",
        "no_communication",
        "no_dynamic_trust",
    ]
    
    fieldnames = [
        "ablation", "scenario", "trial", "seed",
        "fusion_mode", "false_positive_rate", "false_negative_rate",
        "noise_level", "latency_steps", "dropout_probability",
        "confidence_error_level", "collision_risk_count",
        "missed_response_count", "fusion_recovery_count",
        "mission_success", "avg_response_time_s", "avg_formation_error",
    ]
    
    out_dir = os.path.dirname(args.output) or "."
    os.makedirs(out_dir, exist_ok=True)
    
    all_rows = []
    start_time = datetime.now()
    
    for i, ablation_name in enumerate(ablation_names):
        print(f"[{i+1}/{len(ablation_names)}] Running ablation: {ablation_name}")
        rows = run_ablation_experiment(base_config, ablation_name, 
                                      args.scenarios, args.trials, args.seed)
        all_rows.extend(rows)
        print(f"  Collected {len(rows)} runs")
    
    # Write results
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)
    
    elapsed = (datetime.now() - start_time).total_seconds()
    print(f"\nAblation study complete: {len(all_rows)} total runs in {elapsed:.1f}s")
    print(f"Results written to: {args.output}")
    
    # Print summary per ablation
    from collections import defaultdict
    ablation_groups = defaultdict(list)
    for row in all_rows:
        ablation_groups[row["ablation"]].append(row)
    
    print("\n=== Ablation Summary ===")
    for ablation_name in ablation_names:
        rows = ablation_groups[ablation_name]
        if not rows:
            continue
        
        success_count = sum(1 for r in rows if r["mission_success"] == "Yes")
        avg_collision = sum(float(r["collision_risk_count"]) for r in rows) / len(rows) if rows else 0
        avg_response = sum(float(r["avg_response_time_s"]) for r in rows 
                          if r["avg_response_time_s"]) / len([r for r in rows if r["avg_response_time_s"]]) if rows else 0
        
        print(f"  {ablation_name:25} - success={success_count}/{len(rows)}, "
              f"avg_collision={avg_collision:.1f}, avg_response={avg_response:.3f}s")


if __name__ == "__main__":
    sys.exit(main())
