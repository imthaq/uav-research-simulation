"""
radar_calibration_analysis.py

Dedicated confidence-calibration check for the radar model in
radar_like_model.py: does a reported detection probability of, say, 0.8
actually correspond to about an 80% correct-detection frequency across
repeated trials?

This pairs each real target's probability_of_detection (Task 5's
per-detection effective P_D, which varies with range/SNR/environment/
reliability - see radar_like_model._measurement_uncertainty) against
whether that target was actually detected. It deliberately does not use
confidence_score: that field is only ever reported on rows already known,
by construction, to be a genuine detection or a confirmed false alarm, so
calibrating it against that label is trivially true and uninformative.
See the "Confidence calibration (Task 3)" section of radar_like_model.py
for the full rationale, and calibration_pairs() there for exactly which
rows are included/excluded (false alarms, radar dropouts, and hard
range/FOV-gated misses are all excluded since their outcome wasn't
actually determined by the PD roll).

This script runs RadarLikeModel directly (not the full track-fusion
pipeline) across one or more seeded runs per scenario, pools every
(probability_of_detection, detected) pair it produces, and reports:
  - Expected Calibration Error (ECE)
  - Maximum Calibration Error (MCE)
  - Brier score
  - negative log-likelihood
  - reliability-bin accuracy / reliability-bin confidence
  - overconfidence rate / underconfidence rate
  - a simple ASCII reliability diagram

Usage:
    python radar_calibration_analysis.py --config simulation_config.json --runs 20
    python radar_calibration_analysis.py --scenario baseline --runs 50 --bins 20
"""

import argparse
import copy
import json
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from models.radar_like_model import RadarLikeModel, calibration_pairs
from metrics_analysis import confidence_calibration_metrics


def _collect_pairs(config, scenario_name, num_runs, base_seed):
    """Runs RadarLikeModel num_runs times (one seed per run, matching the
    seeding convention used elsewhere in this project) and pools every
    (confidence, correct) pair across all of them, since calibration
    checks need many repeated trials at each confidence level to be
    meaningful - a single run rarely has enough samples per bin."""
    pairs = []
    for run_number in range(num_runs):
        run_config = copy.deepcopy(config)
        run_config["sim"]["seed"] = base_seed + run_number
        model = RadarLikeModel(run_config, scenario_name)
        rows = model.run()
        pairs.extend(calibration_pairs(rows))
    return pairs


def _pairs_to_rows(pairs):
    """Repackages (probability_of_detection, detected) pairs as minimal
    row dicts so they can be fed straight into
    metrics_analysis.confidence_calibration_metrics, which only reads
    probability_of_detection/detection_status for this purpose."""
    return [{"probability_of_detection": p,
             "detection_status": "detected" if detected else "missed"}
            for p, detected in pairs]


def analyze_scenario(config, scenario_name, num_runs, base_seed, num_bins):
    pairs = _collect_pairs(config, scenario_name, num_runs, base_seed)
    rows = _pairs_to_rows(pairs)
    return confidence_calibration_metrics(rows, num_bins=num_bins)


def ascii_reliability_diagram(reliability_bins, width=30):
    """Renders one line per reliability bin: sample count, mean reported
    confidence, mean actual accuracy, and a bar sized to the accuracy so
    over/under-confidence is visible at a glance (a bar shorter than the
    bin's confidence range means the radar is overconfident there; longer
    means underconfident)."""
    lines = [f"{'bin':>12}  {'n':>7}  {'confidence':>10}  {'accuracy':>8}  reliability"]
    lines.append("-" * len(lines[0]))
    for b in reliability_bins:
        lo, hi = b["bin_range"]
        label = f"[{lo:.2f}-{hi:.2f}]"
        if b["n"] == 0:
            lines.append(f"{label:>12}  {0:>7}  {'--':>10}  {'--':>8}  (no samples)")
            continue
        acc = b["bin_accuracy"]
        bar_len = max(0, min(width, round(acc * width)))
        bar = "#" * bar_len + "." * (width - bar_len)
        lines.append(f"{label:>12}  {b['n']:>7}  {b['bin_confidence']:>10.3f}  "
                      f"{acc:>8.3f}  {bar}")
    return "\n".join(lines)


def print_scenario_report(scenario_name, metrics):
    print(f"\n=== {scenario_name} (n={metrics['n_samples']} real-target PD trials) ===")
    if metrics["n_samples"] == 0:
        print("  No real-target detection-probability trials were recorded for "
              "this scenario - nothing to calibrate.")
        return
    print(f"  Expected Calibration Error (ECE): {metrics['expected_calibration_error']}")
    print(f"  Maximum Calibration Error (MCE):  {metrics['maximum_calibration_error']}")
    print(f"  Brier score:                      {metrics['brier_score']}")
    print(f"  Negative log-likelihood:          {metrics['negative_log_likelihood']}")
    print(f"  Overconfidence rate:              {metrics['overconfidence_rate']}")
    print(f"  Underconfidence rate:             {metrics['underconfidence_rate']}")
    print()
    print(ascii_reliability_diagram(metrics["reliability_bins"]))


def main():
    parser = argparse.ArgumentParser(
        description="Radar confidence-calibration analysis: checks whether reported "
                    "radar confidence matches actual correct-detection frequency.")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "simulation_config.json"))
    parser.add_argument("--scenario", default=None, help="Only analyze this one scenario")
    parser.add_argument("--runs", type=int, default=20,
                        help="Seeded runs per scenario, pooled for more calibration samples")
    parser.add_argument("--bins", type=int, default=10, help="Number of reliability bins")
    parser.add_argument("--output", default=os.path.join(PROJECT_ROOT, "results", "radar_calibration.json"))
    args = parser.parse_args()

    if args.runs < 1:
        sys.exit(f"--runs must be >= 1 (got {args.runs})")
    if args.bins < 1:
        sys.exit(f"--bins must be >= 1 (got {args.bins})")

    with open(args.config) as f:
        config = json.load(f)

    if args.scenario and args.scenario not in config["scenarios"]:
        available = ", ".join(config["scenarios"].keys())
        sys.exit(f"Unknown scenario '{args.scenario}'. Available: {available}")

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    base_seed = config["sim"].get("seed", 0)

    results = {}
    for scenario_name in scenario_names:
        metrics = analyze_scenario(config, scenario_name, args.runs, base_seed, args.bins)
        results[scenario_name] = metrics
        print_scenario_report(scenario_name, metrics)

    out_path = args.output
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults written to: {out_path}")


if __name__ == "__main__":
    sys.exit(main())