"""
radar_confidence_calibration.py

Task 4: fits and compares post-hoc calibration methods for the radar's
per-detection probability_of_detection (Task 5's effective P_D - see
radar_like_model._measurement_uncertainty), on top of the raw calibration
check radar_calibration_analysis.py (Task 3) already reports.

Calibration methods compared
-----------------------------
  1. uncalibrated       - the radar's raw probability_of_detection,
                           unchanged. The baseline every other method is
                           judged against.
  2. temperature_scaling - a single scalar T > 0 (fit by minimizing mean
                           binary cross-entropy on the calibration
                           subset) applied as sigmoid(logit(p) / T).
                           T > 1 flattens overconfident probabilities
                           toward 0.5; T < 1 sharpens underconfident ones.
  3. histogram_binning   - per-bin empirical accuracy from the
                           calibration subset (same equal-width bins
                           confidence_calibration_metrics uses for its
                           reliability diagram), looked up by whichever
                           bin an evaluation sample's raw probability
                           falls into.
  4. isotonic            - pool-adjacent-violators (PAV) monotonic
                           regression fit on the calibration subset,
                           applied by linear interpolation between the
                           fitted breakpoints (the same interpolating
                           behavior scikit-learn's IsotonicRegression
                           uses) - optional, on by default, disable with
                           --no-isotonic.

All four reuse radar_calibration_analysis.py / metrics_analysis.py's
existing (probability_of_detection, detected) pair extraction and
ECE/MCE/Brier/NLL/reliability-bin metric computation from Task 3 rather
than recomputing them: metrics_analysis._calibration_pairs pulls the
pairs, radar_calibration_analysis._pairs_to_rows repackages a
recalibrated pair list back into the minimal row shape
confidence_calibration_metrics expects, and confidence_calibration_metrics
itself computes every metric this script reports (identically) for the
raw and each recalibrated pair set.

Note: this deliberately does NOT go through
radar_like_model.calibration_pairs() / radar_calibration_analysis.py's
own _collect_pairs() helper - that pairs confidence_score against a
detected-vs-false-alarm label, a different (and, on rows with a reported
confidence, already tautological per that function's own docstring)
calibration question from the one this script and
metrics_analysis.confidence_calibration_metrics both document: whether
probability_of_detection matches the actual real-target detection rate.

Calibration/evaluation split
-----------------------------
Fitting a calibration method on the same samples used to evaluate it
overstates how well it generalizes - it only proves the method can
imitate the empirical accuracy it was itself fit to. This script always
draws two disjoint seeded RadarLikeModel run pools per scenario:
  - a "calibration" subset (--calib-runs runs, seeds
    base_seed .. base_seed + calib_runs - 1) used only to fit T / bin
    accuracies / isotonic breakpoints;
  - a separate "evaluation" subset (--eval-runs runs, seeds
    base_seed + calib_runs .. base_seed + calib_runs + eval_runs - 1,
    i.e. never overlapping the calibration seeds) used only to report
    pre- and post-calibration metrics.
Pre-calibration metrics are also reported on the calibration subset
itself (clearly labeled "on_calibration_subset"), purely as a sanity
check on what's being fit - only the "on_evaluation_subset" /
per-method eval numbers say anything about how well calibration
generalizes.

Usage:
    python radar_confidence_calibration.py --config simulation_config.json \
        --calib-runs 20 --eval-runs 20
    python radar_confidence_calibration.py --scenario baseline --no-isotonic
"""

import argparse
import copy
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from models.radar_like_model import RadarLikeModel
from metrics_analysis import _calibration_pairs, confidence_calibration_metrics
from radar_calibration_analysis import _pairs_to_rows

EPS = 1e-7
METHOD_NAMES = ("uncalibrated", "temperature_scaling", "histogram_binning", "isotonic")


# ------------------------------------------------------------------
# Pair collection (reuses metrics_analysis._calibration_pairs; see the
# module docstring for why that extractor is used and not
# radar_like_model.calibration_pairs).
# ------------------------------------------------------------------
def _run_pairs(config, scenario_name, num_runs, base_seed):
    """Runs RadarLikeModel num_runs times (seeds base_seed..base_seed+
    num_runs-1) and pools every (probability_of_detection, detected)
    pair across all of them - repeated trials are needed for a
    meaningful per-bin sample count, same rationale as
    radar_calibration_analysis._collect_pairs."""
    pairs = []
    for i in range(num_runs):
        run_config = copy.deepcopy(config)
        run_config["sim"]["seed"] = base_seed + i
        model = RadarLikeModel(run_config, scenario_name)
        rows = model.run()
        pairs.extend(_calibration_pairs(rows))
    return pairs


def metrics_for_pairs(pairs, num_bins):
    """Computes the full Task 3 metric set (ECE/MCE/Brier/NLL/
    reliability bins/over-under-confidence rate) for an arbitrary
    (possibly recalibrated) pair list, by reusing
    confidence_calibration_metrics unchanged."""
    return confidence_calibration_metrics(_pairs_to_rows(pairs), num_bins=num_bins)


# ------------------------------------------------------------------
# Method 2: temperature scaling
# ------------------------------------------------------------------
def _logit(p):
    p = min(max(p, EPS), 1.0 - EPS)
    return math.log(p / (1.0 - p))


def fit_temperature(pairs):
    """Fits a single scalar T > 0 minimizing mean binary cross-entropy of
    sigmoid(logit(p)/T) against the binary outcome, over the calibration
    subset. Falls back to T=1.0 (no-op) if there's nothing to fit."""
    if not pairs:
        return 1.0
    logits = np.array([_logit(p) for p, _ in pairs])
    y = np.array([1.0 if c else 0.0 for _, c in pairs])

    def nll(T):
        T = max(T, 1e-3)
        p = 1.0 / (1.0 + np.exp(-logits / T))
        p = np.clip(p, EPS, 1.0 - EPS)
        return -np.mean(y * np.log(p) + (1.0 - y) * np.log(1.0 - p))

    result = minimize_scalar(nll, bounds=(0.05, 20.0), method="bounded")
    return float(result.x)


def apply_temperature(p, temperature):
    z = _logit(p) / temperature
    return 1.0 / (1.0 + math.exp(-z))


# ------------------------------------------------------------------
# Method 3: histogram / bin-based calibration
# ------------------------------------------------------------------
def fit_histogram_binning(pairs, num_bins):
    """Bins the calibration subset into num_bins equal-width [0, 1] bins
    (the same binning confidence_calibration_metrics uses for reliability
    bins) and records each bin's empirical accuracy. A histogram-
    calibrated probability is then exactly "what fraction of
    calibration-subset samples reported in this same confidence range
    were actually correct." Empty bins fall back to the overall
    calibration-subset base rate, since there's no other information to
    calibrate them with."""
    base_rate = (sum(1.0 for _, c in pairs if c) / len(pairs)) if pairs else 0.5
    bin_sums = [0.0] * num_bins
    bin_counts = [0] * num_bins
    for p, c in pairs:
        idx = min(max(int(p * num_bins), 0), num_bins - 1)
        bin_sums[idx] += 1.0 if c else 0.0
        bin_counts[idx] += 1
    bin_accuracy = [
        (bin_sums[i] / bin_counts[i]) if bin_counts[i] else base_rate
        for i in range(num_bins)
    ]
    return {"num_bins": num_bins, "bin_accuracy": bin_accuracy, "base_rate": base_rate}


def apply_histogram_binning(p, params):
    num_bins = params["num_bins"]
    idx = min(max(int(p * num_bins), 0), num_bins - 1)
    return params["bin_accuracy"][idx]


# ------------------------------------------------------------------
# Method 4 (optional): isotonic regression via pool-adjacent-violators
# ------------------------------------------------------------------
def _pool_adjacent_violators(y):
    """Standard PAV: the least-squares non-decreasing fit to y. Returns
    fitted values, same length/order as y."""
    values, weights = [], []
    for yi in y:
        values.append(float(yi))
        weights.append(1.0)
        while len(values) > 1 and values[-2] > values[-1]:
            v2, w2 = values.pop(), weights.pop()
            v1, w1 = values.pop(), weights.pop()
            values.append((v1 * w1 + v2 * w2) / (w1 + w2))
            weights.append(w1 + w2)
    fitted = []
    for v, w in zip(values, weights):
        fitted.extend([v] * int(w))
    return fitted


def fit_isotonic(pairs):
    """Isotonic (PAV) calibration: sorts the calibration subset by raw
    probability, pool-adjacent-violates the binary outcomes into a
    monotonic non-decreasing fit, then collapses duplicate x's (averaging
    their fitted values) into a strictly increasing breakpoint table for
    apply_isotonic to linearly interpolate over. Returns None if there
    are fewer than 2 samples."""
    if len(pairs) < 2:
        return None
    ordered = sorted(pairs, key=lambda pc: pc[0])
    xs = [p for p, _ in ordered]
    ys = [1.0 if c else 0.0 for _, c in ordered]
    fitted = _pool_adjacent_violators(ys)

    sums, counts = {}, {}
    for x, f in zip(xs, fitted):
        sums[x] = sums.get(x, 0.0) + f
        counts[x] = counts.get(x, 0) + 1
    uniq_x = sorted(sums)
    uniq_y = [sums[x] / counts[x] for x in uniq_x]
    return {"x": uniq_x, "y": uniq_y}


def apply_isotonic(p, params):
    if params is None:
        return p
    return float(np.interp(p, params["x"], params["y"]))


# ------------------------------------------------------------------
# Orchestration
# ------------------------------------------------------------------
def calibrate_and_evaluate(config, scenario_name, calib_runs, eval_runs,
                            base_seed, num_bins, use_isotonic=True):
    calib_pairs = _run_pairs(config, scenario_name, calib_runs, base_seed)
    eval_seed_start = base_seed + calib_runs
    eval_pairs = _run_pairs(config, scenario_name, eval_runs, eval_seed_start)

    result = {
        "scenario": scenario_name,
        "calibration_seeds": [base_seed, base_seed + calib_runs - 1],
        "evaluation_seeds": [eval_seed_start, eval_seed_start + eval_runs - 1],
        "n_calibration_samples": len(calib_pairs),
        "n_evaluation_samples": len(eval_pairs),
        "pre_calibration": {
            "on_calibration_subset": metrics_for_pairs(calib_pairs, num_bins),
            "on_evaluation_subset": metrics_for_pairs(eval_pairs, num_bins),
        },
        "methods": {},
    }

    if not calib_pairs or not eval_pairs:
        return result

    # 1. uncalibrated baseline - same numbers as pre_calibration's
    # eval-subset entry, repeated under "methods" so every method
    # (including doing nothing) sits in one side-by-side comparison.
    result["methods"]["uncalibrated"] = {
        "params": None,
        "metrics": result["pre_calibration"]["on_evaluation_subset"],
    }

    # 2. temperature scaling
    temperature = fit_temperature(calib_pairs)
    temp_eval_pairs = [(apply_temperature(p, temperature), c) for p, c in eval_pairs]
    result["methods"]["temperature_scaling"] = {
        "params": {"temperature": round(temperature, 4)},
        "metrics": metrics_for_pairs(temp_eval_pairs, num_bins),
    }

    # 3. histogram binning
    hist_params = fit_histogram_binning(calib_pairs, num_bins)
    hist_eval_pairs = [(apply_histogram_binning(p, hist_params), c) for p, c in eval_pairs]
    result["methods"]["histogram_binning"] = {
        "params": {
            "num_bins": hist_params["num_bins"],
            "bin_accuracy": [round(v, 4) for v in hist_params["bin_accuracy"]],
            "base_rate": round(hist_params["base_rate"], 4),
        },
        "metrics": metrics_for_pairs(hist_eval_pairs, num_bins),
    }

    # 4. isotonic (optional)
    if use_isotonic:
        iso_params = fit_isotonic(calib_pairs)
        iso_eval_pairs = [(apply_isotonic(p, iso_params), c) for p, c in eval_pairs]
        result["methods"]["isotonic"] = {
            "params": ({
                "breakpoints_x": [round(x, 4) for x in iso_params["x"]],
                "breakpoints_y": [round(y, 4) for y in iso_params["y"]],
            } if iso_params else None),
            "metrics": metrics_for_pairs(iso_eval_pairs, num_bins),
        }

    return result


# ------------------------------------------------------------------
# Reliability diagrams
# ------------------------------------------------------------------
def _plot_reliability(ax, metrics, title):
    bins = [b for b in metrics["reliability_bins"] if b["n"]]
    ax.plot([0, 1], [0, 1], linestyle="--", color="gray", linewidth=1, label="perfect calibration")
    if bins:
        conf = [b["bin_confidence"] for b in bins]
        acc = [b["bin_accuracy"] for b in bins]
        counts = [b["n"] for b in bins]
        max_n = max(counts)
        sizes = [25 + 200 * (n / max_n) for n in counts]
        ax.scatter(conf, acc, s=sizes, alpha=0.75, zorder=3)
        ax.plot(conf, acc, alpha=0.5, zorder=2)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xlabel("mean reported probability_of_detection")
    ax.set_ylabel("empirical detection rate")
    ece = metrics.get("expected_calibration_error")
    n = metrics.get("n_samples")
    ax.set_title(f"{title}\nECE={ece}  (n={n})")


def save_reliability_diagrams(scenario_result, out_dir, scenario_name):
    methods = scenario_result["methods"]
    if not methods:
        return None
    n = len(methods)
    fig, axes = plt.subplots(1, n, figsize=(4.4 * n, 4.2), squeeze=False)
    for ax, (name, m) in zip(axes[0], methods.items()):
        _plot_reliability(ax, m["metrics"], name)
    fig.suptitle(f"Radar detection-probability calibration - {scenario_name} (evaluation subset)")
    fig.tight_layout()
    path = os.path.join(out_dir, f"reliability_{scenario_name}.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


# ------------------------------------------------------------------
# Reporting
# ------------------------------------------------------------------
def print_scenario_summary(scenario_name, result):
    print(f"\n=== {scenario_name} ===")
    print(f"  calibration subset: n={result['n_calibration_samples']} "
          f"(seeds {result['calibration_seeds']})")
    print(f"  evaluation subset:  n={result['n_evaluation_samples']} "
          f"(seeds {result['evaluation_seeds']})")
    if not result["methods"]:
        print("  Not enough samples in one of the subsets - nothing to calibrate.")
        return
    print(f"  {'method':<20}{'ECE':>8}{'MCE':>8}{'Brier':>9}{'NLL':>9}"
          f"{'overconf':>10}{'underconf':>10}")
    for name, m in result["methods"].items():
        met = m["metrics"]
        print(f"  {name:<20}{met['expected_calibration_error']:>8}"
              f"{met['maximum_calibration_error']:>8}{met['brier_score']:>9}"
              f"{met['negative_log_likelihood']:>9}{met['overconfidence_rate']:>10}"
              f"{met['underconfidence_rate']:>10}")


# ------------------------------------------------------------------
# Self-check (ponytail: non-trivial logic needs one runnable check)
# ------------------------------------------------------------------
def _self_check():
    fitted = _pool_adjacent_violators([0.0, 1.0, 0.0, 1.0, 1.0])
    assert all(fitted[i] <= fitted[i + 1] + 1e-9 for i in range(len(fitted) - 1)), \
        "PAV output must be non-decreasing"

    pairs = [(0.05, True)] * 10 + [(0.95, True)] * 10
    hist = fit_histogram_binning(pairs, num_bins=10)
    assert apply_histogram_binning(0.95, hist) > 0.9
    assert apply_histogram_binning(0.05, hist) > 0.9

    overconfident_pairs = [(0.95, i % 5 == 0) for i in range(100)]  # ~20% actually correct
    temperature = fit_temperature(overconfident_pairs)
    assert temperature > 1.0, "an overconfident set should fit T > 1 (flattening toward 0.5)"

    iso = fit_isotonic([(0.1, False), (0.3, True), (0.3, False), (0.6, True), (0.9, True)])
    assert all(iso["y"][i] <= iso["y"][i + 1] + 1e-9 for i in range(len(iso["y"]) - 1)), \
        "isotonic breakpoints must be non-decreasing"

    # calib/eval split must never share seeds
    seeds_calib = set(range(0, 5))
    seeds_eval = set(range(5, 5 + 5))
    assert not (seeds_calib & seeds_eval)


def main():
    _self_check()

    parser = argparse.ArgumentParser(
        description="Fits and compares radar probability_of_detection calibration methods "
                    "(temperature scaling, histogram binning, isotonic) against the raw, "
                    "uncalibrated confidence, evaluated on a held-out simulation subset.")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Only analyze this one scenario")
    parser.add_argument("--calib-runs", type=int, default=20,
                        help="Seeded runs used to fit calibration methods")
    parser.add_argument("--eval-runs", type=int, default=20,
                        help="Separate seeded runs (non-overlapping seeds) used to evaluate them")
    parser.add_argument("--bins", type=int, default=10, help="Number of reliability bins")
    parser.add_argument("--no-isotonic", action="store_true",
                        help="Skip isotonic calibration (methods 1-3 only)")
    parser.add_argument("--output", default="results/radar_confidence_calibration.json")
    parser.add_argument("--plot-dir", default="results/reliability_diagrams")
    args = parser.parse_args()

    if args.calib_runs < 1 or args.eval_runs < 1:
        sys.exit("--calib-runs and --eval-runs must each be >= 1")
    if args.bins < 1:
        sys.exit(f"--bins must be >= 1 (got {args.bins})")

    with open(args.config) as f:
        config = json.load(f)

    if args.scenario and args.scenario not in config["scenarios"]:
        available = ", ".join(config["scenarios"].keys())
        sys.exit(f"Unknown scenario '{args.scenario}'. Available: {available}")

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    base_seed = config["sim"].get("seed", 0)

    os.makedirs(args.plot_dir, exist_ok=True)

    all_results = {}
    for scenario_name in scenario_names:
        result = calibrate_and_evaluate(
            config, scenario_name, args.calib_runs, args.eval_runs, base_seed,
            args.bins, use_isotonic=not args.no_isotonic)
        result["reliability_diagram"] = save_reliability_diagrams(result, args.plot_dir, scenario_name)
        all_results[scenario_name] = result
        print_scenario_summary(scenario_name, result)

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\nResults written to: {args.output}")
    print(f"Reliability diagrams written to: {args.plot_dir}")


if __name__ == "__main__":
    sys.exit(main())
