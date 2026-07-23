"""
generate_final_dependability_plots.py

Task 25: the 15 plots required for the final dependability deliverable,
saved into simulation_prototype/plots/dependability/:

  1.  calibration_reliability_diagram.png
  2.  calibration_error_vs_collision_risk.png
  3.  registration_error_vs_fusion_rmse.png
  4.  sensor_correlation_vs_covariance_consistency.png
  5.  doppler_ambiguity_vs_response_error.png
  6.  ghost_return_rate_vs_false_track_count.png
  7.  perception_quality_vs_safety_margin.png
  8.  abstention_threshold_vs_mission_success.png
  9.  abstention_threshold_vs_mission_delay.png
  10. handoff_strategy_vs_collision_risk.png
  11. handoff_strategy_vs_recovery_time.png
  12. combined_fault_severity_vs_mission_success.png
  13. swarm_size_vs_communication_load.png
  14. swarm_size_vs_simulation_runtime.png
  15. final_failure_envelope_heatmap.png

This is a thin, additive layer on top of work this project already did,
not a reimplementation of any of it:

  - Plot 1 reuses radar_confidence_calibration.py's own `_plot_reliability`
    axes-drawing helper, fed by radar_calibration_analysis.py's
    `analyze_scenario` (the same Task-23 calibration analysis
    run_final_dependability_experiments.py's comparison #1 uses).
  - Plots 2, 3, 5, 6, 7, 12, 13, 14 drive the real pipeline through
    metrics_analysis.run_once() (or, where only detection-level radar
    output is needed, models.radar_like_model.RadarLikeModel directly),
    using generate_plots.py's own `mean_ci`/`line_ci_plot`/`bar_ci_plot`/
    `gather_rows`/`_adv_style` so the visual style matches the existing
    plots/final/ set exactly.
  - Plots 8-11 reuse dependability_common's seed_range/clone_scenario and
    dependability_controllers' attach_dependability_layer/run_controller/
    CONTROLLERS (Task 17's abstention and handoff wiring) rather than
    re-deriving abstention/handoff behavior here.
  - Plot 4 is a controlled Monte Carlo built the same way
    fusion_validation.py's own test_covariance_intersection_correlated
    builds its sources (_as_source over a hand-built track row), just
    swept over a correlation coefficient instead of one fixed case.

Which scenarios/parameters feed which plot (and why) is documented in
each plot function's docstring below - see those for specifics instead
of re-deriving the mapping from first principles.

Run from the simulation_prototype/ folder:
    python generate_final_dependability_plots.py
Optional flags:
    --config  simulation_config.json         (input config)
    --outdir  plots/dependability            (output folder)
    --seeds   5                              (seeds per data point)
    --only    calibration_reliability,...    (comma list of plot names to run)
"""

import argparse
import copy
import json
import math
import os
import statistics
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from generate_plots import (
    ADV_COLOR, ADV_ACCENT, ADV_PALETTE, _adv_style, mean_ci,
    line_ci_plot, bar_ci_plot, gather_rows,
)
from metrics_analysis import run_once
from calibration.radar_calibration_analysis import analyze_scenario as radar_calibration_analyze
from calibration.radar_confidence_calibration import _plot_reliability
from dependability_common import clone_scenario
from dependability_controllers import (
    attach_dependability_layer, _dependability_metrics, run_controller,
)
from simple_swarm_sim import Simulation
from models.radar_like_model import RadarLikeModel
from fusion.fusion_model import (
    fuse_group, _as_source, COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION,
)
from dependability.selective_swarm_decision import SelectiveDecisionMaker

RESULT_RECORD = []  # (plot_name, output_path) for every plot actually written


def _note(name, path):
    RESULT_RECORD.append({"plot": name, "path": path})
    print(f"  -> {path}")


# ---------------------------------------------------------------------
# 1. calibration reliability diagram
# ---------------------------------------------------------------------
def plot_calibration_reliability_diagram(config, outdir, seeds):
    """Detection-probability reliability diagrams (Task 23/comparison-1's
    own analysis) for the calibrated radar plus its two overconfident
    variants, side by side, using RadarLikeModel's real per-detection
    probability_of_detection vs whether the target was actually detected
    - not confidence_score (see radar_calibration_analysis.py's own
    module docstring for why)."""
    scenarios = ["correctly_calibrated_radar", "mildly_overconfident_radar",
                 "severely_overconfident_radar", "underconfident_radar"]
    base_seed = config["sim"].get("seed", 0)
    runs = max(seeds * 5, 10)  # calibration needs many pooled samples per bin

    fig, axes = plt.subplots(1, len(scenarios), figsize=(4.4 * len(scenarios), 4.2), squeeze=False)
    for ax, scenario in zip(axes[0], scenarios):
        if scenario not in config["scenarios"]:
            ax.set_title(f"{scenario}\n(not in config)")
            continue
        metrics = radar_calibration_analyze(config, scenario, num_runs=runs,
                                             base_seed=base_seed, num_bins=10)
        _plot_reliability(ax, metrics, scenario.replace("_radar", "").replace("_", " "))

    fig.suptitle("Radar Confidence Calibration - Reliability Diagrams", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    path = os.path.join(outdir, "calibration_reliability_diagram.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("calibration_reliability_diagram", path)


# ---------------------------------------------------------------------
# 2. calibration error vs collision risk
# ---------------------------------------------------------------------
def plot_calibration_error_vs_collision_risk(config, outdir, seeds):
    """Does worse radar confidence calibration actually translate into
    more collision risk? Uses the same four confidence scenarios as
    plot 1 for ECE, and the full radar/track/fusion/decision pipeline
    (metrics_analysis.run_once) on those same scenarios for
    collision_risk_count."""
    scenarios = ["underconfident_radar", "correctly_calibrated_radar",
                 "mildly_overconfident_radar", "severely_overconfident_radar"]
    base_seed = config["sim"].get("seed", 0)
    ece_runs = max(seeds * 5, 10)

    points = []
    for scenario in scenarios:
        if scenario not in config["scenarios"]:
            continue
        cal = radar_calibration_analyze(config, scenario, num_runs=ece_runs,
                                         base_seed=base_seed, num_bins=10)
        risk_vals = []
        for s in range(seeds):
            try:
                m = run_once(config, scenario, base_seed + s)
                if m.get("collision_risk_count") is not None:
                    risk_vals.append(m["collision_risk_count"])
            except Exception as e:
                print(f"    Warning: run_once failed for {scenario} seed={s}: {e}")
        risk_mean, risk_ci = mean_ci(risk_vals)
        points.append((scenario, cal.get("expected_calibration_error"), risk_mean, risk_ci))

    if not points:
        print("    Warning: no calibration scenarios found, skipping plot 2")
        return

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for (scenario, ece, risk, ci), color in zip(points, ADV_PALETTE):
        if ece is None or risk is None:
            continue
        ax.errorbar([ece], [risk], yerr=[ci], marker="o", markersize=10, color=color,
                    capsize=4, linestyle="none", label=scenario.replace("_radar", ""))
    ax.set_xlabel("Expected Calibration Error (ECE)")
    ax.set_ylabel("Collision risk count")
    ax.set_title("Confidence Calibration Error vs Collision Risk", fontsize=12, fontweight="bold")
    ax.legend(frameon=False, fontsize=9)
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "calibration_error_vs_collision_risk.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("calibration_error_vs_collision_risk", path)


# ---------------------------------------------------------------------
# 3. registration error vs fusion RMSE
# ---------------------------------------------------------------------
def _registration_error_magnitude(scn):
    """Single scalar summarizing a cross_modal_registration block's
    severity: mean radar<->vision / radar<->lidar offset magnitude
    (meters) plus rotation error converted to an equivalent arc-length
    at a nominal 10m sensing range, so both translational and rotational
    misalignment contribute to one comparable number."""
    r = scn.get("cross_modal_registration", {})
    off_v = math.hypot(r.get("radar_to_vision_x_offset", 0.0), r.get("radar_to_vision_y_offset", 0.0))
    off_l = math.hypot(r.get("radar_to_lidar_x_offset", 0.0), r.get("radar_to_lidar_y_offset", 0.0))
    rot_arc = math.radians(r.get("rotation_error_deg", 0.0)) * 10.0
    return round((off_v + off_l) / 2.0 + rot_arc, 4)


def plot_registration_error_vs_fusion_rmse(config, outdir, seeds):
    """Sweeps this project's own registration_perfect/small/medium/severe
    scenarios (Task 20's cross-modal registration-error model) and reads
    fusion_consistency_error (RMSE of fused position vs ground truth)
    back off metrics_analysis.run_once for each."""
    scenario_order = ["registration_perfect", "registration_small_error",
                       "registration_medium_error", "registration_severe_error"]
    xs, means, cis, labels = [], [], [], []
    for scenario in scenario_order:
        if scenario not in config["scenarios"]:
            continue
        x = _registration_error_magnitude(config["scenarios"][scenario])
        vals = []
        for s in range(seeds):
            try:
                m = run_once(config, scenario, config["sim"].get("seed", 0) + s)
                if m.get("fusion_consistency_error") is not None:
                    vals.append(m["fusion_consistency_error"])
            except Exception as e:
                print(f"    Warning: run_once failed for {scenario} seed={s}: {e}")
        m_mean, m_ci = mean_ci(vals)
        xs.append(x)
        means.append(m_mean)
        cis.append(m_ci)
        labels.append(scenario.replace("registration_", ""))

    if not xs:
        print("    Warning: no registration scenarios found, skipping plot 3")
        return

    fig, ax = plt.subplots(figsize=(7, 4.5))
    plot_means = [m if m is not None else 0 for m in means]
    plot_cis = [c if c is not None else 0 for c in cis]
    ax.errorbar(xs, plot_means, yerr=plot_cis, marker="o", color=ADV_COLOR, capsize=4, linewidth=2)
    for x, y, label in zip(xs, plot_means, labels):
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 6), fontsize=8)
    ax.set_xlabel("Registration error magnitude (m-equivalent; offsets + rotation arc-length)")
    ax.set_ylabel("Fusion RMSE vs ground truth (m)")
    ax.set_title("Cross-Modal Registration Error vs Fusion RMSE", fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "registration_error_vs_fusion_rmse.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("registration_error_vs_fusion_rmse", path)


# ---------------------------------------------------------------------
# 4. sensor correlation vs covariance consistency
# ---------------------------------------------------------------------
def _make_correlated_pair(rho, sigma, rng, true_x=0.0, true_y=0.0):
    """Builds two _as_source-shaped sources whose position errors share a
    common component scaled by sqrt(rho) and an independent component
    scaled by sqrt(1-rho) - rho=0 is the textbook-independent case,
    rho=1 is the fully-correlated case fusion_validation.py's own
    test_covariance_intersection_correlated already probes at one fixed
    point. Each sensor still only ever reports its own marginal variance
    (sigma^2 per axis), unaware of the shared component - the realistic
    case where correlated sensor errors go undeclared."""
    shared_x = rng.gauss(0.0, sigma * math.sqrt(rho)) if rho > 0 else 0.0
    shared_y = rng.gauss(0.0, sigma * math.sqrt(rho)) if rho > 0 else 0.0
    sources = []
    for tid, radar_id in (("A", "r1"), ("B", "r2")):
        indep_frac = 1.0 - rho
        ex = shared_x + (rng.gauss(0.0, sigma * math.sqrt(indep_frac)) if indep_frac > 0 else 0.0)
        ey = shared_y + (rng.gauss(0.0, sigma * math.sqrt(indep_frac)) if indep_frac > 0 else 0.0)
        P = [[sigma ** 2, 0.0, 0.0, 0.0],
             [0.0, sigma ** 2, 0.0, 0.0],
             [0.0, 0.0, 25.0, 0.0],
             [0.0, 0.0, 0.0, 25.0]]
        track = {
            "track_id": tid, "radar_id": radar_id,
            "est_x": true_x + ex, "est_y": true_y + ey, "est_vx": 0.0, "est_vy": 0.0,
            "covariance": json.dumps(P), "confidence": 1.0, "age": 10,
            "hit_count": 10, "missed_count": 0, "existence_probability": 0.9,
            "status": "confirmed",
        }
        sources.append(_as_source(track))
    return sources


def plot_sensor_correlation_vs_covariance_consistency(config, outdir, seeds):
    """Monte Carlo (not driven by the simulator - a controlled fusion-math
    experiment, the same style fusion_validation.py's own
    test_covariance_intersection_correlated uses) sweeping the true
    correlation coefficient rho between two sensors' position errors from
    0 (independent) to 1 (fully correlated), and checking each fusion
    mode's covariance consistency: mean squared fused-position error
    divided by the fused position_variance each mode reports. A
    consistency ratio near 1.0 means "reported uncertainty matches actual
    error"; >1 means overconfident (claims tighter precision than it
    actually has); Covariance Intersection is specifically designed to
    stay <=1 (conservative) even as correlation grows, unlike naive
    covariance-weighted (information) fusion."""
    import random
    rng = random.Random(config["sim"].get("seed", 0))
    sigma = 1.0
    trials_per_point = max(seeds * 100, 300)
    rhos = [0.0, 0.25, 0.5, 0.75, 1.0]

    naive_ratio, ci_ratio = [], []
    for rho in rhos:
        naive_sq_err, naive_var, ci_sq_err, ci_var = [], [], [], []
        for _ in range(trials_per_point):
            sources = _make_correlated_pair(rho, sigma, rng)
            naive = fuse_group(sources, COVARIANCE_WEIGHTED_FUSION)
            ci = fuse_group(sources, COVARIANCE_INTERSECTION_FUSION)
            naive_sq_err.append(naive["x"] ** 2 + naive["y"] ** 2)
            naive_var.append(naive["position_variance"])
            ci_sq_err.append(ci["x"] ** 2 + ci["y"] ** 2)
            ci_var.append(ci["position_variance"])
        naive_ratio.append(statistics.mean(naive_sq_err) / statistics.mean(naive_var))
        ci_ratio.append(statistics.mean(ci_sq_err) / statistics.mean(ci_var))

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.axhline(1.0, color="gray", linestyle="--", linewidth=1, label="ideal (consistent)")
    ax.plot(rhos, naive_ratio, marker="o", color=ADV_ACCENT, linewidth=2,
            label="naive covariance-weighted fusion")
    ax.plot(rhos, ci_ratio, marker="o", color=ADV_COLOR, linewidth=2,
            label="Covariance Intersection")
    ax.set_xlabel("True sensor error correlation (rho)")
    ax.set_ylabel("Consistency ratio (actual MSE / reported variance)")
    ax.set_title("Sensor Correlation vs Covariance Consistency", fontsize=12, fontweight="bold")
    ax.legend(frameon=False, fontsize=9)
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "sensor_correlation_vs_covariance_consistency.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("sensor_correlation_vs_covariance_consistency", path)


# ---------------------------------------------------------------------
# 5. Doppler ambiguity vs response error
# ---------------------------------------------------------------------
def plot_doppler_ambiguity_vs_response_error(config, outdir, seeds):
    """Sweeps max_unambiguous_radial_velocity down (with
    doppler_aliasing_enabled=True, the same knobs
    run_final_dependability_experiments.py's ghost/aliasing comparison
    uses) off correctly_calibrated_radar, measuring the resulting
    doppler_ambiguity_flag rate directly off RadarLikeModel's detection
    rows on the x-axis, and the full pipeline's avg_response_time_s
    (how long it takes the swarm to actually react once a velocity
    reading is wrapped/aliased) on the y-axis."""
    base_scenario = "correctly_calibrated_radar"
    if base_scenario not in config["scenarios"]:
        print("    Warning: correctly_calibrated_radar not found, skipping plot 5")
        return
    thresholds = [10.0, 5.0, 2.0, 1.0, 0.5]
    base_seed = config["sim"].get("seed", 0)

    xs, means, cis = [], [], []
    for i, thresh in enumerate(thresholds):
        scenario_name = f"_doppler_sweep_{i}"
        run_config = clone_scenario(
            config, base_scenario, scenario_name,
            {"max_unambiguous_radial_velocity": thresh, "doppler_aliasing_enabled": True},
            description=f"Task 25 sweep point: max_unambiguous_radial_velocity={thresh}")

        alias_rates, response_vals = [], []
        for s in range(seeds):
            seed = base_seed + s
            try:
                run_cfg2 = copy.deepcopy(run_config)
                run_cfg2["sim"]["seed"] = seed
                rows = RadarLikeModel(run_cfg2, scenario_name).run()
                n = len(rows)
                if n:
                    alias_rates.append(sum(1 for r in rows if r.get("doppler_ambiguity_flag")) / n)
                m = run_once(run_config, scenario_name, seed)
                if m.get("avg_response_time_s") is not None:
                    response_vals.append(m["avg_response_time_s"])
            except Exception as e:
                print(f"    Warning: doppler sweep failed at threshold={thresh} seed={s}: {e}")

        x = statistics.mean(alias_rates) if alias_rates else None
        y_mean, y_ci = mean_ci(response_vals)
        if x is not None and y_mean is not None:
            xs.append(x)
            means.append(y_mean)
            cis.append(y_ci)

    if not xs:
        print("    Warning: no data points for plot 5, skipping")
        return

    order = sorted(range(len(xs)), key=lambda i: xs[i])
    xs = [xs[i] for i in order]
    means = [means[i] for i in order]
    cis = [cis[i] for i in order]
    line_ci_plot(xs, means, cis, "Doppler-aliasing rate (fraction of detections wrapped)",
                 "Avg response time (s)", "Doppler Ambiguity Rate vs Response Time",
                 os.path.join(outdir, "doppler_ambiguity_vs_response_error.png"), color=ADV_ACCENT)
    _note("doppler_ambiguity_vs_response_error",
          os.path.join(outdir, "doppler_ambiguity_vs_response_error.png"))


# ---------------------------------------------------------------------
# 6. ghost-return rate vs false-track count
# ---------------------------------------------------------------------
def plot_ghost_return_rate_vs_false_track_count(config, outdir, seeds):
    """Sweeps mean_returns_per_target up (with extended_target_enabled=
    True, the same multipath/extended-target knobs the ghost/aliasing
    comparison uses) off correctly_calibrated_radar, measuring the
    resulting is_extended_return ("ghost return") rate directly off
    RadarLikeModel's rows, and false_track_count from the full tracking
    pipeline (metrics_analysis.perception_metrics, via run_once) for the
    same overrides."""
    base_scenario = "correctly_calibrated_radar"
    if base_scenario not in config["scenarios"]:
        print("    Warning: correctly_calibrated_radar not found, skipping plot 6")
        return
    values = [1.0, 1.5, 2.0, 3.0, 4.0]
    base_seed = config["sim"].get("seed", 0)

    xs, means, cis = [], [], []
    for i, v in enumerate(values):
        scenario_name = f"_ghost_sweep_{i}"
        run_config = clone_scenario(
            config, base_scenario, scenario_name,
            {"extended_target_enabled": True, "mean_returns_per_target": v, "return_spread_std": 0.6},
            description=f"Task 25 sweep point: mean_returns_per_target={v}")

        ghost_rates, false_track_vals = [], []
        for s in range(seeds):
            seed = base_seed + s
            try:
                run_cfg2 = copy.deepcopy(run_config)
                run_cfg2["sim"]["seed"] = seed
                rows = RadarLikeModel(run_cfg2, scenario_name).run()
                n = len(rows)
                if n:
                    ghost_rates.append(sum(1 for r in rows if r.get("is_extended_return")) / n)
                m = run_once(run_config, scenario_name, seed)
                if m.get("false_track_count") is not None:
                    false_track_vals.append(m["false_track_count"])
            except Exception as e:
                print(f"    Warning: ghost-return sweep failed at v={v} seed={s}: {e}")

        x = statistics.mean(ghost_rates) if ghost_rates else None
        y_mean, y_ci = mean_ci(false_track_vals)
        if x is not None and y_mean is not None:
            xs.append(x)
            means.append(y_mean)
            cis.append(y_ci)

    if not xs:
        print("    Warning: no data points for plot 6, skipping")
        return

    order = sorted(range(len(xs)), key=lambda i: xs[i])
    xs = [xs[i] for i in order]
    means = [means[i] for i in order]
    cis = [cis[i] for i in order]
    line_ci_plot(xs, means, cis, "Ghost-return rate (fraction of extended/multipath returns)",
                 "False track count", "Ghost-Return Rate vs False-Track Count",
                 os.path.join(outdir, "ghost_return_rate_vs_false_track_count.png"))
    _note("ghost_return_rate_vs_false_track_count",
          os.path.join(outdir, "ghost_return_rate_vs_false_track_count.png"))


# ---------------------------------------------------------------------
# 7. perception quality vs safety margin
# ---------------------------------------------------------------------
def plot_perception_quality_vs_safety_margin(config, outdir, seeds):
    """Across this project's own four safety-margin scenarios
    (safety_margin_fixed/covariance/confidence/quality_monitor - Task 13),
    plots average track covariance (perception_metrics'
    average_covariance - lower is better perception quality) against the
    mean per-step safety_margin_applied the pipeline actually flew with
    (gathered the same way generate_plots.py's gather_rows already
    collects pipeline rows)."""
    scenarios = ["safety_margin_fixed", "safety_margin_covariance",
                 "safety_margin_confidence", "safety_margin_quality_monitor"]
    base_seed = config["sim"].get("seed", 0)
    points = []
    for scenario in scenarios:
        if scenario not in config["scenarios"]:
            continue
        cov_vals = []
        for s in range(seeds):
            try:
                m = run_once(config, scenario, base_seed + s)
                if m.get("average_covariance") is not None:
                    cov_vals.append(m["average_covariance"])
            except Exception as e:
                print(f"    Warning: run_once failed for {scenario} seed={s}: {e}")
        rows = gather_rows(config, [scenario], seeds)
        margin_vals = [r["safety_margin_applied"] for r in rows if r.get("safety_margin_applied") is not None]
        if cov_vals and margin_vals:
            points.append((scenario, statistics.mean(cov_vals), statistics.mean(margin_vals)))

    if not points:
        print("    Warning: no safety-margin scenarios found, skipping plot 7")
        return

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for (scenario, cov, margin), color in zip(points, ADV_PALETTE):
        ax.scatter([cov], [margin], s=140, color=color, label=scenario.replace("safety_margin_", ""))
    ax.set_xlabel("Average track covariance trace (lower = better perception quality)")
    ax.set_ylabel("Mean safety margin applied (m)")
    ax.set_title("Perception Quality vs Safety Margin", fontsize=12, fontweight="bold")
    ax.legend(frameon=False, fontsize=9)
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "perception_quality_vs_safety_margin.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("perception_quality_vs_safety_margin", path)


# ---------------------------------------------------------------------
# 8/9. abstention threshold vs mission success / mission delay
# ---------------------------------------------------------------------
def _run_abstention_sweep(config, scenario, thresholds, seeds):
    base_seed = config["sim"].get("seed", 0)
    results = []
    for threshold in thresholds:
        successes, delays = [], []
        for s in range(seeds):
            seed = base_seed + s
            try:
                cfg = copy.deepcopy(config)
                cfg["sim"]["seed"] = seed
                sim = Simulation(cfg, scenario)
                attach_dependability_layer(sim, abstention=True, handoff=False)
                sim.abstention_maker = SelectiveDecisionMaker(resume_threshold=threshold)
                metrics = sim.run()
                metrics.update(_dependability_metrics(sim))
                successes.append(bool(metrics["mission_success"]))
                delays.append(sim.abstention_hold_count * sim.dt)
            except Exception as e:
                print(f"    Warning: abstention sweep failed at threshold={threshold} seed={s}: {e}")
        success_rate = (sum(successes) / len(successes)) if successes else None
        delay_mean, delay_ci = mean_ci(delays)
        results.append({"threshold": threshold, "success_rate": success_rate,
                         "delay_mean": delay_mean, "delay_ci": delay_ci})
    return results


def plot_abstention_threshold_vs_mission_success(config, outdir, seeds):
    """Sweeps SelectiveDecisionMaker's resume_threshold (Task 14 - the
    composite-quality score a GOOD reading must clear before an
    abstention episode is considered resolved) on high_dropout, a
    scenario degraded enough for abstention to actually matter, and
    plots the resulting mission_success rate."""
    scenario = "high_dropout" if "high_dropout" in config["scenarios"] else "baseline"
    thresholds = [0.3, 0.45, 0.6, 0.75, 0.9]
    results = _run_abstention_sweep(config, scenario, thresholds, seeds)
    xs = [r["threshold"] for r in results if r["success_rate"] is not None]
    ys = [r["success_rate"] * 100 for r in results if r["success_rate"] is not None]
    if not xs:
        print("    Warning: no data points for plot 8, skipping")
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(xs, ys, marker="o", color=ADV_COLOR, linewidth=2)
    ax.set_xlabel("Abstention resume threshold")
    ax.set_ylabel("Mission success rate (%)")
    ax.set_title(f"Abstention Threshold vs Mission Success ({scenario})", fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "abstention_threshold_vs_mission_success.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("abstention_threshold_vs_mission_success", path)


def plot_abstention_threshold_vs_mission_delay(config, outdir, seeds):
    """Same sweep as plot 8, reporting mission delay instead: seconds
    spent holding position while abstaining (abstention_hold_count * dt)
    - a stricter (higher) resume_threshold should hold longer per
    abstention episode before resuming, at whatever safety benefit that
    buys in plot 8."""
    scenario = "high_dropout" if "high_dropout" in config["scenarios"] else "baseline"
    thresholds = [0.3, 0.45, 0.6, 0.75, 0.9]
    results = _run_abstention_sweep(config, scenario, thresholds, seeds)
    xs = [r["threshold"] for r in results if r["delay_mean"] is not None]
    means = [r["delay_mean"] for r in results if r["delay_mean"] is not None]
    cis = [r["delay_ci"] for r in results if r["delay_mean"] is not None]
    if not xs:
        print("    Warning: no data points for plot 9, skipping")
        return
    line_ci_plot(xs, means, cis, "Abstention resume threshold", "Mission delay from abstention (s)",
                 f"Abstention Threshold vs Mission Delay ({scenario})",
                 os.path.join(outdir, "abstention_threshold_vs_mission_delay.png"), color=ADV_ACCENT)
    _note("abstention_threshold_vs_mission_delay",
          os.path.join(outdir, "abstention_threshold_vs_mission_delay.png"))


# ---------------------------------------------------------------------
# 10/11. handoff strategy vs collision risk / recovery time
# ---------------------------------------------------------------------
def _run_handoff_strategy_sweep(config, scenario, seeds):
    """Task 17's dependability_controllers.CONTROLLERS already encode the
    handoff strategies this project actually implements: no handoff
    (controller 2), handoff on top of uncertainty-aware margins
    (controller 4), and handoff combined with the full dynamic-trust
    radar/track/fusion pipeline (controller 5, run_dynamic_trust_controller
    under the hood via run_controller's own routing... actually invoked
    directly here since run_controller only routes non-dynamic-trust
    controllers - see run_any_controller in dependability_controllers.py
    for the full routing this mirrors for just these three strategies)."""
    from dependability_controllers import run_any_controller
    strategies = {
        "no_handoff": "2_uncertainty_aware",
        "handoff": "4_uncertainty_aware_handoff",
        "handoff_dynamic_trust": "5_dynamic_trust_handoff",
    }
    base_seed = config["sim"].get("seed", 0)
    results = {}
    for label, controller_name in strategies.items():
        risk_vals, recovery_vals = [], []
        for s in range(seeds):
            seed = base_seed + s
            try:
                m = run_any_controller(controller_name, config, scenario, seed)
                risk_vals.append(m.get("near_miss_count", 0) + m.get("collision_count", 0))
                if m.get("recovery_time_s") is not None:
                    recovery_vals.append(m["recovery_time_s"])
            except Exception as e:
                print(f"    Warning: controller {controller_name} failed seed={s}: {e}")
        results[label] = {"risk": risk_vals, "recovery": recovery_vals}
    return results


def plot_handoff_strategy_vs_collision_risk(config, outdir, seeds):
    scenario = "high_dropout" if "high_dropout" in config["scenarios"] else "baseline"
    results = _run_handoff_strategy_sweep(config, scenario, seeds)
    labels = list(results.keys())
    means, cis = [], []
    for label in labels:
        m, c = mean_ci(results[label]["risk"])
        means.append(m)
        cis.append(c)
    if not any(m is not None for m in means):
        print("    Warning: no data points for plot 10, skipping")
        return
    bar_ci_plot(labels, means, cis, "Near-miss + collision count",
                f"Handoff Strategy vs Collision Risk ({scenario})",
                os.path.join(outdir, "handoff_strategy_vs_collision_risk.png"))
    _note("handoff_strategy_vs_collision_risk",
          os.path.join(outdir, "handoff_strategy_vs_collision_risk.png"))


def plot_handoff_strategy_vs_recovery_time(config, outdir, seeds):
    """recovery_time_s only exists for strategies with a handoff model
    attached (PerceptionHandoffModel.summary()'s avg_resolved_duration_
    steps * dt - see dependability_controllers._dependability_metrics);
    no_handoff has nothing to recover from in this sense and is plotted
    as 0 with that noted in the title."""
    scenario = "high_dropout" if "high_dropout" in config["scenarios"] else "baseline"
    results = _run_handoff_strategy_sweep(config, scenario, seeds)
    labels = list(results.keys())
    means, cis = [], []
    for label in labels:
        m, c = mean_ci(results[label]["recovery"])
        means.append(m if m is not None else 0.0)
        cis.append(c)
    bar_ci_plot(labels, means, cis, "Mean recovery time (s)",
                f"Handoff Strategy vs Recovery Time ({scenario})\n"
                "no_handoff has no handoff episodes to recover from (plotted as 0)",
                os.path.join(outdir, "handoff_strategy_vs_recovery_time.png"))
    _note("handoff_strategy_vs_recovery_time",
          os.path.join(outdir, "handoff_strategy_vs_recovery_time.png"))


# ---------------------------------------------------------------------
# 12. combined fault severity vs mission success
# ---------------------------------------------------------------------
def plot_combined_fault_severity_vs_mission_success(config, outdir, seeds):
    """Same arms as run_final_dependability_experiments.py's comparison
    #8 (single_low_pd/high_pfa/high_dropout/high_latency vs
    combined_all_faults which stacks every one of them at once), plotted
    against how many simultaneous fault conditions each arm actually
    stacks (1 for every single-fault arm, 4 for the combined one), with
    baseline (0 faults) included as the reference point."""
    arms = [
        ("baseline", 0), ("very_low_P_D", 1), ("very_high_P_FA", 1),
        ("high_dropout", 1), ("high_latency", 1), ("simultaneous_sensor_failures", 4),
    ]
    base_seed = config["sim"].get("seed", 0)
    xs, ys, labels = [], [], []
    for scenario, severity in arms:
        if scenario not in config["scenarios"]:
            continue
        successes = []
        for s in range(seeds):
            try:
                m = run_once(config, scenario, base_seed + s)
                successes.append(bool(m["mission_success"]))
            except Exception as e:
                print(f"    Warning: run_once failed for {scenario} seed={s}: {e}")
        if successes:
            xs.append(severity)
            ys.append(100.0 * sum(successes) / len(successes))
            labels.append(scenario)

    if not xs:
        print("    Warning: no data points for plot 12, skipping")
        return

    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    colors = [ADV_PALETTE[i % len(ADV_PALETTE)] for i in range(len(xs))]
    ax.scatter(xs, ys, s=110, color=colors)
    for x, y, label in zip(xs, ys, labels):
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=8, rotation=10)
    ax.set_xlabel("Number of simultaneous fault conditions stacked")
    ax.set_xticks(sorted(set(xs)))
    ax.set_ylabel("Mission success rate (%)")
    ax.set_title("Combined Fault Severity vs Mission Success", fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    path = os.path.join(outdir, "combined_fault_severity_vs_mission_success.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("combined_fault_severity_vs_mission_success", path)


# ---------------------------------------------------------------------
# 13/14. swarm size vs communication load / simulation runtime
# ---------------------------------------------------------------------
def _grid_start_positions(n, width, height, margin=5.0):
    """Evenly spaced grid of n start positions within [margin, dim-margin]
    on each axis - the same corner-cluster idea simulation_config.json's
    own 4-UAV start_positions uses, generalized to arbitrary n so swarm
    size can be swept without hand-authoring positions for every n."""
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    xs = np.linspace(margin, width - margin, cols) if cols > 1 else [width / 2.0]
    ys = np.linspace(margin, height - margin, rows) if rows > 1 else [height / 2.0]
    positions = [[float(x), float(y)] for y in ys for x in xs]
    return positions[:n]


def _swarm_size_sweep(config, scenario, sizes, seeds):
    base_seed = config["sim"].get("seed", 0)
    width, height = config["world"]["width"], config["world"]["height"]
    comm_load, runtime = {}, {}
    for n in sizes:
        loads, times = [], []
        for s in range(seeds):
            seed = base_seed + s
            try:
                cfg = copy.deepcopy(config)
                cfg["swarm"]["num_uavs"] = n
                cfg["swarm"]["start_positions"] = _grid_start_positions(n, width, height)
                t0 = time.perf_counter()
                m = run_once(cfg, scenario, seed)
                elapsed = time.perf_counter() - t0
                if m.get("communication_load") is not None:
                    loads.append(m["communication_load"])
                times.append(elapsed)
            except Exception as e:
                print(f"    Warning: swarm-size sweep failed at n={n} seed={s}: {e}")
        comm_load[n] = mean_ci(loads)
        runtime[n] = mean_ci(times)
    return comm_load, runtime


def plot_swarm_size_vs_communication_load(config, outdir, seeds):
    """Sweeps swarm.num_uavs (with a generated grid of start positions
    covering the world so larger swarms don't all spawn on top of each
    other) on naive_fusion - a fusion-enabled scenario so
    communication_metrics.communication_load is actually nonzero -
    reading it back off metrics_analysis.run_once."""
    scenario = "naive_fusion" if "naive_fusion" in config["scenarios"] else "baseline"
    sizes = [2, 4, 6, 8, 10]
    comm_load, _ = _swarm_size_sweep(config, scenario, sizes, seeds)
    xs = [n for n in sizes if comm_load[n][0] is not None]
    means = [comm_load[n][0] for n in xs]
    cis = [comm_load[n][1] for n in xs]
    if not xs:
        print("    Warning: no data points for plot 13, skipping")
        return
    line_ci_plot(xs, means, cis, "Swarm size (num UAVs)", "Mean communication load (messages/run)",
                 f"Swarm Size vs Communication Load ({scenario})",
                 os.path.join(outdir, "swarm_size_vs_communication_load.png"))
    _note("swarm_size_vs_communication_load",
          os.path.join(outdir, "swarm_size_vs_communication_load.png"))


def plot_swarm_size_vs_simulation_runtime(config, outdir, seeds):
    """Same sweep as plot 13, reporting wall-clock seconds per
    metrics_analysis.run_once() call instead - the full radar/track/
    fusion/decision pipeline's actual compute cost as swarm size grows."""
    scenario = "naive_fusion" if "naive_fusion" in config["scenarios"] else "baseline"
    sizes = [2, 4, 6, 8, 10]
    _, runtime = _swarm_size_sweep(config, scenario, sizes, seeds)
    xs = [n for n in sizes if runtime[n][0] is not None]
    means = [runtime[n][0] for n in xs]
    cis = [runtime[n][1] for n in xs]
    if not xs:
        print("    Warning: no data points for plot 14, skipping")
        return
    line_ci_plot(xs, means, cis, "Swarm size (num UAVs)", "Wall-clock seconds per run",
                 f"Swarm Size vs Simulation Runtime ({scenario})",
                 os.path.join(outdir, "swarm_size_vs_simulation_runtime.png"), color=ADV_ACCENT)
    _note("swarm_size_vs_simulation_runtime",
          os.path.join(outdir, "swarm_size_vs_simulation_runtime.png"))


# ---------------------------------------------------------------------
# 15. final failure-envelope heatmap
# ---------------------------------------------------------------------
def plot_failure_envelope_heatmap(config, outdir, seeds):
    """The project's own two clearest independent fault axes - detection
    probability (P_D) and radar dropout probability - swept jointly on
    baseline, with mission_success rate as the heatmap value: the
    "failure envelope" is the boundary in this 2D grid where mission
    success rate collapses."""
    if "baseline" not in config["scenarios"]:
        print("    Warning: baseline scenario not found, skipping plot 15")
        return
    pd_values = [1.0, 0.7, 0.5, 0.3, 0.1]  # rows, best -> worst (top -> bottom)
    dropout_values = [0.0, 0.1, 0.2, 0.3, 0.4]  # columns, best -> worst (left -> right)
    base_seed = config["sim"].get("seed", 0)
    n_cells_seeds = max(seeds, 3)

    grid = np.full((len(pd_values), len(dropout_values)), np.nan)
    for i, pd in enumerate(pd_values):
        for j, dropout in enumerate(dropout_values):
            scenario_name = f"_envelope_{i}_{j}"
            run_config = clone_scenario(
                config, "baseline", scenario_name,
                {"radar_detection_probability": pd, "radar_dropout_probability": dropout},
                description=f"Task 25 failure-envelope grid point: P_D={pd}, dropout={dropout}")
            successes = []
            for s in range(n_cells_seeds):
                try:
                    m = run_once(run_config, scenario_name, base_seed + s)
                    successes.append(bool(m["mission_success"]))
                except Exception as e:
                    print(f"    Warning: envelope cell (P_D={pd}, dropout={dropout}) seed={s} failed: {e}")
            if successes:
                grid[i, j] = 100.0 * sum(successes) / len(successes)

    fig, ax = plt.subplots(figsize=(7.5, 6))
    im = ax.imshow(grid, cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
    ax.set_xticks(range(len(dropout_values)))
    ax.set_xticklabels(dropout_values)
    ax.set_yticks(range(len(pd_values)))
    ax.set_yticklabels(pd_values)
    ax.set_xlabel("Radar dropout probability")
    ax.set_ylabel("Detection probability (P_D)")
    ax.set_title("Final Failure-Envelope Heatmap\nMission success rate (%) vs combined sensor faults",
                 fontsize=12, fontweight="bold")
    for i in range(len(pd_values)):
        for j in range(len(dropout_values)):
            if not np.isnan(grid[i, j]):
                ax.text(j, i, f"{grid[i, j]:.0f}", ha="center", va="center",
                        color="black", fontsize=9)
    fig.colorbar(im, ax=ax, label="Mission success rate (%)")
    fig.tight_layout()
    path = os.path.join(outdir, "final_failure_envelope_heatmap.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    _note("final_failure_envelope_heatmap", path)


# ---------------------------------------------------------------------
# main
# ---------------------------------------------------------------------
ALL_PLOTS = [
    ("calibration_reliability", plot_calibration_reliability_diagram),
    ("calibration_vs_collision_risk", plot_calibration_error_vs_collision_risk),
    ("registration_vs_fusion_rmse", plot_registration_error_vs_fusion_rmse),
    ("sensor_correlation_vs_consistency", plot_sensor_correlation_vs_covariance_consistency),
    ("doppler_vs_response_error", plot_doppler_ambiguity_vs_response_error),
    ("ghost_return_vs_false_track", plot_ghost_return_rate_vs_false_track_count),
    ("perception_quality_vs_safety_margin", plot_perception_quality_vs_safety_margin),
    ("abstention_vs_mission_success", plot_abstention_threshold_vs_mission_success),
    ("abstention_vs_mission_delay", plot_abstention_threshold_vs_mission_delay),
    ("handoff_vs_collision_risk", plot_handoff_strategy_vs_collision_risk),
    ("handoff_vs_recovery_time", plot_handoff_strategy_vs_recovery_time),
    ("combined_fault_severity", plot_combined_fault_severity_vs_mission_success),
    ("swarm_size_vs_comm_load", plot_swarm_size_vs_communication_load),
    ("swarm_size_vs_runtime", plot_swarm_size_vs_simulation_runtime),
    ("failure_envelope_heatmap", plot_failure_envelope_heatmap),
]


def main():
    parser = argparse.ArgumentParser(description="Task 25: generate the final dependability plots")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--outdir", default=os.path.join(_ROOT_DIR, "plots", "dependability"))
    parser.add_argument("--seeds", type=int, default=5, help="Seeds per sweep data point")
    parser.add_argument("--only", default=None,
                         help="Comma-separated subset of plot names to run (see ALL_PLOTS keys)")
    args = parser.parse_args()

    if args.seeds < 1:
        sys.exit(f"--seeds must be >= 1 (got {args.seeds})")

    with open(args.config) as f:
        config = json.load(f)

    os.makedirs(args.outdir, exist_ok=True)

    wanted = set(args.only.split(",")) if args.only else None
    for name, plot_fn in ALL_PLOTS:
        if wanted is not None and name not in wanted:
            continue
        print(f"[{name}]")
        try:
            plot_fn(config, args.outdir, args.seeds)
        except Exception as e:
            print(f"    Warning: {plot_fn.__name__} failed: {e}")
            import traceback
            traceback.print_exc()

    manifest_path = os.path.join(args.outdir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(RESULT_RECORD, f, indent=2)

    print(f"\nSaved {len(RESULT_RECORD)}/{len(ALL_PLOTS)} final dependability plots to {args.outdir}/")
    return 0 if len(RESULT_RECORD) == len(ALL_PLOTS) else 1


if __name__ == "__main__":
    sys.exit(main())
