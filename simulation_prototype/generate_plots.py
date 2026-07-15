"""
Generate graphs summarizing simulation results.

Basic set - reads the per-run metrics produced by metrics_analysis.py and
saves six PNG charts into plots/:

  1. false_positive_vs_unnecessary_avoidance.png
  2. false_negative_vs_collision_risk.png
  3. latency_vs_response_time.png
  4. dropout_vs_mission_success.png
  5. baseline_vs_error_scenarios.png
  6. fusion_mode_vs_safety_metrics.png

Advanced set - parameter sweeps and comparisons that need data
results_summary.csv doesn't carry (per-detection range/SNR, architecture/
trust/fusion-mode comparisons), so these drive the simulation/fusion
pipeline directly instead. Saved into plots/advanced/:

  P_D vs track continuity, P_FA vs false track count, clutter vs fusion
  error, range vs position RMSE, SNR vs measurement error, latency vs
  response time, packet loss vs mission success, trust error vs collision
  risk, fusion mode vs {position RMSE, collision risk, mission success},
  centralized vs distributed fusion, static vs dynamic trust, tracking
  performance by scenario, confidence intervals for major results.

Run from the simulation_prototype/ folder:
    python generate_plots.py
Optional flags:
    --summary results/results_summary.csv   (basic-set input file)
    --outdir  plots                         (basic-set output folder)
    --config  simulation_config.json        (advanced-set input config)
    --advanced-outdir plots/advanced        (advanced-set output folder)
    --seeds   4                             (seeds per advanced data point)
    --skip-advanced                         (basic set only)
"""

import argparse
import copy
import json
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from simple_swarm_sim import Simulation, run_radar_track_fusion_pipeline
from fusion_model import (
    build_fused_log, estimation_error_against_ground_truth,
    ARCHITECTURE_CENTRALIZED, ARCHITECTURE_DISTRIBUTED, FUSION_MODES,
)
from metrics_analysis import run_once


SCENARIO_ORDER = [
    "baseline", "false_positive", "false_negative",
    "sensor_noise", "latency", "sensor_dropout",
    "confidence_error", "no_fusion_matched", "naive_fusion", "trust_weighted_fusion",
]
COLORS = {
    "baseline": "#4B5694",
    "false_positive": "#C2554A",
    "false_negative": "#D98C3D",
    "sensor_noise": "#7288AE",
    "latency": "#6E9075",
    "sensor_dropout": "#8B5FA8",
    "confidence_error": "#B08968",
    "no_fusion_matched": "#A6763D",
    "naive_fusion": "#5FA88B",
    "trust_weighted_fusion": "#111844",
}


def load(summary_path):
    df = pd.read_csv(summary_path)
    df["mission_success_bool"] = df["mission_success"].map({"Yes": 1, "No": 0})
    return df


def per_run_bar(df, scenario, value_col, title, ylabel, out_path, baseline_df=None):
    sub = df[df["scenario"] == scenario].sort_values("run_number")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = sub["run_number"].astype(str)
    y = sub[value_col]

    missing = y.isna()
    y_plot = y.fillna(0)
    bars = ax.bar(x, y_plot, color=COLORS.get(scenario, "#4B5694"), width=0.55,
                   label=scenario.replace("_", " "))

    for bar, was_missing in zip(bars, missing):
        if was_missing:
            bar.set_hatch("//")
            bar.set_edgecolor("white")

    if baseline_df is not None:
        base_mean = baseline_df[value_col].mean()
        ax.axhline(base_mean, color="#111844", linestyle="--", linewidth=1.5,
                    label=f"baseline avg ({base_mean:.2f})")

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Trial number (fixed parameter value - see title)")
    ax.set_ylabel(ylabel)
    ax.legend(frameon=False, fontsize=9)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def dropout_mission_success(df, out_path):
    sub = df[df["scenario"] == "sensor_dropout"].sort_values("run_number")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = sub["run_number"].astype(str)
    colors = ["#4B5694" if v == 1 else "#C2554A" for v in sub["mission_success_bool"]]
    ax.bar(x, [1] * len(sub), color=colors, width=0.55)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Failed", "Succeeded"])
    ax.set_ylim(0, 1.2)
    ax.set_xlabel("Run number")
    dropout_p = sub["dropout_probability"].iloc[0] if len(sub) else "?"
    ax.set_title(f"Mission Success at Fixed Dropout Probability = {dropout_p}\n(5 trials, not a probability sweep)",
                 fontsize=12, fontweight="bold")
    success_rate = sub["mission_success_bool"].mean() * 100
    ax.text(0.5, 1.08, f"Success rate: {success_rate:.0f}% ({int(sub['mission_success_bool'].sum())}/{len(sub)} runs)",
            transform=ax.transAxes, ha="center", fontsize=10, color="#111844")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def baseline_vs_all(df, out_path):
    metrics = [
        ("collision_risk_count", "Avg collision risk count"),
        ("unnecessary_avoidance_count", "Avg unnecessary avoidance"),
        ("missed_response_count", "Avg missed response"),
        ("mission_success_bool", "Mission success rate"),
    ]
    scenarios = [s for s in SCENARIO_ORDER if s in df["scenario"].unique()]
    agg = df.groupby("scenario")[[m[0] for m in metrics]].mean().reindex(scenarios)

    fig, axes = plt.subplots(1, len(metrics), figsize=(16, 4.5), sharex=False)
    for ax, (col, label) in zip(axes, metrics):
        vals = agg[col]
        if col == "mission_success_bool":
            vals = vals * 100
            ax.set_ylabel("%")
        else:
            ax.set_ylabel("count")
        bars = ax.bar(scenarios, vals, color=[COLORS.get(s, "#7288AE") for s in scenarios])
        for b, s in zip(bars, scenarios):
            if s == "baseline":
                b.set_edgecolor("#111844")
                b.set_linewidth(2)
        ax.set_title(label, fontsize=10, fontweight="bold")
        ax.set_xticks(range(len(scenarios)))
        ax.set_xticklabels(scenarios, rotation=40, ha="right", fontsize=8)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle("Baseline vs Error Scenarios - Average Across Runs", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def fusion_mode_comparison(df, out_path):
    fusion_scenarios = ["no_fusion_matched", "naive_fusion", "trust_weighted_fusion"]
    labels = ["no_fusion", "naive_fusion", "trust_weighted_fusion"]
    present = [s for s in fusion_scenarios if s in df["scenario"].unique()]
    if not present:
        print("Warning: No fusion comparison scenarios found in data")
        return
    
    available_cols = df.columns.tolist()
    metric_cols = []
    for col in ["collision_risk_count", "missed_response_count", "fusion_recovery_count"]:
        if col in available_cols:
            metric_cols.append(col)
        else:
            print(f"Warning: Column '{col}' not found in data - skipping")
    
    if not metric_cols:
        print("Error: No metric columns available for fusion comparison")
        return
    
    agg = df[df["scenario"].isin(present)].groupby("scenario")[metric_cols].mean().reindex(present)

    n_metrics = len(metric_cols)
    fig, axes = plt.subplots(1, max(1, n_metrics), figsize=(max(4.5, 4.5 * n_metrics), 4.5))
    if n_metrics == 1:
        axes = [axes]
    
    metric_labels = {
        "collision_risk_count": "Avg collision risk count",
        "missed_response_count": "Avg missed response count",
        "fusion_recovery_count": "Avg fusion-recovered detections",
    }
    
    for ax, col in zip(axes, metric_cols):
        bars = ax.bar(range(len(present)), agg[col], color=[COLORS.get(s, "#7288AE") for s in present])
        ax.set_xticks(range(len(present)))
        ax.set_xticklabels([labels[fusion_scenarios.index(s)] for s in present], fontsize=8)
        ax.set_title(metric_labels.get(col, col), fontsize=10, fontweight="bold")
        ax.set_ylabel("count")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle("Fusion Mode Comparison - Average Across Runs", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# --- advanced set ---

ADV_COLOR = "#4B5694"
ADV_ACCENT = "#C2554A"
ADV_PALETTE = ["#4B5694", "#C2554A", "#D98C3D", "#6E9075", "#8B5FA8", "#111844"]
FUSION_SUCCESS_THRESHOLD = 5.0


def _adv_style(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def mean_ci(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, 0.0
    m = statistics.mean(vals)
    if len(vals) < 2:
        return m, 0.0
    return m, 1.96 * statistics.stdev(vals) / math.sqrt(len(vals))


def line_ci_plot(x, means, cis, xlabel, ylabel, title, out_path, color=ADV_COLOR):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.errorbar(x, means, yerr=cis, marker="o", color=color, capsize=4, linewidth=2)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def bar_ci_plot(labels, means, cis, ylabel, title, out_path, colors=None):
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    colors = colors or ADV_PALETTE[:len(labels)]
    plot_means = [m if m is not None else 0 for m in means]
    plot_cis = [c if c is not None else 0 for c in cis]
    ax.bar(labels, plot_means, yerr=plot_cis, capsize=4, color=colors)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    _adv_style(ax)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def sweep_metric(config, base_scenario, overrides_list, seeds, metric_key):
    means, cis = [], []
    for overrides in overrides_list:
        vals = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                # Make sure the scenario exists
                if base_scenario not in run_config["scenarios"]:
                    print(f"Warning: Scenario '{base_scenario}' not found, skipping")
                    continue
                run_config["scenarios"][base_scenario].update(overrides)
                result = run_once(run_config, base_scenario, s)
                val = result.get(metric_key)
                if val is not None:
                    vals.append(val)
            except Exception as e:
                print(f"Warning: Error in sweep for {base_scenario}, seed={s}: {e}")
                continue
        m, ci = mean_ci(vals)
        means.append(m)
        cis.append(ci)
    return means, cis


def gather_rows(config, scenarios, seeds):
    all_rows = []
    for name in scenarios:
        if name not in config["scenarios"]:
            print(f"Warning: Scenario '{name}' not found, skipping")
            continue
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                run_config["sim"]["seed"] = s
                rows, _ = run_radar_track_fusion_pipeline(run_config, name)
                all_rows.extend(rows)
            except Exception as e:
                print(f"Warning: Error gathering rows for {name}, seed={s}: {e}")
                continue
    return all_rows


def bin_rmse(pairs, n_bins):
    if not pairs:
        return [], []
    xs = [p[0] for p in pairs if p[0] is not None]
    if not xs:
        return [], []
    lo, hi = min(xs), max(xs)
    width = (hi - lo) / n_bins if hi > lo else 1.0
    buckets = [[] for _ in range(n_bins)]
    for x, e in pairs:
        if x is None or e is None:
            continue
        idx = min(n_bins - 1, max(0, int((x - lo) / width)))
        buckets[idx].append(e)
    centers, rmse = [], []
    for i, b in enumerate(buckets):
        if not b:
            continue
        centers.append(lo + width * (i + 0.5))
        rmse.append(math.sqrt(statistics.mean(e * e for e in b)))
    return centers, rmse


def obstacle_xy(config, scenario):
    sim = Simulation(config, scenario)
    return (sim.obstacle[0], sim.obstacle[1])


def plot_pd_vs_continuity(config, outdir, seeds):
    values = [0.1, 0.3, 0.5, 0.7, 1.0]
    overrides = [{"radar_detection_probability": v} for v in values]
    means, cis = sweep_metric(config, "baseline", overrides, seeds, "track_continuity")
    line_ci_plot(values, means, cis, "Detection probability (P_D)", "Track continuity",
                 "P_D vs Track Continuity", os.path.join(outdir, "pd_vs_track_continuity.png"))


def plot_pfa_vs_false_track(config, outdir, seeds):
    values = [0.0, 0.1, 0.2, 0.3, 0.4]
    overrides = [{"radar_false_alarm_probability": v} for v in values]
    means, cis = sweep_metric(config, "baseline", overrides, seeds, "false_track_count")
    line_ci_plot(values, means, cis, "False alarm probability (P_FA)", "False track count",
                 "P_FA vs False Track Count", os.path.join(outdir, "pfa_vs_false_track_count.png"))


def plot_clutter_vs_fusion_error(config, outdir, seeds):
    values = [0.0, 0.1, 0.2, 0.3, 0.5]
    overrides = [{"radar_clutter_density": v} for v in values]
    means, cis = sweep_metric(config, "naive_fusion", overrides, seeds, "fusion_consistency_error")
    line_ci_plot(values, means, cis, "Clutter intensity", "Fusion consistency error",
                 "Clutter Intensity vs Fusion Error", os.path.join(outdir, "clutter_vs_fusion_error.png"))


def plot_range_vs_rmse(config, outdir, seeds):
    rows = gather_rows(config, ["baseline", "sensor_noise", "env_low_visibility"], seeds)
    pairs = [(r.get("measured_range"), math.hypot(r.get("detected_x", 0) - r.get("true_target_x", 0), 
                                                   r.get("detected_y", 0) - r.get("true_target_y", 0)))
             for r in rows if r.get("measured_range") is not None and r.get("detected_x") is not None]
    centers, rmse = bin_rmse(pairs, 8)
    if not centers:
        print("Warning: No data for range vs RMSE plot")
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(centers, rmse, marker="o", color=ADV_COLOR, linewidth=2)
    ax.set_xlabel("Measured range")
    ax.set_ylabel("Position RMSE")
    ax.set_title("Range vs Position RMSE", fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "range_vs_position_rmse.png"), dpi=150)
    plt.close(fig)


def plot_snr_vs_error(config, outdir, seeds):
    rows = gather_rows(config, ["baseline", "sensor_noise", "env_low_visibility"], seeds)
    pairs = [(r.get("radar_snr"), math.hypot(r.get("detected_x", 0) - r.get("true_target_x", 0),
                                               r.get("detected_y", 0) - r.get("true_target_y", 0)))
             for r in rows if r.get("radar_snr") is not None and r.get("detected_x") is not None]
    centers, rmse = bin_rmse(pairs, 8)
    if not centers:
        print("Warning: No data for SNR vs error plot")
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(centers, rmse, marker="o", color=ADV_ACCENT, linewidth=2)
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("Measurement error")
    ax.set_title("SNR vs Measurement Error", fontsize=12, fontweight="bold")
    _adv_style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "snr_vs_measurement_error.png"), dpi=150)
    plt.close(fig)


def plot_latency_vs_response(config, outdir, seeds):
    values = [0, 5, 10, 15, 20]
    overrides = [{"radar_latency_steps": v} for v in values]
    means, cis = sweep_metric(config, "baseline", overrides, seeds, "avg_response_time_s")
    line_ci_plot(values, means, cis, "Latency (steps)", "Avg response time (s)",
                 "Latency vs Response Time", os.path.join(outdir, "latency_vs_response_time.png"))


def plot_packet_loss_vs_mission_success(config, outdir, seeds):
    scenario = "no_fusion_matched"
    if scenario not in config["scenarios"]:
        print(f"Warning: Scenario '{scenario}' not found - using baseline")
        scenario = "baseline"
    gt = obstacle_xy(config, scenario)
    values = [0.0, 0.1, 0.3, 0.5, 0.7, 1.0]
    means, cis = [], []
    for p in values:
        successes = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                if "communication" not in run_config["scenarios"][scenario]:
                    run_config["scenarios"][scenario]["communication"] = {}
                run_config["scenarios"][scenario]["communication"]["packet_loss_probability"] = p
                rows = build_fused_log(scenario, run_config, architecture=ARCHITECTURE_DISTRIBUTED, seed=s)
                err = estimation_error_against_ground_truth(rows, gt)["mean_error"]
                successes.append(1.0 if (err is not None and err < FUSION_SUCCESS_THRESHOLD) else 0.0)
            except Exception as e:
                print(f"Warning: Error in packet_loss plot for p={p}, seed={s}: {e}")
                continue
        if successes:
            m, ci = mean_ci(successes)
            means.append((m or 0) * 100)
            cis.append(ci * 100 if ci is not None else 0)
        else:
            means.append(0)
            cis.append(0)
    line_ci_plot(values, means, cis, "Packet loss probability", "Fusion-accurate success rate (%)",
                 "Packet Loss vs Mission Success", os.path.join(outdir, "packet_loss_vs_mission_success.png"))


def plot_trust_error_vs_collision_risk(config, outdir, seeds):
    scenario = "faulty_sensor_trust_weighted_fusion_dynamic"
    if scenario not in config["scenarios"]:
        print(f"Warning: Scenario '{scenario}' not found - using baseline")
        scenario = "baseline"
    values = [0.0, 1.5, 3.5, 5.0, 7.0]
    overrides = [{"faulty_position_bias": [v, 0.0]} for v in values]
    means, cis = sweep_metric(config, scenario, overrides, seeds, "collision_risk_count")
    line_ci_plot(values, means, cis, "Faulty-sensor position bias (trust error)", "Collision risk count",
                 "Trust Error vs Collision Risk", os.path.join(outdir, "trust_error_vs_collision_risk.png"),
                 color=ADV_ACCENT)


def _fusion_mode_metric(config, metric_key, seeds):
    scenario = "no_fusion_matched"
    if scenario not in config["scenarios"]:
        print(f"Warning: Scenario '{scenario}' not found - using baseline")
        scenario = "baseline"
    means, cis = [], []
    for mode in FUSION_MODES:
        vals = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                run_config["scenarios"][scenario]["fusion_mode"] = mode
                result = run_once(run_config, scenario, s)
                val = result.get(metric_key)
                if val is not None:
                    vals.append(val)
            except Exception as e:
                print(f"Warning: Error in fusion_mode plot for mode={mode}, seed={s}: {e}")
                continue
        m, ci = mean_ci(vals)
        means.append(m)
        cis.append(ci)
    return list(FUSION_MODES), means, cis


def plot_fusion_mode_vs_rmse(config, outdir, seeds):
    labels, means, cis = _fusion_mode_metric(config, "fusion_consistency_error", seeds)
    bar_ci_plot(labels, means, cis, "Position RMSE", "Fusion Mode vs Position RMSE",
                os.path.join(outdir, "fusion_mode_vs_position_rmse.png"))


def plot_fusion_mode_vs_collision_risk(config, outdir, seeds):
    labels, means, cis = _fusion_mode_metric(config, "collision_risk_count", seeds)
    bar_ci_plot(labels, means, cis, "Collision risk count", "Fusion Mode vs Collision Risk",
                os.path.join(outdir, "fusion_mode_vs_collision_risk.png"))


def plot_fusion_mode_vs_mission_success(config, outdir, seeds):
    scenario = "no_fusion_matched"
    if scenario not in config["scenarios"]:
        print(f"Warning: Scenario '{scenario}' not found - using baseline")
        scenario = "baseline"
    means, cis = [], []
    for mode in FUSION_MODES:
        vals = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                run_config["scenarios"][scenario]["fusion_mode"] = mode
                result = run_once(run_config, scenario, s)
                val = 1.0 if result.get("mission_success") else 0.0
                vals.append(val)
            except Exception as e:
                print(f"Warning: Error in fusion_mode mission_success plot for mode={mode}, seed={s}: {e}")
                continue
        if vals:
            m, ci = mean_ci(vals)
            means.append((m or 0) * 100)
            cis.append(ci * 100 if ci is not None else 0)
        else:
            means.append(0)
            cis.append(0)
    bar_ci_plot(list(FUSION_MODES), means, cis, "Mission success rate (%)",
                "Fusion Mode vs Mission Success", os.path.join(outdir, "fusion_mode_vs_mission_success.png"))


def plot_architecture_comparison(config, outdir, seeds):
    scenario = "no_fusion_matched"
    if scenario not in config["scenarios"]:
        print(f"Warning: Scenario '{scenario}' not found - using baseline")
        scenario = "baseline"
    gt = obstacle_xy(config, scenario)
    labels, means, cis = [], [], []
    for arch in (ARCHITECTURE_CENTRALIZED, ARCHITECTURE_DISTRIBUTED):
        errs = []
        for s in range(seeds):
            try:
                rows = build_fused_log(scenario, config, architecture=arch, seed=s)
                err = estimation_error_against_ground_truth(rows, gt)["mean_error"]
                if err is not None:
                    errs.append(err)
            except Exception as e:
                print(f"Warning: Error in architecture plot for arch={arch}, seed={s}: {e}")
                continue
        m, ci = mean_ci(errs)
        labels.append(arch)
        means.append(m)
        cis.append(ci)
    bar_ci_plot(labels, means, cis, "Mean fused-position error",
                "Centralized vs Distributed Fusion", os.path.join(outdir, "centralized_vs_distributed.png"))


def plot_static_vs_dynamic_trust(config, outdir, seeds):
    scenarios = ["faulty_sensor_trust_weighted_fusion_fixed", "faulty_sensor_trust_weighted_fusion_dynamic"]
    available = [s for s in scenarios if s in config["scenarios"]]
    if not available:
        print(f"Warning: No trust comparison scenarios found")
        available = ["baseline"]
    
    means, cis = [], []
    for scenario in available:
        vals = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                result = run_once(run_config, scenario, s)
                val = result.get("collision_risk_count")
                if val is not None:
                    vals.append(val)
            except Exception as e:
                print(f"Warning: Error in trust plot for scenario={scenario}, seed={s}: {e}")
                continue
        m, ci = mean_ci(vals)
        means.append(m)
        cis.append(ci)
    
    if len(available) == 2:
        labels = ["static_trust", "dynamic_trust"]
    else:
        labels = available
    
    bar_ci_plot(labels, means, cis, "Collision risk count", "Static Trust vs Dynamic Trust",
                os.path.join(outdir, "static_vs_dynamic_trust.png"))


def plot_tracking_performance_by_scenario(config, outdir, seeds):
    scenarios = ["baseline", "sensor_noise", "sensor_dropout", "high_clutter", "high_latency"]
    available = [s for s in scenarios if s in config["scenarios"]]
    if not available:
        print("Warning: No tracking scenarios found - using baseline only")
        available = ["baseline"]
    
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    metrics = [("track_continuity", "Track continuity"),
               ("track_fragmentation", "Track fragmentation"),
               ("rmse_position_error", "Position RMSE")]
    for ax, (key, label) in zip(axes, metrics):
        means, cis = [], []
        for scenario in available:
            vals = []
            for s in range(seeds):
                try:
                    run_config = copy.deepcopy(config)
                    result = run_once(run_config, scenario, s)
                    val = result.get(key)
                    if val is not None:
                        vals.append(val)
                except Exception as e:
                    print(f"Warning: Error in tracking plot for scenario={scenario}, seed={s}: {e}")
                    continue
            m, ci = mean_ci(vals)
            means.append(m)
            cis.append(ci)
        ax.bar(available, [m or 0 for m in means], yerr=[c or 0 for c in cis], capsize=4,
               color=ADV_PALETTE[:len(available)])
        ax.set_title(label, fontsize=10, fontweight="bold")
        ax.set_xticks(range(len(available)))
        ax.set_xticklabels(available, rotation=40, ha="right", fontsize=8)
        _adv_style(ax)
    fig.suptitle("Tracking Performance Across Sensing Conditions", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(os.path.join(outdir, "tracking_method_comparison.png"), dpi=150)
    plt.close(fig)


def plot_confidence_intervals(config, outdir, seeds):
    scenarios = ["baseline", "sensor_noise", "sensor_dropout", "latency", "high_clutter", "faulty_sensor_naive_fusion"]
    available = [s for s in scenarios if s in config["scenarios"]]
    if not available:
        print("Warning: No scenarios found for confidence intervals - using baseline only")
        available = ["baseline"]
    
    means, cis = [], []
    for scenario in available:
        vals = []
        for s in range(seeds):
            try:
                run_config = copy.deepcopy(config)
                result = run_once(run_config, scenario, s)
                val = result.get("collision_risk_count")
                if val is not None:
                    vals.append(val)
            except Exception as e:
                print(f"Warning: Error in CI plot for scenario={scenario}, seed={s}: {e}")
                continue
        m, ci = mean_ci(vals)
        means.append(m)
        cis.append(ci)
    bar_ci_plot(available, means, cis, "Collision risk count (mean, 95% CI)",
                "Confidence Intervals for Major Results", os.path.join(outdir, "confidence_intervals_major_results.png"),
                colors=ADV_PALETTE[:len(available)])


ADVANCED_PLOTS = [
    plot_pd_vs_continuity,
    plot_pfa_vs_false_track,
    plot_clutter_vs_fusion_error,
    plot_range_vs_rmse,
    plot_snr_vs_error,
    plot_latency_vs_response,
    plot_packet_loss_vs_mission_success,
    plot_trust_error_vs_collision_risk,
    plot_fusion_mode_vs_rmse,
    plot_fusion_mode_vs_collision_risk,
    plot_fusion_mode_vs_mission_success,
    plot_architecture_comparison,
    plot_static_vs_dynamic_trust,
    plot_tracking_performance_by_scenario,
    plot_confidence_intervals,
]


def generate_advanced_plots(config_path, outdir, seeds):
    os.makedirs(outdir, exist_ok=True)
    with open(config_path) as f:
        config = json.load(f)
    
    print(f"\nGenerating {len(ADVANCED_PLOTS)} advanced plots...")
    for i, plot_fn in enumerate(ADVANCED_PLOTS, 1):
        try:
            print(f"  [{i}/{len(ADVANCED_PLOTS)}] {plot_fn.__name__}...")
            plot_fn(config, outdir, seeds)
        except Exception as e:
            print(f"    Warning: {plot_fn.__name__} failed: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"Saved advanced graphs to {outdir}/")


def main():
    parser = argparse.ArgumentParser(description="Generate graphs from results_summary.csv")
    parser.add_argument("--summary", default="results/results_summary.csv")
    parser.add_argument("--outdir", default="plots")
    parser.add_argument("--config", default="simulation_config.json", help="Advanced-set input config")
    parser.add_argument("--advanced-outdir", default="plots/advanced")
    parser.add_argument("--seeds", type=int, default=4, help="Seeds per advanced-set data point")
    parser.add_argument("--skip-advanced", action="store_true", help="Only generate the basic six graphs")
    args = parser.parse_args()

    # Basic plots
    os.makedirs(args.outdir, exist_ok=True)
    
    if not os.path.exists(args.summary):
        print(f"Warning: Summary file not found: {args.summary}")
        print("Skipping basic plots. Run metrics_analysis.py first.")
    else:
        df = load(args.summary)
        baseline_df = df[df["scenario"] == "baseline"]
        available_scenarios = df["scenario"].unique()

        if "false_positive" in available_scenarios:
            fp_rate = df[df["scenario"] == "false_positive"]["false_positive_rate"].iloc[0]
            per_run_bar(
                df, "false_positive", "unnecessary_avoidance_count",
                f"Unnecessary Avoidance at Fixed False Positive Rate = {fp_rate}\n(20 trials)",
                "Unnecessary avoidance count",
                os.path.join(args.outdir, "false_positive_vs_unnecessary_avoidance.png"),
                baseline_df=baseline_df,
            )
        else:
            print("Warning: 'false_positive' scenario not found")

        if "false_negative" in available_scenarios:
            fn_rate = df[df["scenario"] == "false_negative"]["false_negative_rate"].iloc[0]
            per_run_bar(
                df, "false_negative", "collision_risk_count",
                f"Collision Risk at Fixed False Negative Rate = {fn_rate}\n(20 trials)",
                "Collision risk count",
                os.path.join(args.outdir, "false_negative_vs_collision_risk.png"),
                baseline_df=baseline_df,
            )
        else:
            print("Warning: 'false_negative' scenario not found")

        if "latency" in available_scenarios:
            lat_steps = df[df["scenario"] == "latency"]["latency_steps"].iloc[0]
            per_run_bar(
                df, "latency", "avg_response_time_s",
                f"Response Time at Fixed Latency = {lat_steps} steps\n(20 trials)",
                "Avg response time (s)",
                os.path.join(args.outdir, "latency_vs_response_time.png"),
                baseline_df=baseline_df,
            )
        else:
            print("Warning: 'latency' scenario not found")

        if "sensor_dropout" in available_scenarios:
            dropout_mission_success(df, os.path.join(args.outdir, "dropout_vs_mission_success.png"))
        else:
            print("Warning: 'sensor_dropout' scenario not found")

        error_scenarios = ["false_positive", "false_negative", "sensor_noise", "latency", "sensor_dropout", "confidence_error"]
        if "baseline" in available_scenarios and any(s in available_scenarios for s in error_scenarios):
            baseline_vs_all(df, os.path.join(args.outdir, "baseline_vs_error_scenarios.png"))
        else:
            print("Warning: Not enough scenarios for baseline comparison")

        fusion_scenarios = ["no_fusion_matched", "naive_fusion", "trust_weighted_fusion"]
        if any(s in available_scenarios for s in fusion_scenarios):
            fusion_mode_comparison(df, os.path.join(args.outdir, "fusion_mode_vs_safety_metrics.png"))
        else:
            print("Warning: No fusion scenarios found")

        print(f"Saved basic graphs to {args.outdir}/")

    if not args.skip_advanced:
        generate_advanced_plots(args.config, args.advanced_outdir, args.seeds)


if __name__ == "__main__":
    sys.exit(main())