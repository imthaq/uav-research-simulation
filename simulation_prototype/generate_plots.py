"""
Generate basic graphs from results/results_summary.csv.

Reads the per-run metrics produced by metrics_analysis.py and saves six
PNG charts into plots/:

  1. false_positive_vs_unnecessary_avoidance.png
  2. false_negative_vs_collision_risk.png
  3. latency_vs_response_time.png
  4. dropout_vs_mission_success.png
  5. baseline_vs_error_scenarios.png
  6. fusion_mode_vs_safety_metrics.png

Run from the simulation_prototype/ folder:
    python generate_plots.py
Optional flags:
    --summary results/results_summary.csv   (input file)
    --outdir  plots                         (output folder)
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


SCENARIO_ORDER = [
    "baseline", "false_positive", "false_negative",
    "sensor_noise", "latency", "sensor_dropout",
    "confidence_error", "naive_fusion", "trust_weighted_fusion",
]
COLORS = {
    "baseline": "#4B5694",
    "false_positive": "#C2554A",
    "false_negative": "#D98C3D",
    "sensor_noise": "#7288AE",
    "latency": "#6E9075",
    "sensor_dropout": "#8B5FA8",
    "confidence_error": "#B08968",
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

    # Mark bars where the underlying value was actually missing (NaN) rather
    # than a true zero, so the chart doesn't silently misrepresent "no data"
    # as "zero response time".
    for bar, was_missing in zip(bars, missing):
        if was_missing:
            bar.set_hatch("//")
            bar.set_edgecolor("white")

    if baseline_df is not None:
        base_mean = baseline_df[value_col].mean()
        ax.axhline(base_mean, color="#111844", linestyle="--", linewidth=1.5,
                    label=f"baseline avg ({base_mean:.2f})")

    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlabel("Run number")
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
    ax.set_title("Sensor Dropout Scenario - Mission Success per Run", fontsize=12, fontweight="bold")
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
    """Compares no_fusion (confidence_error, same error profile family) vs
    naive_fusion vs trust_weighted_fusion on the two safety metrics fusion is
    meant to help with: missed responses (recovered via corroboration) and
    collision risk."""
    fusion_scenarios = ["confidence_error", "naive_fusion", "trust_weighted_fusion"]
    labels = ["no_fusion\n(confidence_error)", "naive_fusion", "trust_weighted_fusion"]
    present = [s for s in fusion_scenarios if s in df["scenario"].unique()]
    if not present:
        return
    agg = df[df["scenario"].isin(present)].groupby("scenario")[
        ["collision_risk_count", "missed_response_count", "fusion_recovery_count"]
    ].mean().reindex(present)

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    metrics = [
        ("collision_risk_count", "Avg collision risk count"),
        ("missed_response_count", "Avg missed response count"),
        ("fusion_recovery_count", "Avg fusion-recovered detections"),
    ]
    for ax, (col, label) in zip(axes, metrics):
        bars = ax.bar(range(len(present)), agg[col], color=[COLORS.get(s, "#7288AE") for s in present])
        ax.set_xticks(range(len(present)))
        ax.set_xticklabels([labels[fusion_scenarios.index(s)] for s in present], fontsize=8)
        ax.set_title(label, fontsize=10, fontweight="bold")
        ax.set_ylabel("count")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle("Fusion Mode Comparison - Average Across Runs", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate graphs from results_summary.csv")
    parser.add_argument("--summary", default="results/results_summary.csv")
    parser.add_argument("--outdir", default="plots")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    df = load(args.summary)
    baseline_df = df[df["scenario"] == "baseline"]

    per_run_bar(
        df, "false_positive", "unnecessary_avoidance_count",
        "False Positive Scenario vs Unnecessary Avoidance",
        "Unnecessary avoidance count",
        os.path.join(args.outdir, "false_positive_vs_unnecessary_avoidance.png"),
        baseline_df=baseline_df,
    )

    per_run_bar(
        df, "false_negative", "collision_risk_count",
        "False Negative Scenario vs Collision Risk",
        "Collision risk count",
        os.path.join(args.outdir, "false_negative_vs_collision_risk.png"),
        baseline_df=baseline_df,
    )

    per_run_bar(
        df, "latency", "avg_response_time_s",
        "Latency Scenario vs Response Time",
        "Avg response time (s)",
        os.path.join(args.outdir, "latency_vs_response_time.png"),
        baseline_df=baseline_df,
    )

    dropout_mission_success(df, os.path.join(args.outdir, "dropout_vs_mission_success.png"))

    baseline_vs_all(df, os.path.join(args.outdir, "baseline_vs_error_scenarios.png"))

    fusion_mode_comparison(df, os.path.join(args.outdir, "fusion_mode_vs_safety_metrics.png"))

    print(f"Saved 6 graphs to {args.outdir}/")


if __name__ == "__main__":
    main()
