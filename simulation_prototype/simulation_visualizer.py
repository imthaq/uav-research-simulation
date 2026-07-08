"""
Simulation Visualizer: Replay and visualize UAV swarm scenarios from CSV logs.
Supports live viewing, replay from logs, and video export (mp4/gif).
"""

import csv
import json
import os
import math
import shutil
import argparse
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import numpy as np


class SimulationData:
    """Container for parsed CSV log data."""

    def __init__(self):
        self.rows: List[Dict] = []
        self.scenario_name: str = ""
        self.world_width: float = 100.0
        self.world_height: float = 100.0
        self.uav_trajectories: Dict[int, List[Tuple[float, float]]] = defaultdict(list)
        self.num_uavs: int = 0
        self.steps: int = 0
        self.obstacle_pos: Tuple[float, float] = (50.0, 50.0)
        self.obstacle_radius: float = 5.0

    @staticmethod
    def from_csv(csv_path: str) -> "SimulationData":
        """Load simulation data from CSV log file."""
        data = SimulationData()
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            data.rows = list(reader)

        if not data.rows:
            raise ValueError(f"No data in {csv_path}")

        data.scenario_name = data.rows[0].get("scenario", "unknown")

        # Extract unique UAVs and build trajectories
        uav_ids = set()
        steps_set = set()
        for row in data.rows:
            uav_id = int(row["uav_id"])
            uav_ids.add(uav_id)
            steps_set.add(int(row["step"]))

            x = float(row["uav_pos_x"])
            y = float(row["uav_pos_y"])
            data.uav_trajectories[uav_id].append((x, y))

        data.num_uavs = len(uav_ids)
        data.steps = max(steps_set) + 1 if steps_set else 0

        # Extract obstacle from first row
        try:
            data.obstacle_pos = (
                float(data.rows[0]["actual_obstacle_x"]),
                float(data.rows[0]["actual_obstacle_y"]),
            )
            # ponytail: Assuming fixed obstacle radius across logs
            data.obstacle_radius = 5.0
        except (ValueError, KeyError):
            pass

        return data

    def get_step_data(self, step: int) -> List[Dict]:
        """Get all UAV data for a given step."""
        return [r for r in self.rows if int(r["step"]) == step]


class SimulationVisualizer:
    """Visualizes UAV swarm simulation scenarios."""

    def __init__(self, data: SimulationData, figsize: Tuple[int, int] = (12, 10)):
        self.data = data
        self.figsize = figsize
        self.current_step = 0

        # Setup figure
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.setup_axis()

        # Persistent plot elements
        self.uav_dots = {}  # UAV position circles by ID
        self.uav_labels = {}  # UAV ID text labels
        self.uav_trails = {}  # Trajectory lines
        self.obstacle_circle = None
        self.perceived_obstacles = []
        self.collision_zones = []
        self.goal_markers = {}
        self.info_text = None

    def setup_axis(self):
        """Configure the plot axis and static elements."""
        self.ax.set_xlim(-5, self.data.world_width + 5)
        self.ax.set_ylim(-5, self.data.world_height + 5)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.set_title(f"UAV Swarm Simulation: {self.data.scenario_name}")
        self.ax.grid(True, alpha=0.3)

        # Draw world boundary
        boundary = patches.Rectangle(
            (0, 0),
            self.data.world_width,
            self.data.world_height,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
        )
        self.ax.add_patch(boundary)

        # Draw obstacle (static)
        ox, oy = self.data.obstacle_pos
        self.obstacle_circle = patches.Circle(
            (ox, oy),
            self.data.obstacle_radius,
            color="red",
            alpha=0.6,
            label="Actual Obstacle",
        )
        self.ax.add_patch(self.obstacle_circle)

    def _get_colors(self) -> List[str]:
        """Get distinct colors for each UAV."""
        colors = plt.cm.tab10(np.linspace(0, 1, self.data.num_uavs))
        return [plt.cm.hsv(i / max(self.data.num_uavs, 1)) for i in range(self.data.num_uavs)]

    def render_step(self, step: int):
        """Render a single simulation step."""
        self.current_step = step
        step_data = self.data.get_step_data(step)

        if not step_data:
            return

        colors = self._get_colors()

        # Clear previous ephemeral elements
        for artist in self.perceived_obstacles + self.collision_zones:
            if artist in self.ax.patches:
                artist.remove()
        self.perceived_obstacles.clear()
        self.collision_zones.clear()

        # Process each UAV's data for this step
        for idx, row in enumerate(step_data):
            uav_id = int(row["uav_id"])
            color = colors[uav_id % len(colors)]

            x = float(row["uav_pos_x"])
            y = float(row["uav_pos_y"])
            gx = float(row["goal_pos_x"])
            gy = float(row["goal_pos_y"])

            # Draw/update UAV position dot
            if uav_id not in self.uav_dots:
                (dot,) = self.ax.plot(x, y, "o", markersize=10, color=color)
                self.uav_dots[uav_id] = dot
                label = self.ax.text(
                    x, y + 1.5, f"U{uav_id}", fontsize=8, ha="center", color=color
                )
                self.uav_labels[uav_id] = label
            else:
                self.uav_dots[uav_id].set_data([x], [y])
                self.uav_labels[uav_id].set_position((x, y + 1.5))

            # Draw trajectory (if not at start)
            if uav_id not in self.uav_trails and step > 0:
                trail_data = self.data.uav_trajectories[uav_id][: step + 1]
                if trail_data:
                    xs, ys = zip(*trail_data)
                    (trail,) = self.ax.plot(xs, ys, "-", color=color, alpha=0.4, linewidth=1)
                    self.uav_trails[uav_id] = trail
            elif uav_id in self.uav_trails:
                trail_data = self.data.uav_trajectories[uav_id][: step + 1]
                if trail_data:
                    xs, ys = zip(*trail_data)
                    self.uav_trails[uav_id].set_data(xs, ys)

            # Draw goal position
            if uav_id not in self.goal_markers:
                (goal,) = self.ax.plot(
                    gx, gy, "s", markersize=8, color=color, alpha=0.5, markerfacecolor="none"
                )
                self.goal_markers[uav_id] = goal
            else:
                self.goal_markers[uav_id].set_data([gx], [gy])

            # Draw perceived obstacle if present
            try:
                px = row.get("perceived_obstacle_x")
                py = row.get("perceived_obstacle_y")
                if px and py and px != "" and py != "":
                    px, py = float(px), float(py)
                    perc_obs = patches.Circle(
                        (px, py),
                        1.0,
                        color=color,
                        alpha=0.3,
                        linestyle="--",
                        fill=False,
                        linewidth=1,
                    )
                    self.ax.add_patch(perc_obs)
                    self.perceived_obstacles.append(perc_obs)
            except (ValueError, TypeError):
                pass

            # Draw collision-risk zone if applicable
            # ponytail: collision_risk_flag now correctly reflects near_miss_distance threshold
            # which triggers only when real threat is detected, improved from phantom-only detection
            if row.get("collision_risk_flag") == "True" or row.get("collision_risk_flag") is True:
                dist = float(row.get("nearest_entity_distance", 10))
                if dist < 5:
                    collision_zone = patches.Circle(
                        (x, y), dist, color="orange", alpha=0.2, linestyle=":", linewidth=1
                    )
                    self.ax.add_patch(collision_zone)
                    self.collision_zones.append(collision_zone)

        # Update info text
        if not self.info_text:
            self.info_text = self.ax.text(
                0.02,
                0.98,
                "",
                transform=self.ax.transAxes,
                verticalalignment="top",
                fontsize=9,
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
            )

        # Extract info from first UAV row
        info_row = step_data[0]
        time_s = float(info_row.get("time_s", 0))
        scenario = info_row.get("scenario", "unknown")
        error_type = info_row.get("perception_error_type", "none")
        mission_ok = info_row.get("mission_completed_flag", "False")
        action_taken = info_row.get("action_taken", "move")

        # ponytail: action_taken now distinguishes "avoidance" vs "false_avoidance"
        # based on triggered_real vs triggered_phantom, matching improved simulation logic
        info_text = f"""Step: {step} | Time: {time_s:.1f}s
Scenario: {scenario}
Action: {action_taken}
Error: {error_type}
Mission: {'? SUCCESS' if mission_ok in ('True', True) else 'In Progress'}"""

        self.info_text.set_text(info_text)

        self.fig.canvas.draw_idle()

    def add_legend(self):
        """Add a legend to the plot."""
        legend_elements = [
            patches.Patch(facecolor="red", alpha=0.6, label="Actual Obstacle"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=8, label="UAV"),
            Line2D([0], [0], marker="s", color="w", markerfacecolor="none", markeredgecolor="gray", markersize=8, label="Goal"),
            Line2D([0], [0], color="gray", linestyle="--", label="Perceived Obstacle"),
            patches.Patch(facecolor="orange", alpha=0.2, label="Collision-Risk Zone"),
        ]
        self.ax.legend(handles=legend_elements, loc="upper right", fontsize=9)

    def save_animation(self, output_path: str, fps: int = 5, dpi: int = 80):
        """Generate and save animation as MP4 or GIF."""
        print(f"Creating animation... this may take a while")

        def animate(frame):
            self.render_step(frame)
            return []

        anim = animation.FuncAnimation(
            self.fig,
            animate,
            frames=self.data.steps,
            interval=1000 // fps,
            repeat=True,
        )

        ext = Path(output_path).suffix.lower()
        writer = None
        if ext == ".mp4":
            writer = animation.FFMpegWriter(fps=fps)
        elif ext == ".gif":
            writer = animation.PillowWriter(fps=fps)
        else:
            raise ValueError(f"Unsupported format: {ext}. Use .mp4 or .gif")

        anim.save(output_path, writer=writer, dpi=dpi)
        print(f"Saved animation to {output_path}")

    def play_auto(self, fps: int = 5, hold_seconds: float = 0.6):
        """Auto-advance through every step in a popup window (no keypresses
        needed), then close itself. Used by batch mode so all scenarios can
        be previewed unattended. Silently skipped if no GUI backend is
        available (e.g. running headless)."""
        self.add_legend()
        plt.show(block=False)
        interval = 1.0 / max(fps, 1)
        for step in range(self.data.steps):
            self.render_step(step)
            plt.pause(interval)
        plt.pause(hold_seconds)
        plt.close(self.fig)

    def show_interactive(self):
        """Show interactive plot with keyboard controls."""
        print("Interactive mode - Use arrow keys to navigate, 'q' to quit")

        def on_key(event):
            if event.key == "left":
                self.current_step = max(0, self.current_step - 1)
                self.render_step(self.current_step)
                print(f"Step: {self.current_step}")
            elif event.key == "right":
                self.current_step = min(self.data.steps - 1, self.current_step + 1)
                self.render_step(self.current_step)
                print(f"Step: {self.current_step}")
            elif event.key == "q":
                plt.close()

        self.fig.canvas.mpl_connect("key_press_event", on_key)
        self.render_step(0)
        self.add_legend()
        plt.tight_layout()
        plt.show()


def batch_generate(config_path: str = "simulation_config.json", logs_dir: str = "logs",
                    media_dir: str = "media", fps: int = 5, show_popups: bool = True) -> int:
    """Default (no-args) behavior: for every scenario declared in the config,
    load its run1 log, pop up an auto-playing preview, and save an MP4 to
    media/. Scenario list is read from the config (not hardcoded) so this
    stays in sync with whatever scenarios simple_swarm_sim.py/simulation_config.json
    define - including newly added ones like no_fusion_matched."""
    with open(config_path) as f:
        config = json.load(f)
    scenario_names = list(config["scenarios"].keys())
    os.makedirs(media_dir, exist_ok=True)

    print(f"Batch mode: {len(scenario_names)} scenario(s) from {config_path}\n")

    ffmpeg_ok = shutil.which("ffmpeg") is not None
    if not ffmpeg_ok:
        print("NOTE: ffmpeg not found on PATH - MP4s will be skipped this run (popups still work).")
        print("      Install it (e.g. https://www.gyan.dev/ffmpeg/ or `winget install ffmpeg`),")
        print("      make sure it's on PATH, and re-run to get the videos.\n")

    results = {}
    for name in scenario_names:
        log_path = os.path.join(logs_dir, f"{name}_run1.csv")
        if not os.path.exists(log_path):
            print(f"SKIP {name}: no log at {log_path} (run run_experiments.py or simple_swarm_sim.py first)")
            results[name] = False
            continue

        print(f"{name}: loading {log_path}")
        data = SimulationData.from_csv(log_path)

        if show_popups:
            viz = SimulationVisualizer(data)
            try:
                viz.play_auto(fps=fps)
            except Exception as e:
                print(f"  (skipping popup view: {e})")

        if not ffmpeg_ok:
            results[name] = False
            continue

        # play_auto() closes its own figure/canvas when done (and may have
        # torn it down on a real GUI backend), so build a fresh visualizer
        # for saving rather than reusing a figure that's no longer alive.
        viz = SimulationVisualizer(data)
        out_path = os.path.join(media_dir, f"{name}_video.mp4")
        try:
            viz.save_animation(out_path, fps=fps, dpi=100)
            results[name] = True
        except Exception as e:
            print(f"  video save failed for {name}: {e}")
            results[name] = False

    success = sum(1 for v in results.values() if v)
    print(f"\nDone: {success}/{len(scenario_names)} videos saved to {media_dir}/")
    return 0 if success == len(scenario_names) else 1


def main():
    parser = argparse.ArgumentParser(
        description="Visualize UAV swarm simulation from CSV logs. "
        "Run with no arguments to auto-generate popup previews and MP4s for every scenario."
    )
    parser.add_argument(
        "--log",
        default=None,
        help="Path to a single CSV log file (e.g., logs/baseline_run1.csv). "
        "Omit this to run batch mode over every scenario in --config instead.",
    )
    parser.add_argument(
        "--mode",
        choices=["interactive", "mp4", "gif"],
        default="interactive",
        help="Visualization mode",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output path for video/gif (required for mp4/gif modes)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="Frames per second for animation (default: 5)",
    )
    parser.add_argument(
        "--figsize",
        type=int,
        nargs=2,
        default=[12, 10],
        help="Figure size (width height)",
    )
    parser.add_argument(
        "--config",
        default="simulation_config.json",
        help="Config file to read scenario names from (batch mode only)",
    )
    parser.add_argument("--logs-dir", default="logs", help="Batch mode only")
    parser.add_argument("--media-dir", default="media", help="Batch mode only")
    parser.add_argument(
        "--no-popup", action="store_true",
        help="Batch mode only: skip popup previews, just save the MP4s",
    )
    args = parser.parse_args()

    if args.log is None:
        return batch_generate(args.config, args.logs_dir, args.media_dir, args.fps,
                               show_popups=not args.no_popup)

    # Validate inputs
    if not os.path.exists(args.log):
        print(f"Error: Log file not found: {args.log}")
        return 1

    if args.mode in ("mp4", "gif") and not args.output:
        print(f"Error: --output required for {args.mode} mode")
        return 1

    # Load and visualize
    print(f"Loading {args.log}...")
    data = SimulationData.from_csv(args.log)
    print(
        f"Loaded {data.num_uavs} UAVs, {data.steps} steps, scenario: {data.scenario_name}"
    )

    viz = SimulationVisualizer(data, figsize=tuple(args.figsize))

    if args.mode == "interactive":
        viz.show_interactive()
    elif args.mode == "mp4":
        viz.save_animation(args.output, fps=args.fps, dpi=100)
    elif args.mode == "gif":
        viz.save_animation(args.output, fps=args.fps, dpi=80)

    return 0


if __name__ == "__main__":
    exit(main())