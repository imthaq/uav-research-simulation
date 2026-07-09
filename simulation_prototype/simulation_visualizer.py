"""
Simulation Visualizer: Replay and visualize UAV swarm scenarios from CSV logs.
Supports live viewing, replay from logs, video export (mp4/gif), and
side-by-side fusion-mode comparison videos.
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
        # Mission outcome, precomputed once so the renderer can show a
        # definitive SUCCESS/FAILURE instead of only ever "In Progress".
        self.mission_success: bool = False
        self.mission_success_step: Optional[int] = None
        # True only while a LiveSimulationView is actively streaming steps in.
        # A replay loaded from a finished CSV (from_csv) never sets this, so
        # its behavior is unchanged. Live mode needs this because it doesn't
        # know the final step count in advance - see render_step().
        self.is_live: bool = False

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
            # Assuming fixed obstacle radius across logs
            data.obstacle_radius = 5.0
        except (ValueError, KeyError):
            pass

        # Determine mission outcome: find the first step (if any) at which
        # mission_completed_flag is True for any UAV. If the log ends
        # without that ever happening, the mission is a FAILURE, not
        # perpetually "In Progress".
        for row in data.rows:
            flag = row.get("mission_completed_flag")
            if flag in ("True", True):
                step = int(row["step"])
                if data.mission_success_step is None or step < data.mission_success_step:
                    data.mission_success_step = step
        data.mission_success = data.mission_success_step is not None

        return data

    def get_step_data(self, step: int) -> List[Dict]:
        """Get all UAV data for a given step."""
        return [r for r in self.rows if int(r["step"]) == step]


class SimulationVisualizer:
    """Visualizes UAV swarm simulation scenarios.

    Normally creates its own figure/axis. For multi-panel comparison
    renders (see `generate_fusion_comparison_video`), an existing
    `fig`/`ax` pair can be passed in so several scenarios share one figure.
    """

    def __init__(self, data: SimulationData, figsize: Tuple[int, int] = (12, 10),
                 fig=None, ax=None):
        self.data = data
        self.figsize = figsize
        self.current_step = 0

        # Setup figure (or reuse one provided by a multi-panel caller)
        if fig is not None and ax is not None:
            self.fig, self.ax = fig, ax
        else:
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
        action_taken = info_row.get("action_taken", "move")

        # Mission status: SUCCESS once mission_completed_flag has gone True
        # at or before this step; FAILURE once we reach the final logged
        # step without that ever happening; otherwise still In Progress.
        # While live (is_live=True), self.data.steps is just "how many steps
        # have arrived so far", not the true final count - the sim might
        # still be running - so we never claim FAILURE mid-stream. The final
        # frame gets re-rendered with is_live=False by LiveSimulationView.close()
        # once the run loop actually ends, which is when FAILURE can be shown.
        is_last_step = step >= self.data.steps - 1 and not self.data.is_live
        if self.data.mission_success and step >= self.data.mission_success_step:
            mission_status = "SUCCESS"
        elif is_last_step and not self.data.mission_success:
            mission_status = "FAILURE"
        else:
            mission_status = "In Progress"

        info_text = f"""Step: {step} | Time: {time_s:.1f}s
Scenario: {scenario}
Action: {action_taken}
Error: {error_type}
Mission: {mission_status}"""

        self.info_text.set_text(info_text)
        if mission_status == "SUCCESS":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightgreen", alpha=0.7))
        elif mission_status == "FAILURE":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightcoral", alpha=0.7))
        else:
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="wheat", alpha=0.5))

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


class LiveSimulationView:
    """Live view for use *while a simulation is actually running*, as
    opposed to replaying a CSV log after the fact.

    A running simulation loop (e.g. inside simple_swarm_sim.py) can create
    one of these and call `update_step()` once per timestep with that
    step's UAV rows (same field names as a CSV log row: uav_id,
    uav_pos_x/y, goal_pos_x/y, actual_obstacle_x/y, perceived_obstacle_x/y,
    action_taken, collision_risk_flag, mission_completed_flag, etc). The
    plot window updates live, frame by frame, as the simulation executes -
    no log file has to exist yet.

    Example integration inside a simulation's step loop:

        from simulation_visualizer import LiveSimulationView

        view = LiveSimulationView(scenario_name="baseline",
                                   world_width=100, world_height=100,
                                   obstacle_pos=(50, 50))
        for step in range(num_steps):
            rows = [uav.to_log_row(step) for uav in uavs]  # one dict/UAV
            view.update_step(step, rows)
        view.close()

    If no interactive GUI backend is available (e.g. running headless on a
    server), `update_step` fails silently after the first attempt so a
    simulation run is never blocked by the lack of a display.

    Mission status while running always reads SUCCESS or "In Progress" -
    never FAILURE - since a live view can't know it has reached the final
    step until the caller's loop actually ends. Call `close()` once the
    loop is done; it reveals the true final status (including FAILURE, if
    the mission never succeeded) for `hold_seconds` before closing the
    window.
    """

    def __init__(self, scenario_name: str = "live", world_width: float = 100.0,
                 world_height: float = 100.0, obstacle_pos: Tuple[float, float] = (50.0, 50.0),
                 obstacle_radius: float = 5.0, figsize: Tuple[int, int] = (12, 10),
                 pause: float = 0.001):
        data = SimulationData()
        data.scenario_name = scenario_name
        data.world_width = world_width
        data.world_height = world_height
        data.obstacle_pos = obstacle_pos
        data.obstacle_radius = obstacle_radius
        data.steps = 0
        data.num_uavs = 0
        data.is_live = True

        self._pause = pause
        self._known_uavs = set()
        self._gui_ok = True

        try:
            plt.ion()
            self.viz = SimulationVisualizer(data, figsize=figsize)
        except Exception as e:
            print(f"LiveSimulationView: no display available, live view disabled ({e})")
            self._gui_ok = False
            self.viz = None

    def update_step(self, step: int, rows: List[Dict]):
        """Push one timestep's worth of UAV rows into the live plot."""
        if not self._gui_ok or self.viz is None:
            return
        try:
            for row in rows:
                uav_id = int(row["uav_id"])
                self._known_uavs.add(uav_id)
                self.viz.data.uav_trajectories[uav_id].append(
                    (float(row["uav_pos_x"]), float(row["uav_pos_y"]))
                )
            self.viz.data.rows.extend(rows)
            self.viz.data.num_uavs = len(self._known_uavs)
            self.viz.data.steps = step + 1

            # Live view has no future data, so it cannot yet know if the
            # mission will ultimately succeed - only that it hasn't yet.
            for row in rows:
                flag = row.get("mission_completed_flag")
                if flag in ("True", True) and self.viz.data.mission_success_step is None:
                    self.viz.data.mission_success = True
                    self.viz.data.mission_success_step = step

            self.viz.render_step(step)
            plt.pause(self._pause)
        except Exception as e:
            print(f"LiveSimulationView: disabling live view after error ({e})")
            self._gui_ok = False

    def close(self, hold_seconds: float = 2.0):
        """Reveal the true final mission status (SUCCESS/FAILURE), hold the
        window open briefly so it can actually be seen, then close it.

        While steps were still streaming in, render_step() never claimed
        FAILURE (it doesn't know if more steps are coming). Now that the
        caller's step loop has actually ended, is_live is flipped off and
        the last received step is re-rendered once more so FAILURE can be
        correctly shown if the mission never succeeded."""
        if self.viz is None:
            return
        try:
            if self._gui_ok:
                self.viz.data.is_live = False
                last_step = max(self.viz.data.steps - 1, 0)
                self.viz.render_step(last_step)
                plt.pause(hold_seconds)
            plt.ioff()
            plt.close(self.viz.fig)
        except Exception:
            pass


def _load_available_scenarios(logs_dir: str, scenario_names: Tuple[str, ...],
                               run: str = "run1") -> List[Tuple[str, "SimulationData"]]:
    """Load whichever of the requested scenario logs actually exist."""
    loaded = []
    for name in scenario_names:
        log_path = os.path.join(logs_dir, f"{name}_{run}.csv")
        if not os.path.exists(log_path):
            print(f"  (skipping {name}: no log at {log_path})")
            continue
        loaded.append((name, SimulationData.from_csv(log_path)))
    return loaded


def generate_fusion_comparison_video(
    logs_dir: str = "logs",
    media_dir: str = "media",
    fps: int = 5,
    scenario_names: Tuple[str, ...] = ("naive_fusion", "trust_weighted_fusion", "no_fusion_matched"),
    output_name: str = "fusion_comparison_video.mp4",
) -> Optional[str]:
    """Render a side-by-side comparison video of the fusion-mode scenarios
    (naive fusion vs trust-weighted fusion vs no-fusion baseline, or
    whichever subset of these has logs available), sharing one figure so
    they can be watched and judged against each other directly."""
    print(f"Fusion comparison: looking for {scenario_names} in {logs_dir}/")
    loaded = _load_available_scenarios(logs_dir, scenario_names)

    if len(loaded) < 2:
        print("  Not enough fusion-mode logs found (need at least 2) - skipping comparison video.")
        return None

    if shutil.which("ffmpeg") is None:
        print("  ffmpeg not found on PATH - skipping fusion comparison video.")
        return None

    n = len(loaded)
    fig, axes = plt.subplots(1, n, figsize=(6.5 * n, 6.5))
    if n == 1:
        axes = [axes]

    visualizers = []
    for ax, (name, data) in zip(axes, loaded):
        viz = SimulationVisualizer(data, fig=fig, ax=ax)
        visualizers.append(viz)

    max_steps = max(v.data.steps for v in visualizers)

    def animate(frame):
        for v in visualizers:
            step = min(frame, v.data.steps - 1)
            v.render_step(step)
        return []

    anim = animation.FuncAnimation(
        fig, animate, frames=max_steps, interval=1000 // fps, repeat=True
    )

    # One shared legend for the whole comparison figure rather than one per panel.
    legend_elements = [
        patches.Patch(facecolor="red", alpha=0.6, label="Actual Obstacle"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=8, label="UAV"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="none", markeredgecolor="gray", markersize=8, label="Goal"),
        Line2D([0], [0], color="gray", linestyle="--", label="Perceived Obstacle"),
        patches.Patch(facecolor="orange", alpha=0.2, label="Collision-Risk Zone"),
    ]
    fig.legend(handles=legend_elements, loc="lower center", ncol=len(legend_elements), fontsize=9)
    fig.suptitle("Fusion Mode Comparison", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0.05, 1, 0.95])

    os.makedirs(media_dir, exist_ok=True)
    out_path = os.path.join(media_dir, output_name)
    writer = animation.FFMpegWriter(fps=fps)
    anim.save(out_path, writer=writer, dpi=100)
    plt.close(fig)
    print(f"Saved fusion comparison video to {out_path}")
    return out_path


def batch_generate(config_path: str = "simulation_config.json", logs_dir: str = "logs",
                    media_dir: str = "media", fps: int = 5, show_popups: bool = True) -> int:
    """Default (no-args) behavior: for every scenario declared in the config,
    load its run1 log, pop up an auto-playing preview, and save an MP4 to
    media/. Scenario list is read from the config (not hardcoded) so this
    stays in sync with whatever scenarios simple_swarm_sim.py/simulation_config.json
    define - including newly added ones like no_fusion_matched. Also
    generates a side-by-side fusion-mode comparison video if enough of
    those scenarios' logs are present."""
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

    # Fusion comparison video (best-effort; only if the relevant scenarios exist).
    fusion_candidates = tuple(
        n for n in ("naive_fusion", "trust_weighted_fusion", "no_fusion_matched") if n in scenario_names
    )
    if len(fusion_candidates) >= 2:
        print()
        generate_fusion_comparison_video(logs_dir, media_dir, fps, scenario_names=fusion_candidates)

    return 0 if success == len(scenario_names) else 1


def run_live_scenario(scenario_name: str, config_path: str = "simulation_config.json",
                       out_log: Optional[str] = None, hold_seconds: float = 2.0,
                       figsize: Tuple[int, int] = (12, 10)) -> Dict:
    """Actually run a scenario (not replay a finished CSV) and watch it live.

    This is where the two visualization paths meet: `simple_swarm_sim.py`
    itself never touches LiveSimulationView or matplotlib at all - it only
    knows how to run a scenario and produce log rows. This function is the
    one place that drives that simulation *and* visualizes it, live, step
    by step, keeping all visualization concerns in this module. It steps
    the `Simulation` object exactly the same way `Simulation.run()` would
    (same loop, same termination condition), just with a LiveSimulationView
    watching each step as it happens.

    Returns the same metrics dict `Simulation.run()` returns. If `out_log`
    is given, also writes the full per-step CSV log there once finished, so
    a live-watched run can still be replayed/exported later like any other.
    """
    from simple_swarm_sim import Simulation  # local import: only live mode needs this

    with open(config_path) as f:
        config = json.load(f)

    sim = Simulation(config, scenario_name)
    ox, oy, orad = sim.obstacle

    view = LiveSimulationView(
        scenario_name=scenario_name,
        world_width=config["world"]["width"],
        world_height=config["world"]["height"],
        obstacle_pos=(ox, oy),
        obstacle_radius=orad,
        figsize=figsize,
    )

    t = 0
    for t in range(sim.max_steps):
        rows_before = len(sim.log_rows)
        sim.step(t)
        # sim.step() just appended exactly sim.num_uavs rows for this step -
        # hand those straight to the live view, no CSV round-trip needed.
        view.update_step(t, sim.log_rows[rows_before:])
        if all(sim.reached_goal):
            break

    view.close(hold_seconds=hold_seconds)
    metrics = sim._metrics(t)

    if out_log:
        os.makedirs(os.path.dirname(out_log) or ".", exist_ok=True)
        fieldnames = list(sim.log_rows[0].keys())
        with open(out_log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(sim.log_rows)

    return metrics


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
        choices=["interactive", "mp4", "gif", "live-demo", "live"],
        default="interactive",
        help="Visualization mode. 'live-demo' replays --log through the "
        "LiveSimulationView update_step() API (the same call pattern a "
        "running simulation would use) instead of the static replay path - "
        "useful for testing the live view against a scenario that already "
        "finished. 'live' actually RUNS --scenario (via simple_swarm_sim.py's "
        "Simulation class) and watches it live as it computes, step by step - "
        "no existing --log needed.",
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
    parser.add_argument(
        "--fusion-comparison", action="store_true",
        help="Generate only the side-by-side fusion-mode comparison video "
        "from --logs-dir into --media-dir and exit.",
    )
    parser.add_argument(
        "--scenario", default=None,
        help="'live' mode only: which scenario from --config to actually run "
        "and watch live. Omit to run every scenario in --config live, one "
        "window at a time.",
    )
    parser.add_argument(
        "--out-log", default=None,
        help="'live' mode only: write the finished run's CSV log here once "
        "it completes (omit to not save a log). If --scenario is omitted "
        "(running every scenario live), this is treated as a directory and "
        "each scenario is written to <out-log>/<scenario>_live.csv.",
    )
    parser.add_argument(
        "--live-hold", type=float, default=2.0,
        help="'live' mode only: seconds to hold the final SUCCESS/FAILURE "
        "frame on screen before closing each scenario's window (default: 2.0)",
    )
    args = parser.parse_args()

    if args.fusion_comparison:
        generate_fusion_comparison_video(args.logs_dir, args.media_dir, args.fps)
        return 0

    if args.mode == "live":
        # True live mode: actually run the scenario(s) and watch as they
        # compute, instead of replaying something already finished. Doesn't
        # need --log as an input (there's nothing to load yet).
        with open(args.config) as f:
            config = json.load(f)
        scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

        for name in scenario_names:
            if args.out_log is None:
                out_log = None
            elif args.scenario is not None:
                out_log = args.out_log
            else:
                out_log = os.path.join(args.out_log, f"{name}_live.csv")

            print(f"--- live: {name} ---")
            metrics = run_live_scenario(
                name, config_path=args.config, out_log=out_log,
                hold_seconds=args.live_hold, figsize=tuple(args.figsize),
            )
            print(json.dumps(metrics, indent=2))
        return 0

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

    if args.mode == "live-demo":
        # Demonstrates the LiveSimulationView API (the same one a running
        # simulation loop would call) by feeding it the log's rows one
        # step at a time instead of loading the whole CSV into a static
        # replay - i.e. a live view, not a replay.
        view = LiveSimulationView(
            scenario_name=data.scenario_name,
            world_width=data.world_width,
            world_height=data.world_height,
            obstacle_pos=data.obstacle_pos,
            obstacle_radius=data.obstacle_radius,
            figsize=tuple(args.figsize),
        )
        for step in range(data.steps):
            view.update_step(step, data.get_step_data(step))
        view.close()
        return 0

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