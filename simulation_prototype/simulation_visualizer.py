"""
Simulation Visualizer: Replay and visualize UAV swarm scenarios from CSV logs.
Supports live viewing, replay from logs, video export (mp4/gif), side-by-side
fusion-mode/architecture comparison videos, and a one-shot "advanced demo"
video suite covering Kalman tracking, sensor fusion, and inter-UAV
communication.

Task 22 overlay additions on top of the original radar-only overlay:
  - predicted vs. filtered track position (Kalman "coasting" vs. updated)
  - measured detection position (radar/vision/LiDAR)
  - covariance ellipses (tracks, fused estimates, vision/LiDAR measurements)
  - track status, track history trail, and a false-track heuristic label
  - stale measurement indicator (vision/LiDAR async-update staleness)
  - sensor source label + per-measurement confidence/trust
  - fused track marker (centralized and per-UAV distributed estimates)
  - inter-UAV communication links, with packet-dropout indication
  - centralized/distributed fusion architecture labeling

Task 26 overlay additions:
  - radar operating mode (normal/long_range/high_resolution/degraded/...)
  - calibrated confidence indicator (green=correct, red=false-alarm)
  - perception-quality state and critical action taken
  - adaptive safety margin value and mode
  - abstention status flag
  - handoff success/failure indicator
  - ghost radar returns (multipath, side-lobe, duplicate) with type label
  - Doppler ambiguity flag on radial-velocity label
  - cross-modal registration offset (from scenario config)
  - degraded sensor / reliability state label
  - recovery state (degraded/critical mode steps)
  - centralized/distributed fusion state (enhanced: per-step architecture)
"""

# Task 26 note: all new overlay fields are read from existing log columns
# (ghost_flag, doppler_ambiguity_flag, radar_operating_mode, confidence_correct,
# safety_margin_applied, abstention_flag, handoff_*_flag, degraded_mode_flag,
# critical_mode_flag, recovery_time_steps, radar_reliability_state).
# Registration offset is read from the scenario config's
# cross_modal_registration block, not from individual detection rows.

import copy
import csv
import json
import os
import re
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
    def from_rows(rows: List[Dict]) -> "SimulationData":
        """Builds a SimulationData directly from already-in-memory log rows
        (e.g. Simulation.log_rows, or a RadarLikeModel's own `.sim.log_rows`)
        without a CSV round trip. from_csv is just this + a CSV read, so
        in-memory pipelines (like the advanced-demo generator below) can
        skip writing/reading a file entirely and still get identical
        behavior to a replayed log."""
        data = SimulationData()
        if not rows:
            raise ValueError("No rows given to SimulationData.from_rows")

        data.rows = rows
        data.scenario_name = rows[0].get("scenario", "unknown")

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

        try:
            data.obstacle_pos = (
                float(data.rows[0]["actual_obstacle_x"]),
                float(data.rows[0]["actual_obstacle_y"]),
            )
            data.obstacle_radius = 5.0
        except (ValueError, KeyError, TypeError):
            pass

        for row in data.rows:
            flag = row.get("mission_completed_flag")
            if flag in ("True", True):
                step = int(row["step"])
                if data.mission_success_step is None or step < data.mission_success_step:
                    data.mission_success_step = step
        data.mission_success = data.mission_success_step is not None

        return data

    @staticmethod
    def from_csv(csv_path: str) -> "SimulationData":
        """Load simulation data from CSV log file."""
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if not rows:
            raise ValueError(f"No data in {csv_path}")
        return SimulationData.from_rows(rows)

    def get_step_data(self, step: int) -> List[Dict]:
        """Get all UAV data for a given step."""
        return [r for r in self.rows if int(r["step"]) == step]


class RadarData:
    """Parsed radar detection rows (radar_like_model.py) and radar track
    rows (radar_track_model.py) for one scenario, indexed by
    (step, radar_id) - radar_id is the same as uav_id - so the visualizer
    can look up "what did this UAV's radar see/track this step" in O(1)
    per frame. Both logs are optional and independent: either can be
    missing and the visualizer just skips the pieces that depend on it."""

    def __init__(self):
        self.detections: Dict[Tuple[int, int], List[Dict]] = defaultdict(list)
        self.tracks: Dict[Tuple[int, int], List[Dict]] = defaultdict(list)

    @staticmethod
    def from_rows(detection_rows: Optional[List[Dict]] = None,
                   track_rows: Optional[List[Dict]] = None) -> "RadarData":
        """Builds a RadarData directly from in-memory detection/track rows
        (as returned by RadarLikeModel.run() / radar_track_model.build_tracks),
        skipping the CSV round trip - used by the in-memory advanced-demo
        pipeline below."""
        rd = RadarData()
        for row in (detection_rows or []):
            key = (int(row["time_step"]), int(row["radar_id"]))
            rd.detections[key].append(row)
        for row in (track_rows or []):
            key = (int(row["time_step"]), int(row["radar_id"]))
            rd.tracks[key].append(row)
        return rd

    @staticmethod
    def from_csvs(scenario_name: str, radar_log_path: Optional[str] = None,
                   track_log_path: Optional[str] = None) -> "RadarData":
        """Loads and filters the (possibly multi-scenario) combined CSVs
        that radar_like_model.py / radar_track_model.py write, keeping only
        this scenario's rows. Missing/absent paths just leave that half
        empty rather than raising."""
        rd = RadarData()
        if radar_log_path and os.path.exists(radar_log_path):
            with open(radar_log_path) as f:
                for row in csv.DictReader(f):
                    if row.get("scenario") != scenario_name:
                        continue
                    key = (int(row["time_step"]), int(row["radar_id"]))
                    rd.detections[key].append(row)
        if track_log_path and os.path.exists(track_log_path):
            with open(track_log_path) as f:
                for row in csv.DictReader(f):
                    if row.get("scenario") != scenario_name:
                        continue
                    key = (int(row["time_step"]), int(row["radar_id"]))
                    rd.tracks[key].append(row)
        return rd


class AuxSensorData:
    """Parsed detection rows from an auxiliary point-sensor model that
    shares the vision_like_model.py / lidar_like_model.py row shape:
    measured_x/measured_y, confidence_score, covariance, validity_flag,
    is_stale, sensor_reliability, plus a per-model id column
    (vision_id / lidar_id). Indexed by (step, uav_id), same shape as
    RadarData.detections, so the visualizer can treat radar/vision/LiDAR
    overlays uniformly wherever their fields line up."""

    def __init__(self, sensor_label: str):
        self.sensor_label = sensor_label  # "vision" or "lidar"
        self.detections: Dict[Tuple[int, int], List[Dict]] = defaultdict(list)

    @staticmethod
    def from_csv(path: Optional[str], scenario_name: str, sensor_label: str) -> "AuxSensorData":
        data = AuxSensorData(sensor_label)
        if not path or not os.path.exists(path):
            return data
        id_field = f"{sensor_label}_id"
        with open(path) as f:
            for row in csv.DictReader(f):
                if row.get("scenario") != scenario_name:
                    continue
                if id_field not in row:
                    continue
                key = (int(row["time_step"]), int(row[id_field]))
                data.detections[key].append(row)
        return data


def _parse_contributor_uav_ids(source_ids_str: Optional[str]) -> set:
    """fusion_model.py encodes each fused estimate's contributing radar
    tracks as a ';'-joined list of track ids shaped 'r{radar_id}_t{n}'
    (see RadarTrack.track_id). Pulls the contributing UAV/radar ids back
    out of that string - the only per-link record the fusion log carries -
    so the visualizer can infer which inter-UAV broadcasts a distributed
    fused estimate actually incorporated this step."""
    ids = set()
    for tok in (source_ids_str or "").split(";"):
        m = re.match(r"r(\d+)_t\d+", tok.strip())
        if m:
            ids.add(int(m.group(1)))
    return ids


class FusedTrackData:
    """Parsed rows from fusion_model.py's fused-track log, indexed by
    time_step (a step can contain several rows: one shared centralized
    estimate, or one per UAV's local distributed estimate)."""

    def __init__(self):
        self.rows_by_step: Dict[int, List[Dict]] = defaultdict(list)

    @staticmethod
    def from_rows(rows: List[Dict]) -> "FusedTrackData":
        fd = FusedTrackData()
        for row in rows:
            fd.rows_by_step[int(row["time_step"])].append(row)
        return fd

    @staticmethod
    def from_csv(path: Optional[str], scenario_name: str) -> "FusedTrackData":
        fd = FusedTrackData()
        if not path or not os.path.exists(path):
            return fd
        with open(path) as f:
            for row in csv.DictReader(f):
                if row.get("scenario") != scenario_name:
                    continue
                fd.rows_by_step[int(row["time_step"])].append(row)
        return fd


def _resolve_radar_params(config: dict, scenario_name: str) -> Tuple[float, float]:
    """Mirrors radar_like_model.RadarLikeModel's own scenario-first config
    resolution (scenario override -> top-level "radar" section -> default)
    for just the two params the visualizer needs to draw: sensing range and
    field of view. Keeping this logic in sync with RadarLikeModel matters -
    otherwise the drawn sensing circle wouldn't match what the radar model
    actually used to gate detections for that scenario."""
    radar_cfg = config.get("radar", {})
    scn = config.get("scenarios", {}).get(scenario_name, {})
    default_range = config.get("sensing", {}).get("sensor_range", 15.0)
    radar_range = scn.get("radar_max_range", radar_cfg.get("radar_max_range", default_range))
    radar_fov = scn.get("radar_field_of_view", radar_cfg.get("radar_field_of_view", 360.0))
    return float(radar_range), float(radar_fov)


def _covariance_ellipse(cx: float, cy: float, cov2x2, n_std: float = 2.0, **kwargs):
    """Builds a matplotlib Ellipse patch representing the n_std-sigma
    confidence region of a 2x2 position covariance matrix, via its
    eigen-decomposition (ellipse axes = sqrt(eigenvalues), orientation =
    the corresponding eigenvector). Returns None for a degenerate/missing
    covariance rather than raising, since a malformed covariance shouldn't
    crash a whole frame's render."""
    try:
        cov = np.array(cov2x2, dtype=float)
        if cov.shape != (2, 2):
            return None
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.clip(eigvals, 0.0, None)
        order = np.argsort(eigvals)[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]
        width, height = 2 * n_std * np.sqrt(eigvals)
        angle = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
        return patches.Ellipse((cx, cy), width=max(float(width), 1e-3),
                                height=max(float(height), 1e-3), angle=angle, **kwargs)
    except (ValueError, TypeError, np.linalg.LinAlgError, IndexError):
        return None


class SimulationVisualizer:
    """Visualizes UAV swarm simulation scenarios.

    Normally creates its own figure/axis. For multi-panel comparison
    renders (see `generate_fusion_comparison_video` /
    `generate_advanced_demo_videos`), an existing `fig`/`ax` pair can be
    passed in so several scenarios share one figure.
    """

    # Shared font sizes so every overlay reads at a consistent, legible
    # scale instead of a mix of ad-hoc values.
    FONT_TITLE = 13
    FONT_AXIS = 10
    FONT_ENTITY_ID = 9      # UAV / target ID labels
    FONT_DETAIL = 8         # detection / track / fusion annotation text
    FONT_INFO_PANEL = 9.5
    FONT_LEGEND = 8.5

    def __init__(self, data: SimulationData, figsize: Tuple[int, int] = (12, 10),
                 fig=None, ax=None, radar_data: Optional["RadarData"] = None,
                 radar_range: float = 15.0, radar_fov_deg: float = 360.0,
                 vision_data: Optional["AuxSensorData"] = None,
                 lidar_data: Optional["AuxSensorData"] = None,
                 fused_data: Optional["FusedTrackData"] = None,
                 show_covariance: bool = True, show_track_history: bool = True,
                 scenario_config: Optional[Dict] = None):
        self.data = data
        self.figsize = figsize
        self.current_step = 0

        # Optional radar overlay: detections, tracks, sensing range/FOV.
        # radar_data is None means "no radar log was supplied" - every
        # radar-specific draw call below is skipped in that case, so the
        # visualizer works exactly as before with no radar log present.
        self.radar_data = radar_data
        self.radar_range = radar_range
        self.radar_fov_deg = radar_fov_deg

        # Optional vision/LiDAR overlays - same "None = skip" contract.
        self.vision_data = vision_data
        self.lidar_data = lidar_data

        # Optional fused-track overlay (fusion_model.py output): fused
        # position, architecture, communication links/dropout.
        self.fused_data = fused_data

        self.show_covariance = show_covariance
        self.show_track_history = show_track_history
        # Task 26: optional per-scenario config dict (scenario sub-dict,
        # not the full config) for registration offset display.
        self._scenario_config = scenario_config

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
        # Everything radar/vision/LiDAR/fusion-related is redrawn from
        # scratch every step (like perceived_obstacles/collision_zones
        # above), so one flat list of artists - patches AND text, both
        # support .remove() - is enough.
        self.radar_artists = []

        # Track history: track_id -> sorted [(step, x, y), ...], built once
        # up front from every track row this replay/run has (radar_data is
        # already fully populated before the visualizer is constructed,
        # whether it came from a finished CSV or an in-memory model run).
        self._track_history: Dict[str, List[Tuple[int, float, float]]] = defaultdict(list)
        if self.radar_data is not None:
            for rows in self.radar_data.tracks.values():
                for row in rows:
                    tid = row.get("track_id")
                    try:
                        self._track_history[tid].append(
                            (int(row["time_step"]), float(row["est_x"]), float(row["est_y"])))
                    except (TypeError, ValueError, KeyError):
                        continue
            for tid in self._track_history:
                self._track_history[tid].sort(key=lambda r: r[0])

    def setup_axis(self):
        """Configure the plot axis and static elements. Called once; the
        axis bounds and aspect ratio are fixed here and never changed
        again, so the "camera" stays put for the whole render/animation
        instead of drifting frame to frame."""
        self.ax.set_xlim(-5, self.data.world_width + 5)
        self.ax.set_ylim(-5, self.data.world_height + 5)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (meters)", fontsize=self.FONT_AXIS)
        self.ax.set_ylabel("Y (meters)", fontsize=self.FONT_AXIS)
        self.ax.tick_params(labelsize=self.FONT_AXIS - 1)
        self.ax.set_title(
            f"UAV Swarm Simulation \u2014 {self.data.scenario_name}",
            fontsize=self.FONT_TITLE, fontweight="bold",
        )
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

        # Draw obstacle (static). Its legend entry is added explicitly in
        # add_legend(), so no label is set here.
        ox, oy = self.data.obstacle_pos
        self.obstacle_circle = patches.Circle(
            (ox, oy), self.data.obstacle_radius, color="red", alpha=0.6,
        )
        self.ax.add_patch(self.obstacle_circle)

    def _get_colors(self) -> List[str]:
        """Get one distinct, stable color per UAV, evenly spaced around
        the color wheel so IDs stay visually distinguishable regardless
        of swarm size."""
        n = max(self.data.num_uavs, 1)
        return [plt.cm.hsv(i / n) for i in range(self.data.num_uavs)]

    def _mission_status(self, step: int) -> str:
        """SUCCESS once mission_completed_flag has gone True at or before
        this step; FAILURE once the final logged step is reached without
        that ever happening; otherwise still "In Progress".

        While live (is_live=True), self.data.steps only reflects how many
        steps have arrived so far, not the true final count, so FAILURE is
        never claimed mid-stream. LiveSimulationView.close() re-renders the
        final frame with is_live=False once the run actually ends, which is
        when FAILURE can be shown."""
        is_last_step = step >= self.data.steps - 1 and not self.data.is_live
        if self.data.mission_success and step >= self.data.mission_success_step:
            return "SUCCESS"
        if is_last_step and not self.data.mission_success:
            return "FAILURE"
        return "In Progress"

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

        for artist in self.radar_artists:
            try:
                artist.remove()
            except (ValueError, NotImplementedError):
                pass
        self.radar_artists.clear()

        uav_positions: Dict[int, Tuple[float, float]] = {}

        # Process each UAV's data for this step
        for idx, row in enumerate(step_data):
            uav_id = int(row["uav_id"])
            color = colors[uav_id % len(colors)]

            x = float(row["uav_pos_x"])
            y = float(row["uav_pos_y"])
            gx = float(row["goal_pos_x"])
            gy = float(row["goal_pos_y"])
            uav_positions[uav_id] = (x, y)

            # Draw/update UAV position dot
            if uav_id not in self.uav_dots:
                (dot,) = self.ax.plot(x, y, "o", markersize=10, color=color)
                self.uav_dots[uav_id] = dot
                label = self.ax.text(
                    x, y + 1.5, f"U{uav_id}", fontsize=self.FONT_ENTITY_ID,
                    ha="center", fontweight="bold", color=color,
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

            # ---- Radar overlay (only if a RadarData was supplied) ----
            if self.radar_data is not None:
                self._draw_radar_overlay(step, uav_id, x, y, gx, gy, color, row)

            # ---- Vision / LiDAR overlays (only if supplied) ----
            if self.vision_data is not None:
                self._draw_aux_overlay(self.vision_data, "*", step, uav_id, color)
            if self.lidar_data is not None:
                self._draw_aux_overlay(self.lidar_data, "P", step, uav_id, color)

        # ---- Fusion overlay: fused tracks + communication links ----
        if self.fused_data is not None:
            self._draw_fusion_overlay(step, colors, uav_positions)

        # Update info text
        if not self.info_text:
            self.info_text = self.ax.text(
                0.02,
                0.98,
                "",
                transform=self.ax.transAxes,
                verticalalignment="top",
                fontsize=self.FONT_INFO_PANEL,
                family="monospace",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6),
            )

        info_row = step_data[0]
        time_s = float(info_row.get("time_s", 0))
        scenario = info_row.get("scenario", "unknown")
        fusion_mode = info_row.get("fusion_mode", "no_fusion")
        mission_status = self._mission_status(step)

        # Core block: always shown.
        lines = [
            f"Scenario: {scenario}",
            f"Step {step}  |  t = {time_s:.1f}s",
            f"Fusion mode: {fusion_mode}",
        ]

        # Conditional block: only real, non-empty conditions are shown, so
        # the panel doesn't fill up with "none" / "nominal" noise on a
        # normal step.
        extras = []
        error_type = info_row.get("perception_error_type", "none")
        if error_type not in (None, "", "none", "None"):
            extras.append(f"Perception error: {error_type}")

        if self.radar_data is not None:
            radar_error = self._radar_error_summary(step)
            if radar_error and radar_error != "none":
                extras.append(f"Radar condition: {radar_error}")
            radar_mode_s = self._radar_mode_summary(step)
            if radar_mode_s:
                extras.append(f"Radar mode: {radar_mode_s}")

        quality_action = info_row.get("quality_action_taken")
        if quality_action not in (None, "", "None"):
            extras.append(f"Quality action: {quality_action}")

        sm_val = info_row.get("safety_margin_applied")
        if sm_val not in (None, ""):
            try:
                sm_mode = info_row.get("safety_margin_mode", "")
                extras.append(f"Safety margin ({sm_mode}): {float(sm_val):.2f}")
            except (TypeError, ValueError):
                pass

        if info_row.get("abstention_flag") in ("True", True):
            kind = ("correct" if info_row.get("correct_abstention_flag") in ("True", True)
                    else "unnecessary" if info_row.get("unnecessary_abstention_flag") in ("True", True)
                    else "abstained")
            extras.append(f"Abstention: {kind}")

        if info_row.get("handoff_success_flag") in ("True", True):
            extras.append("Handoff: success")
        elif info_row.get("handoff_failure_flag") in ("True", True):
            extras.append("Handoff: failed")

        if info_row.get("critical_mode_flag") in ("True", True):
            extras.append("System mode: CRITICAL")
        elif info_row.get("degraded_mode_flag") in ("True", True):
            extras.append("System mode: degraded")
        rec = info_row.get("recovery_time_steps")
        if rec not in (None, "", "None"):
            try:
                extras.append(f"Recovery: {int(float(rec))} step(s)")
            except (TypeError, ValueError):
                pass

        reg_s = self._registration_summary()
        if reg_s:
            extras.append(f"Sensor registration: {reg_s}")

        if self.fused_data is not None:
            arch_summary = self._fusion_architecture_summary(step)
            if arch_summary:
                extras.append(f"Fusion architecture: {arch_summary}")

        lines.extend(extras)
        lines.append(f"Mission: {mission_status}")

        self.info_text.set_text("\n".join(lines))
        if mission_status == "SUCCESS":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightgreen", alpha=0.7))
        elif mission_status == "FAILURE":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightcoral", alpha=0.7))
        else:
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="wheat", alpha=0.5))

        self.fig.canvas.draw_idle()

    def _draw_radar_overlay(self, step: int, uav_id: int, x: float, y: float,
                             gx: float, gy: float, color, row: Dict):
        """Draws everything radar-related for one UAV at one step: sensing
        range, field-of-view wedge, this step's raw detections (color-coded
        by real/false-alarm/clutter/missed), this step's radar tracks
        (predicted vs. filtered position, status, covariance ellipse, track
        history trail, and a false-track heuristic label), and a "fused"
        tag on the perceived-obstacle marker when fusion is active for this
        run. All artists it creates are appended to self.radar_artists so
        render_step() can wipe them clean again next frame - nothing here
        is persistent."""
        # Sensing range: faint fill + a thin ring so it reads at a glance
        # without drowning out the UAV/obstacle markers underneath it.
        if self.radar_range > 0:
            fill = patches.Circle((x, y), self.radar_range, color=color,
                                   alpha=0.05, linewidth=0)
            ring = patches.Circle((x, y), self.radar_range, color=color,
                                   alpha=0.3, fill=False, linestyle=":", linewidth=0.8)
            self.ax.add_patch(fill)
            self.ax.add_patch(ring)
            self.radar_artists.extend([fill, ring])

        # Field of view: only draw a wedge when it's actually restricted
        # (360 = omnidirectional = no-op, matches radar_like_model.py).
        # Heading is approximated the same way radar_like_model.py does:
        # direction from the UAV to its own goal slot.
        if self.radar_fov_deg < 360.0:
            heading_deg = math.degrees(math.atan2(gy - y, gx - x))
            half = self.radar_fov_deg / 2.0
            wedge = patches.Wedge((x, y), self.radar_range, heading_deg - half,
                                   heading_deg + half, color=color, alpha=0.10)
            self.ax.add_patch(wedge)
            self.radar_artists.append(wedge)

        # This step's raw radar detections for this UAV.
        for d in self.radar_data.detections.get((step, uav_id), []):
            status = d.get("detection_status")
            # Task 26: ghost returns — drawn as a hollow triangle ("v") in
            # orange, distinct from clutter ("^") and real detections ("x").
            if d.get("ghost_flag") in ("True", True):
                dx, dy = float(d["detected_x"]), float(d["detected_y"])
                (pt,) = self.ax.plot(dx, dy, "v", color="darkorange", markersize=7,
                                      alpha=0.85, markeredgecolor="saddlebrown",
                                      markeredgewidth=0.8)
                self.radar_artists.append(pt)
                ghost_type = d.get("ghost_type", "ghost")
                lbl = self.ax.text(dx + 0.6, dy + 0.6, f"ghost:{ghost_type[:4]}",
                                    fontsize=self.FONT_DETAIL, color="darkorange")
                self.radar_artists.append(lbl)
                continue

            if status == "detected":
                dx, dy = float(d["detected_x"]), float(d["detected_y"])
                (pt,) = self.ax.plot(dx, dy, "x", color=color, markersize=6,
                                      markeredgewidth=1.5)
                self.radar_artists.append(pt)
                conf = d.get("confidence_score")
                if conf not in (None, ""):
                    # Task 26: calibrated confidence — color the label green
                    # when confidence_correct=True (genuine target), red when
                    # False (false alarm that reported high confidence), or
                    # the UAV's own color when unknown (field absent).
                    cal = d.get("confidence_correct")
                    if cal in ("True", True):
                        lbl_color = "limegreen"
                    elif cal in ("False", False):
                        lbl_color = "red"
                    else:
                        lbl_color = color
                    # Task 26: Doppler ambiguity — append "!dop" to the
                    # confidence label so the reader can see which detections
                    # had an aliased radial-velocity measurement.
                    dop_s = "!dop" if d.get("doppler_ambiguity_flag") in ("True", True) else ""
                    lbl = self.ax.text(dx + 0.6, dy + 0.6,
                                        f"R:{float(conf):.2f}{dop_s}",
                                        fontsize=self.FONT_DETAIL, color=lbl_color)
                    self.radar_artists.append(lbl)
            elif status == "false_alarm":
                dx, dy = float(d["detected_x"]), float(d["detected_y"])
                is_clutter = d.get("clutter_flag") in ("True", True)
                fa_color = "purple" if is_clutter else "magenta"
                (pt,) = self.ax.plot(dx, dy, "^", color=fa_color, markersize=6, alpha=0.85)
                self.radar_artists.append(pt)
                lbl = self.ax.text(dx + 0.6, dy + 0.6, "clutter" if is_clutter else "false+",
                                    fontsize=self.FONT_DETAIL, color=fa_color)
                self.radar_artists.append(lbl)
            elif status == "missed":
                # Mark where the real target was, to show what the radar
                # should have seen this scan but didn't.
                tx, ty = d.get("true_target_x"), d.get("true_target_y")
                if tx not in (None, "") and ty not in (None, ""):
                    tx, ty = float(tx), float(ty)
                    (pt,) = self.ax.plot(tx, ty, "o", markerfacecolor="none",
                                          markeredgecolor="gray", markersize=8)
                    self.radar_artists.append(pt)
                    lbl = self.ax.text(tx + 0.6, ty - 1.2, "missed", fontsize=self.FONT_DETAIL, color="gray")
                    self.radar_artists.append(lbl)

        # This step's radar tracks for this UAV (from radar_track_model.py).
        track_colors = {"tentative": "gray", "confirmed": color,
                         "coasting": "slateblue", "lost": "red", "deleted": "black"}
        for t in self.radar_data.tracks.get((step, uav_id), []):
            tx, ty = float(t["est_x"]), float(t["est_y"])
            status = t.get("status", "tentative")
            tcolor = track_colors.get(status, "gray")

            # "coasting" = this step had no matching detection, so est_x/y
            # is a pure Kalman *prediction*; every other status means it
            # was actually Kalman-*updated* (filtered) against a real
            # detection this step.
            is_predicted = status == "coasting"
            marker_shape = "d" if is_predicted else "D"
            facecolor = "none" if is_predicted else tcolor
            (pt,) = self.ax.plot(tx, ty, marker_shape, color=tcolor,
                                  markerfacecolor=facecolor, markersize=7,
                                  alpha=0.5 if status in ("lost", "deleted") else 0.9)
            self.radar_artists.append(pt)

            # Covariance ellipse: top-left 2x2 (position) block of the
            # filter's 4x4 state covariance.
            if self.show_covariance:
                cov_json = t.get("covariance")
                if cov_json:
                    try:
                        cov4 = json.loads(cov_json)
                        cov2 = [[cov4[0][0], cov4[0][1]], [cov4[1][0], cov4[1][1]]]
                        ell = _covariance_ellipse(
                            tx, ty, cov2, n_std=2.0, edgecolor=tcolor, facecolor=tcolor,
                            linestyle="-", linewidth=0.8, alpha=0.12)
                        if ell is not None:
                            self.ax.add_patch(ell)
                            self.radar_artists.append(ell)
                    except (ValueError, TypeError, IndexError, json.JSONDecodeError):
                        pass

            # Track history trail: every est_x/y this track has had, up to
            # (and including) the current step.
            if self.show_track_history:
                track_id = t.get("track_id")
                hist = self._track_history.get(track_id, [])
                hist_upto = [(hx, hy) for (hs, hx, hy) in hist if hs <= step]
                if len(hist_upto) > 1:
                    hxs, hys = zip(*hist_upto)
                    (trail,) = self.ax.plot(hxs, hys, "-", color=tcolor, alpha=0.3, linewidth=1.0)
                    self.radar_artists.append(trail)

            # False-track heuristic: the nearest same-step detection this
            # track sits on top of was itself flagged as a false alarm /
            # clutter return, so this is likely a track locked onto a
            # spurious return rather than a real target. Approximate, since
            # the track log doesn't retain a direct link to its originating
            # detection - proximity to a same-step false-alarm return is the
            # best signal available without changing the upstream schema.
            false_track = False
            for d in self.radar_data.detections.get((step, uav_id), []):
                if d.get("detection_status") != "false_alarm":
                    continue
                dxv, dyv = d.get("detected_x"), d.get("detected_y")
                if dxv in (None, "") or dyv in (None, ""):
                    continue
                if math.hypot(float(dxv) - tx, float(dyv) - ty) < 1.0:
                    false_track = True
                    break

            # Marker shape/color already encode predicted-vs-filtered and
            # status (see track_colors / is_predicted above and the
            # legend), so the label itself stays short: track ID,
            # confidence, and a "?" flag for a likely false track.
            conf = t.get("confidence")
            conf_s = f" c={float(conf):.2f}" if conf not in (None, "") else ""
            false_s = "?" if false_track else ""
            lbl = self.ax.text(
                tx + 0.7, ty + 0.7, f"{t.get('track_id', '')}{false_s}{conf_s}",
                fontsize=self.FONT_DETAIL, color="crimson" if false_track else tcolor)
            self.radar_artists.append(lbl)

        # Tag the existing perceived-obstacle marker as "fused" when this
        # run's fusion mode actually combined more than one UAV's view.
        if row.get("fusion_mode", "no_fusion") != "no_fusion":
            px, py = row.get("perceived_obstacle_x"), row.get("perceived_obstacle_y")
            if px not in (None, "") and py not in (None, ""):
                lbl = self.ax.text(float(px), float(py) + 1.6, "fused",
                                    fontsize=self.FONT_DETAIL, color=color, ha="center")
                self.radar_artists.append(lbl)

    def _draw_aux_overlay(self, sensor_data: "AuxSensorData", marker_char: str,
                           step: int, uav_id: int, color):
        """Draws one auxiliary point-sensor's (vision or LiDAR) measured
        detection position, confidence/reliability ("trust"), covariance
        ellipse, and stale-measurement indicator for one UAV at one step.
        Rows with no measured_x/y this step (out of FOV/range, occluded, a
        missed detection roll, weather dropout, ...) are skipped - there's
        nothing to plot for them."""
        label = sensor_data.sensor_label
        base_color = "saddlebrown" if label == "lidar" else "teal"
        for d in sensor_data.detections.get((step, uav_id), []):
            mx, my = d.get("measured_x"), d.get("measured_y")
            if mx in (None, "") or my in (None, ""):
                continue
            mx, my = float(mx), float(my)

            is_stale = d.get("is_stale") in ("True", True)
            is_clutter = d.get("is_clutter") in ("True", True)
            conf = d.get("confidence_score")
            rel = d.get("sensor_reliability")

            mcolor = "darkorange" if is_clutter else base_color
            alpha = 0.35 if is_stale else 0.9
            (pt,) = self.ax.plot(mx, my, marker_char, color=mcolor, markersize=7, alpha=alpha)
            self.radar_artists.append(pt)

            tag = label[0].upper()
            conf_s = f"{float(conf):.2f}" if conf not in (None, "") else "?"
            rel_s = f"/t={float(rel):.2f}" if rel not in (None, "") else ""
            stale_s = " *STALE*" if is_stale else ""
            lbl = self.ax.text(mx + 0.5, my - 0.9, f"{tag}:{conf_s}{rel_s}{stale_s}",
                                fontsize=self.FONT_DETAIL, color=mcolor)
            self.radar_artists.append(lbl)

            if self.show_covariance:
                cov_json = d.get("covariance")
                if cov_json:
                    try:
                        cov3 = json.loads(cov_json)
                        # Both vision's and LiDAR's 3x3 covariance carry
                        # their position-uncertainty terms in the top-left
                        # 2x2 block (isotropic for vision; range/position
                        # for LiDAR, used here as an x/y proxy).
                        cov2 = [[cov3[0][0], 0.0], [0.0, cov3[1][1]]]
                        ell = _covariance_ellipse(
                            mx, my, cov2, n_std=2.0, edgecolor=mcolor, facecolor="none",
                            linestyle=":", linewidth=0.8, alpha=0.5)
                        if ell is not None:
                            self.ax.add_patch(ell)
                            self.radar_artists.append(ell)
                    except (ValueError, TypeError, IndexError, json.JSONDecodeError):
                        pass

    def _draw_fusion_overlay(self, step: int, colors, uav_positions: Dict[int, Tuple[float, float]]):
        """Draws every fused estimate reported for this step: a star marker
        (gold for a single shared centralized estimate, UAV-colored for
        each UAV's own local distributed estimate), its confidence/#
        sources/staleness, an approximate covariance ellipse when a
        position_variance is available, and - for the distributed
        architecture - the inter-UAV communication links this estimate
        actually drew on, plus a dropped-link indicator for swarm-mates
        that didn't make it into this UAV's local fusion this step."""
        rows = self.fused_data.rows_by_step.get(step, [])
        if not rows:
            return

        all_uav_ids = list(uav_positions.keys())

        for row in rows:
            arch = row.get("architecture", "centralized")
            local_uav = row.get("local_uav_id")
            is_centralized = local_uav in (None, "", "None")

            try:
                fx, fy = float(row["fused_x"]), float(row["fused_y"])
            except (KeyError, TypeError, ValueError):
                continue

            conf = row.get("fused_confidence")
            n_src = row.get("num_sources")
            is_stale = row.get("is_stale") in ("True", True)

            if is_centralized:
                mcolor = "gold"
                anchor_uav = None
            else:
                anchor_uav = int(local_uav)
                mcolor = colors[anchor_uav % len(colors)]

            (pt,) = self.ax.plot(
                fx, fy, "*", color=mcolor, markersize=15 if is_centralized else 12,
                alpha=0.4 if is_stale else 0.95, markeredgecolor="black", markeredgewidth=0.6)
            self.radar_artists.append(pt)

            # Architecture (centralized/distributed) is shown once in the
            # info panel rather than repeated on every marker.
            tag = "FUSED" if is_centralized else f"FUSED[U{anchor_uav}]"
            conf_s = f" c={float(conf):.2f}" if conf not in (None, "") else ""
            n_s = f" n={n_src}" if n_src not in (None, "") else ""
            stale_s = " STALE" if is_stale else ""
            lbl = self.ax.text(
                fx + 0.7, fy - 1.1, f"{tag}{n_s}{conf_s}{stale_s}",
                fontsize=self.FONT_DETAIL, color=mcolor, fontweight="bold")
            self.radar_artists.append(lbl)

            if self.show_covariance:
                pos_var = row.get("position_variance")
                if pos_var not in (None, ""):
                    try:
                        var = float(pos_var)
                        if var > 0:
                            side = 2 * 2.0 * math.sqrt(var)  # 2-sigma, isotropic approximation
                            ellipse = patches.Ellipse(
                                (fx, fy), width=side, height=side, angle=0,
                                edgecolor=mcolor, facecolor="none", linestyle="--",
                                linewidth=1.0, alpha=0.5)
                            self.ax.add_patch(ellipse)
                            self.radar_artists.append(ellipse)
                    except (TypeError, ValueError):
                        pass

            # Inter-UAV communication links + packet-dropout indication:
            # only meaningful for the distributed architecture, where each
            # UAV's local fused estimate is built from whatever peer
            # broadcasts actually arrived this step.
            if arch == "distributed" and anchor_uav is not None and anchor_uav in uav_positions:
                contributors = _parse_contributor_uav_ids(row.get("source_track_ids"))
                contributors.discard(anchor_uav)
                ax_pos = uav_positions[anchor_uav]
                for other in all_uav_ids:
                    if other == anchor_uav or other not in uav_positions:
                        continue
                    ox2, oy2 = uav_positions[other]
                    delivered = other in contributors
                    (line,) = self.ax.plot(
                        [ax_pos[0], ox2], [ax_pos[1], oy2],
                        color=(colors[other % len(colors)] if delivered else "red"),
                        linestyle="-" if delivered else ":",
                        alpha=0.35 if delivered else 0.55,
                        linewidth=1.2 if delivered else 1.0)
                    self.radar_artists.append(line)
                    if not delivered:
                        mx, my = (ax_pos[0] + ox2) / 2.0, (ax_pos[1] + oy2) / 2.0
                        drop_lbl = self.ax.text(
                            mx, my, "x", fontsize=self.FONT_ENTITY_ID, color="red",
                            ha="center", va="center", fontweight="bold")
                        self.radar_artists.append(drop_lbl)

    def _fusion_architecture_summary(self, step: int) -> str:
        """One-line architecture/comm summary for the info box: which
        architecture is active this step, and (for distributed) how many
        of the attempted inter-UAV broadcasts were actually delivered."""
        rows = self.fused_data.rows_by_step.get(step, [])
        if not rows:
            return ""
        archs = {r.get("architecture", "centralized") for r in rows}
        if "distributed" in archs:
            attempted = next((r.get("comm_messages") for r in rows if r.get("comm_messages") not in (None, "")), None)
            delivered = next((r.get("comm_messages_delivered") for r in rows
                               if r.get("comm_messages_delivered") not in (None, "")), None)
            if attempted not in (None, "") and delivered not in (None, ""):
                return f"distributed ({delivered}/{attempted} msgs delivered)"
            return "distributed"
        return "centralized"

    def _radar_error_summary(self, step: int) -> str:
        """Aggregates this step's radar-level error flags (dropout, false
        alarm, clutter, missed, P_D miss, ghost, Doppler ambiguity) across
        every UAV's detections into one short string for the info box."""
        flags = set()
        for uav_id in range(self.data.num_uavs):
            for d in self.radar_data.detections.get((step, uav_id), []):
                if d.get("dropout_flag") in ("True", True):
                    flags.add("dropout")
                if d.get("false_alarm_flag") in ("True", True):
                    flags.add("clutter" if d.get("clutter_flag") in ("True", True) else "false_alarm")
                if d.get("missed_detection_flag") in ("True", True):
                    flags.add("missed")
                if d.get("radar_pd_miss_flag") in ("True", True):
                    flags.add("pd_miss")
                # Task 26: ghost returns and Doppler ambiguity.
                if d.get("ghost_flag") in ("True", True):
                    flags.add("ghost")
                if d.get("doppler_ambiguity_flag") in ("True", True):
                    flags.add("dop_ambig")
        return "+".join(sorted(flags)) if flags else "none"

    def _radar_mode_summary(self, step: int) -> str:
        """Task 26: returns a one-line string showing the radar operating
        mode and reliability state this step, drawn from the first
        detection row for any UAV that has one. Returns '' if neither
        field is available (e.g. no radar data)."""
        for uav_id in range(self.data.num_uavs):
            dets = self.radar_data.detections.get((step, uav_id), [])
            for d in dets:
                mode = d.get("radar_operating_mode", "")
                rel = d.get("radar_reliability_state", "")
                env = d.get("radar_environmental_condition", "")
                parts = [p for p in [mode, rel, env] if p and p not in ("", "nominal", "normal", "clear")]
                # Always show mode even if it's normal, so the viewer can
                # confirm which mode is active; suppress nominal/clear/normal
                # for reliability/env to avoid clutter when everything is OK.
                return f"{mode or '?'}" + (f" | {'+'.join(parts[1:] if mode else parts)}" if parts[1:] else "")
        return ""

    def _registration_summary(self) -> str:
        """Task 26: returns a short string describing the cross-modal
        registration offset for this scenario, if the visualizer was
        constructed with a config dict that carries one. Reads
        self._scenario_config, which is set by SimulationVisualizer.__init__
        only when a caller passes scenario_config= (absent in batch mode).
        Registration drift is not reflected per-step since it's a static
        config property; this is only meant to remind the viewer that
        mis-registration is active for the run."""
        cfg = getattr(self, "_scenario_config", None)
        if cfg is None:
            return ""
        reg = cfg.get("cross_modal_registration", {})
        if not reg.get("enabled", False):
            return ""
        ox = reg.get("radar_to_vision_x_offset", 0.0)
        oy = reg.get("radar_to_vision_y_offset", 0.0)
        rot = reg.get("rotation_error_deg", 0.0)
        offset_m = math.hypot(ox, oy)
        parts = [f"offset={offset_m:.1f}m"]
        if rot:
            parts.append(f"rot={rot:.0f}deg")
        frame_mis = reg.get("lidar_coordinate_frame_mismatch", "none")
        if frame_mis and frame_mis != "none":
            parts.append(frame_mis)
        return " ".join(parts)

    def add_legend(self):
        """Add a legend to the plot."""
        legend_elements = [
            patches.Patch(facecolor="red", alpha=0.6, label="Actual Obstacle"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=8, label="UAV"),
            Line2D([0], [0], marker="s", color="w", markerfacecolor="none", markeredgecolor="gray", markersize=8, label="Goal"),
            Line2D([0], [0], color="gray", linestyle="--", label="Perceived Obstacle"),
            patches.Patch(facecolor="orange", alpha=0.2, label="Collision-Risk Zone"),
        ]
        if self.radar_data is not None:
            legend_elements += [
                patches.Patch(facecolor="gray", alpha=0.15, label="Radar Sensing Range"),
                Line2D([0], [0], marker="x", color="w", markeredgecolor="gray", markersize=8, label="Radar Detection"),
                Line2D([0], [0], marker="^", color="w", markerfacecolor="magenta", markersize=8, label="False Alarm / Clutter"),
                Line2D([0], [0], marker="o", color="w", markerfacecolor="none", markeredgecolor="gray", markersize=8, label="Missed Detection"),
                Line2D([0], [0], marker="d", color="w", markerfacecolor="none", markeredgecolor="slateblue", markersize=8, label="Predicted Track (coasting)"),
                Line2D([0], [0], marker="D", color="w", markerfacecolor="gray", markersize=8, label="Filtered Track (updated)"),
                patches.Patch(facecolor="gray", alpha=0.15, label="Track Covariance (2-sigma)"),
                Line2D([0], [0], color="gray", alpha=0.4, label="Track History"),
                Line2D([0], [0], marker="v", color="w", markerfacecolor="darkorange", markersize=8, label="Ghost Return (multipath/side-lobe)"),
            ]
        if self.vision_data is not None:
            legend_elements.append(
                Line2D([0], [0], marker="*", color="w", markerfacecolor="teal", markersize=9, label="Vision Detection"))
        if self.lidar_data is not None:
            legend_elements.append(
                Line2D([0], [0], marker="P", color="w", markerfacecolor="saddlebrown", markersize=9, label="LiDAR Detection"))
        if self.fused_data is not None:
            legend_elements += [
                Line2D([0], [0], marker="*", color="w", markerfacecolor="gold", markersize=11, label="Fused Track (centralized)"),
                Line2D([0], [0], marker="*", color="w", markerfacecolor="gray", markersize=11, label="Fused Track (distributed, per-UAV)"),
                Line2D([0], [0], color="gray", alpha=0.4, label="Comm Link (delivered, colored by sender)"),
                Line2D([0], [0], color="red", linestyle=":", label="Comm Link (dropped)"),
            ]
        self.ax.legend(handles=legend_elements, loc="upper right", fontsize=self.FONT_LEGEND)

    def save_animation(self, output_path: str, fps: int = 5, dpi: int = 80):
        """Generate and save animation as MP4 or GIF."""
        print("Creating animation... this may take a while")
        self.add_legend()
        self.fig.tight_layout()

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
                    media_dir: str = "media", fps: int = 5, show_popups: bool = True,
                    radar_log: Optional[str] = None, track_log: Optional[str] = None,
                    vision_log: Optional[str] = None, lidar_log: Optional[str] = None,
                    fused_log: Optional[str] = None) -> int:
    """Default (no-args) behavior: for every scenario declared in the config,
    load its run1 log, pop up an auto-playing preview, and save an MP4 to
    media/. Scenario list is read from the config (not hardcoded) so this
    stays in sync with whatever scenarios simple_swarm_sim.py/simulation_config.json
    define - including newly added ones like no_fusion_matched. Also
    generates a side-by-side fusion-mode comparison video if enough of
    those scenarios' logs are present.

    radar_log/track_log/vision_log/lidar_log/fused_log (if given and
    present on disk) are the combined, multi-scenario CSVs the
    corresponding model scripts write; each scenario's rows are filtered
    out of them automatically, and the sensing range/FOV drawn per
    scenario come from this same config file via _resolve_radar_params(),
    so the overlay always matches what that scenario's radar model
    actually used."""
    with open(config_path) as f:
        config = json.load(f)
    scenario_names = list(config["scenarios"].keys())
    os.makedirs(media_dir, exist_ok=True)

    print(f"Batch mode: {len(scenario_names)} scenario(s) from {config_path}\n")
    if radar_log and os.path.exists(radar_log):
        print(f"  radar overlay: {radar_log}" + (f" + {track_log}" if track_log else "") + "\n")
    if vision_log and os.path.exists(vision_log):
        print(f"  vision overlay: {vision_log}\n")
    if lidar_log and os.path.exists(lidar_log):
        print(f"  lidar overlay: {lidar_log}\n")
    if fused_log and os.path.exists(fused_log):
        print(f"  fused-track overlay: {fused_log}\n")

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

        radar_data = None
        radar_range, radar_fov = 15.0, 360.0
        if radar_log:
            radar_data = RadarData.from_csvs(name, radar_log, track_log)
            radar_range, radar_fov = _resolve_radar_params(config, name)

        vision_data = AuxSensorData.from_csv(vision_log, name, "vision") if vision_log else None
        lidar_data = AuxSensorData.from_csv(lidar_log, name, "lidar") if lidar_log else None
        fused_data = FusedTrackData.from_csv(fused_log, name) if fused_log else None

        if show_popups:
            viz = SimulationVisualizer(data, radar_data=radar_data,
                                        radar_range=radar_range, radar_fov_deg=radar_fov,
                                        vision_data=vision_data, lidar_data=lidar_data,
                                        fused_data=fused_data)
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
        viz = SimulationVisualizer(data, radar_data=radar_data,
                                    radar_range=radar_range, radar_fov_deg=radar_fov,
                                    vision_data=vision_data, lidar_data=lidar_data,
                                    fused_data=fused_data)
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


# ---------------------------------------------------------------------------
# Advanced demo video suite (Task 22)
# ---------------------------------------------------------------------------

def _run_full_stack(config: dict, scenario_name: str, architecture: str = "centralized",
                     seed: Optional[int] = None, fusion_mode_override: Optional[str] = None,
                     use_adaptive_trust: bool = True):
    """Runs the sensor -> Kalman-track -> fusion pipeline for one
    scenario/architecture entirely in memory (Simulation -> RadarLikeModel
    -> radar_track_model.build_tracks -> fusion_model.build_fused_log) and
    returns (SimulationData, RadarData, FusedTrackData, radar_model) ready
    to hand straight to SimulationVisualizer - no pre-existing CSV logs
    required. Used by generate_advanced_demo_videos below."""
    from models.radar_like_model import RadarLikeModel
    from tracking.radar_track_model import build_tracks
    from fusion.fusion_model import build_fused_log

    cfg = copy.deepcopy(config)
    if seed is not None:
        cfg.setdefault("sim", {})["seed"] = seed
    if fusion_mode_override is not None:
        cfg["scenarios"][scenario_name]["fusion_mode"] = fusion_mode_override

    radar_model = RadarLikeModel(cfg, scenario_name)
    detection_rows = radar_model.run()
    dt = cfg["sim"]["dt"]
    track_rows = build_tracks(scenario_name, detection_rows, dt, radar_model.range_noise_std)
    fused_rows = build_fused_log(scenario_name, cfg, architecture=architecture,
                                  seed=seed, use_adaptive_trust=use_adaptive_trust)

    sim_data = SimulationData.from_rows(radar_model.sim.log_rows)
    radar_data = RadarData.from_rows(detection_rows, track_rows)
    fused_data = FusedTrackData.from_rows(fused_rows)

    return sim_data, radar_data, fused_data, radar_model


# name -> (scenario, architecture, display title). Scenario names match
# simulation_config.json; each is chosen to specifically exercise the demo
# it's named for (see simulation_config.json for the full scenario set).
ADVANCED_DEMOS = [
    dict(name="kalman_tracking_example", scenario="baseline",
         architecture="centralized", title="Kalman Tracking Example"),
    dict(name="target_crossing_example", scenario="target_crossing",
         architecture="centralized", title="Target Crossing Example"),
    dict(name="clutter_stress_test", scenario="high_clutter",
         architecture="centralized", title="Clutter Stress Test"),
    dict(name="sensor_dropout_recovery", scenario="target_reappearing_after_dropout",
         architecture="centralized", title="Sensor Dropout Recovery"),
    dict(name="overconfident_faulty_sensor", scenario="overconfident_faulty_sensor",
         architecture="centralized", title="Overconfident Faulty Sensor"),
    dict(name="dynamic_trust_adaptation", scenario="faulty_sensor_trust_weighted_fusion_dynamic",
         architecture="centralized", title="Dynamic Trust Adaptation"),
    dict(name="communication_outage_scenario", scenario="communication_outage",
         architecture="distributed", title="Communication Outage Scenario"),
]


# ---------------------------------------------------------------------------
# Task 26: Final demo video suite
# ---------------------------------------------------------------------------

# Each entry describes one single-panel video. Entries that need a scenario
# config override carry an `override` dict that is deep-merged on top of the
# named scenario's existing config before the run; this lets us enable ghost
# detection or Doppler aliasing without adding a dedicated scenario to the
# config file.
#
# Side-by-side videos (calibrated vs overconfident, safety margin comparison,
# swarm-size comparison) are handled separately in generate_final_demo_videos
# below using a dedicated panel-list approach, same as the existing
# centralized-vs-distributed comparison video.

FINAL_DEMOS = [
    dict(name="radar_ghost_return", scenario="high_clutter",
         architecture="centralized", title="Ghost Radar Returns (multipath/side-lobe)",
         override={"ghost_detection_enabled": True, "ghost_probability": 0.35}),
    dict(name="doppler_ambiguity", scenario="rapidly_moving_obstacle",
         architecture="centralized", title="Doppler Ambiguity",
         override={"doppler_aliasing_enabled": True,
                   "max_unambiguous_radial_velocity": 1.5}),
    dict(name="cross_modal_registration_failure", scenario="registration_severe_error",
         architecture="centralized", title="Cross-Modal Registration Failure",
         override={}),
    dict(name="abstention_case", scenario="simultaneous_sensor_failures",
         architecture="centralized", title="Abstention Under Combined Faults",
         override={}),
    dict(name="centralized_handoff_case", scenario="communication_outage",
         architecture="distributed", title="Centralized Handoff Under Comms Outage",
         override={}),
    dict(name="combined_fault_recovery", scenario="simultaneous_sensor_failures",
         architecture="centralized", title="Combined-Fault Recovery",
         override={"safety_margin_mode": "quality_monitor"}),
]


def generate_advanced_demo_videos(config_path: str = "simulation_config.json",
                                   media_dir: str = "media", fps: int = 5,
                                   figsize: Tuple[int, int] = (12, 10),
                                   seed: Optional[int] = None) -> Dict[str, bool]:
    """Generates the full suite of advanced, sensor-fusion-aware demo
    videos: Kalman tracking, target crossing, a clutter stress test,
    sensor-dropout recovery, an overconfident faulty sensor, dynamic trust
    adaptation, a communication-outage scenario, and a dedicated
    centralized-vs-distributed fusion comparison. Every video is built
    entirely in memory via _run_full_stack (no pre-existing CSV logs
    needed) and renders every overlay this module supports: predicted/
    filtered track position, covariance ellipses, track status/history,
    false-track flags, fused tracks, and (for the distributed run)
    inter-UAV communication links with packet-dropout indication."""
    with open(config_path) as f:
        config = json.load(f)
    os.makedirs(media_dir, exist_ok=True)
    scenario_names = set(config["scenarios"].keys())

    ffmpeg_ok = shutil.which("ffmpeg") is not None
    if not ffmpeg_ok:
        print("NOTE: ffmpeg not found on PATH - advanced demo videos will be skipped.")

    results: Dict[str, bool] = {}

    for demo in ADVANCED_DEMOS:
        name, scenario, architecture = demo["name"], demo["scenario"], demo["architecture"]
        if scenario not in scenario_names:
            print(f"SKIP {name}: scenario '{scenario}' not found in {config_path}")
            results[name] = False
            continue

        print(f"{name}: running '{scenario}' [{architecture}] ...")
        try:
            sim_data, radar_data, fused_data, radar_model = _run_full_stack(
                config, scenario, architecture=architecture, seed=seed)
        except Exception as e:
            print(f"  FAILED to build data for {name}: {e}")
            results[name] = False
            continue

        sim_data.scenario_name = demo["title"]

        if not ffmpeg_ok:
            results[name] = False
            continue

        viz = SimulationVisualizer(
            sim_data, figsize=figsize, radar_data=radar_data,
            radar_range=radar_model.radar_max_range,
            radar_fov_deg=radar_model.radar_field_of_view,
            fused_data=fused_data)
        out_path = os.path.join(media_dir, f"{name}.mp4")
        try:
            viz.save_animation(out_path, fps=fps, dpi=100)
            results[name] = True
        except Exception as e:
            print(f"  video save failed for {name}: {e}")
            results[name] = False

    # Dedicated centralized-vs-distributed comparison, sharing one figure -
    # same scenario, run once under each fusion architecture.
    print("centralized_vs_distributed_fusion: building both architectures ...")
    try:
        comparison_scenario = ("trust_weighted_fusion" if "trust_weighted_fusion" in scenario_names
                                else next(iter(scenario_names)))
        panels = []
        for architecture in ("centralized", "distributed"):
            sim_data, radar_data, fused_data, radar_model = _run_full_stack(
                config, comparison_scenario, architecture=architecture, seed=seed)
            sim_data.scenario_name = f"{comparison_scenario} [{architecture}]"
            panels.append((architecture, sim_data, radar_data, fused_data, radar_model))

        if ffmpeg_ok:
            fig, axes = plt.subplots(1, len(panels), figsize=(6.5 * len(panels), 6.5))
            if len(panels) == 1:
                axes = [axes]
            visualizers = []
            for ax, (architecture, sim_data, radar_data, fused_data, radar_model) in zip(axes, panels):
                viz = SimulationVisualizer(
                    sim_data, fig=fig, ax=ax, radar_data=radar_data,
                    radar_range=radar_model.radar_max_range,
                    radar_fov_deg=radar_model.radar_field_of_view,
                    fused_data=fused_data)
                visualizers.append(viz)
            max_steps = max(v.data.steps for v in visualizers)

            def animate(frame):
                for v in visualizers:
                    step = min(frame, v.data.steps - 1)
                    v.render_step(step)
                return []

            anim = animation.FuncAnimation(fig, animate, frames=max_steps,
                                            interval=1000 // fps, repeat=True)
            fig.suptitle("Centralized vs Distributed Fusion", fontsize=14, fontweight="bold")
            fig.tight_layout(rect=[0, 0.05, 1, 0.95])
            out_path = os.path.join(media_dir, "centralized_vs_distributed_fusion.mp4")
            writer = animation.FFMpegWriter(fps=fps)
            anim.save(out_path, writer=writer, dpi=100)
            plt.close(fig)
            results["centralized_vs_distributed_fusion"] = True
            print(f"  saved {out_path}")
        else:
            results["centralized_vs_distributed_fusion"] = False
    except Exception as e:
        print(f"  FAILED centralized_vs_distributed_fusion: {e}")
        results["centralized_vs_distributed_fusion"] = False

    success = sum(1 for v in results.values() if v)
    print(f"\nAdvanced demo videos: {success}/{len(results)} saved to {media_dir}/")
    return results


def generate_final_demo_videos(config_path: str = "simulation_config.json",
                               media_dir: str = "media", fps: int = 5,
                               figsize: Tuple[int, int] = (12, 10),
                               seed: Optional[int] = None) -> Dict[str, bool]:
    """Task 26: generates the final demo video suite:

    Single-panel videos (FINAL_DEMOS list):
      - radar_ghost_return: ghost detections from multipath/side-lobe effects
      - doppler_ambiguity: Doppler velocity aliasing on a fast-moving target
      - cross_modal_registration_failure: severe radar<->vision mis-registration
      - abstention_case: UAV abstains under combined sensor faults
      - centralized_handoff_case: distributed->centralized handoff on comms loss
      - combined_fault_recovery: quality-monitor safety margin + multi-fault

    Side-by-side comparison videos:
      - calibrated_vs_overconfident_radar: correctly_calibrated vs
        severely_overconfident_radar, highlighting the calibrated-confidence
        overlay difference.
      - adaptive_safety_margin_comparison: safety_margin_fixed vs
        safety_margin_quality_monitor, highlighting margin value differences.
      - swarm_size_comparison: baseline scenario run with 3, 5, and 8 UAVs
        (num_uavs overridden in config) to show scaling behavior.

    Every video uses the Task 26 overlays (ghost, Doppler, calibration color,
    safety margin, mode, registration) automatically via SimulationVisualizer.
    Scenario config overrides (FINAL_DEMOS[*]["override"]) let us enable ghost
    detection / Doppler aliasing without polluting the config file."""
    with open(config_path) as f:
        config = json.load(f)
    os.makedirs(media_dir, exist_ok=True)
    scenario_names = set(config["scenarios"].keys())

    ffmpeg_ok = shutil.which("ffmpeg") is not None
    if not ffmpeg_ok:
        print("NOTE: ffmpeg not found on PATH - final demo videos will be skipped.")

    results: Dict[str, bool] = {}

    # ---- Single-panel demos (same shape as ADVANCED_DEMOS) ----
    for demo in FINAL_DEMOS:
        name, scenario = demo["name"], demo["scenario"]
        override = demo.get("override", {})
        architecture = demo.get("architecture", "centralized")

        if scenario not in scenario_names:
            print(f"SKIP {name}: scenario '{scenario}' not found in {config_path}")
            results[name] = False
            continue

        print(f"{name}: running '{scenario}' [{architecture}] ...")
        try:
            # Deep-merge override keys into a copy of the scenario config so
            # the original config dict stays unmodified across iterations.
            cfg = copy.deepcopy(config)
            cfg["scenarios"][scenario].update(override)
            sim_data, radar_data, fused_data, radar_model = _run_full_stack(
                cfg, scenario, architecture=architecture, seed=seed)
        except Exception as e:
            print(f"  FAILED to build data for {name}: {e}")
            results[name] = False
            continue

        sim_data.scenario_name = demo["title"]

        if not ffmpeg_ok:
            results[name] = False
            continue

        scn_cfg = config["scenarios"].get(scenario, {})
        viz = SimulationVisualizer(
            sim_data, figsize=figsize, radar_data=radar_data,
            radar_range=radar_model.radar_max_range,
            radar_fov_deg=radar_model.radar_field_of_view,
            fused_data=fused_data,
            scenario_config=scn_cfg)  # Task 26: registration overlay
        out_path = os.path.join(media_dir, f"{name}.mp4")
        try:
            viz.save_animation(out_path, fps=fps, dpi=100)
            results[name] = True
        except Exception as e:
            print(f"  video save failed for {name}: {e}")
            results[name] = False

    # ---- calibrated vs overconfident radar (side-by-side) ----
    _comparison_panels = [
        ("correctly_calibrated_radar", "Calibrated Radar"),
        ("severely_overconfident_radar", "Severely Overconfident Radar"),
    ]
    _comparison_panels = [(s, t) for s, t in _comparison_panels if s in scenario_names]
    if len(_comparison_panels) >= 2 and ffmpeg_ok:
        print("calibrated_vs_overconfident_radar: building comparison ...")
        try:
            panels = []
            for scn, title in _comparison_panels:
                sd, rd, fd, rm = _run_full_stack(config, scn, architecture="centralized", seed=seed)
                sd.scenario_name = title
                panels.append((scn, sd, rd, fd, rm))
            fig, axes = plt.subplots(1, len(panels), figsize=(6.5 * len(panels), 6.5))
            if len(panels) == 1:
                axes = [axes]
            vizs = [SimulationVisualizer(sd, fig=fig, ax=ax, radar_data=rd,
                                          radar_range=rm.radar_max_range,
                                          radar_fov_deg=rm.radar_field_of_view,
                                          fused_data=fd)
                    for ax, (scn, sd, rd, fd, rm) in zip(axes, panels)]
            max_steps = max(v.data.steps for v in vizs)
            anim = animation.FuncAnimation(
                fig, lambda fr: [v.render_step(min(fr, v.data.steps - 1)) for v in vizs],
                frames=max_steps, interval=1000 // fps, repeat=True)
            fig.suptitle("Calibrated vs Overconfident Radar", fontsize=14, fontweight="bold")
            fig.tight_layout(rect=[0, 0, 1, 0.95])
            out = os.path.join(media_dir, "calibrated_vs_overconfident_radar.mp4")
            anim.save(out, writer=animation.FFMpegWriter(fps=fps), dpi=100)
            plt.close(fig)
            results["calibrated_vs_overconfident_radar"] = True
            print(f"  saved {out}")
        except Exception as e:
            print(f"  FAILED calibrated_vs_overconfident_radar: {e}")
            results["calibrated_vs_overconfident_radar"] = False
    else:
        results["calibrated_vs_overconfident_radar"] = False

    # ---- adaptive safety-margin comparison (side-by-side) ----
    _margin_panels = [
        ("safety_margin_fixed", "Fixed Safety Margin"),
        ("safety_margin_quality_monitor", "Quality-Monitor Margin"),
    ]
    _margin_panels = [(s, t) for s, t in _margin_panels if s in scenario_names]
    if len(_margin_panels) >= 2 and ffmpeg_ok:
        print("adaptive_safety_margin_comparison: building comparison ...")
        try:
            panels = []
            for scn, title in _margin_panels:
                sd, rd, fd, rm = _run_full_stack(config, scn, architecture="centralized", seed=seed)
                sd.scenario_name = title
                panels.append((scn, sd, rd, fd, rm))
            fig, axes = plt.subplots(1, len(panels), figsize=(6.5 * len(panels), 6.5))
            if len(panels) == 1:
                axes = [axes]
            vizs = [SimulationVisualizer(sd, fig=fig, ax=ax, radar_data=rd,
                                          radar_range=rm.radar_max_range,
                                          radar_fov_deg=rm.radar_field_of_view,
                                          fused_data=fd,
                                          scenario_config=config["scenarios"].get(scn, {}))
                    for ax, (scn, sd, rd, fd, rm) in zip(axes, panels)]
            max_steps = max(v.data.steps for v in vizs)
            anim = animation.FuncAnimation(
                fig, lambda fr: [v.render_step(min(fr, v.data.steps - 1)) for v in vizs],
                frames=max_steps, interval=1000 // fps, repeat=True)
            fig.suptitle("Adaptive Safety-Margin Comparison", fontsize=14, fontweight="bold")
            fig.tight_layout(rect=[0, 0, 1, 0.95])
            out = os.path.join(media_dir, "adaptive_safety_margin_comparison.mp4")
            anim.save(out, writer=animation.FFMpegWriter(fps=fps), dpi=100)
            plt.close(fig)
            results["adaptive_safety_margin_comparison"] = True
            print(f"  saved {out}")
        except Exception as e:
            print(f"  FAILED adaptive_safety_margin_comparison: {e}")
            results["adaptive_safety_margin_comparison"] = False
    else:
        results["adaptive_safety_margin_comparison"] = False

    # ---- swarm-size comparison (3 / 5 / 8 UAVs, side-by-side) ----
    # swarm_sizes is hardcoded to three representative sizes.
    # Upgrade path: make it a parameter if a sweep over more sizes is needed.
    swarm_sizes = [3, 5, 8]
    base_scn = "baseline"
    if base_scn in scenario_names and ffmpeg_ok:
        print("swarm_size_comparison: building 3-panel comparison ...")
        try:
            panels = []
            for n in swarm_sizes:
                cfg = copy.deepcopy(config)
                cfg["swarm"]["num_uavs"] = n
                # Ensure we have enough start positions (pad with offsets if needed)
                while len(cfg["swarm"]["start_positions"]) < n:
                    last = cfg["swarm"]["start_positions"][-1]
                    cfg["swarm"]["start_positions"].append(
                        [last[0] + 5.0, last[1]])
                cfg["swarm"]["start_positions"] = cfg["swarm"]["start_positions"][:n]
                sd, rd, fd, rm = _run_full_stack(cfg, base_scn, architecture="centralized", seed=seed)
                sd.scenario_name = f"Baseline ({n} UAVs)"
                panels.append((n, sd, rd, fd, rm))
            fig, axes = plt.subplots(1, len(panels), figsize=(6.0 * len(panels), 6.5))
            if len(panels) == 1:
                axes = [axes]
            vizs = [SimulationVisualizer(sd, fig=fig, ax=ax, radar_data=rd,
                                          radar_range=rm.radar_max_range,
                                          radar_fov_deg=rm.radar_field_of_view,
                                          fused_data=fd)
                    for ax, (n, sd, rd, fd, rm) in zip(axes, panels)]
            max_steps = max(v.data.steps for v in vizs)
            anim = animation.FuncAnimation(
                fig, lambda fr: [v.render_step(min(fr, v.data.steps - 1)) for v in vizs],
                frames=max_steps, interval=1000 // fps, repeat=True)
            fig.suptitle("Swarm-Size Comparison", fontsize=14, fontweight="bold")
            fig.tight_layout(rect=[0, 0, 1, 0.95])
            out = os.path.join(media_dir, "swarm_size_comparison.mp4")
            anim.save(out, writer=animation.FFMpegWriter(fps=fps), dpi=100)
            plt.close(fig)
            results["swarm_size_comparison"] = True
            print(f"  saved {out}")
        except Exception as e:
            print(f"  FAILED swarm_size_comparison: {e}")
            results["swarm_size_comparison"] = False
    else:
        results["swarm_size_comparison"] = False

    success = sum(1 for v in results.values() if v)
    print(f"\nFinal demo videos: {success}/{len(results)} saved to {media_dir}/")
    return results


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
        "--radar-log", default=None,
        help="Path to the combined radar detection CSV from radar_like_model.py "
        "(e.g. logs/radar_log.csv). When given, adds sensing range, FOV, "
        "detections, false alarms/clutter, missed-detection markers, and "
        "Kalman track overlays (predicted/filtered position, covariance "
        "ellipse, status, history, false-track flag) to the plot. Optional - "
        "omit to visualize without the radar overlay.",
    )
    parser.add_argument(
        "--track-log", default=None,
        help="Path to the combined radar track CSV from radar_track_model.py "
        "(e.g. logs/radar_track_log.csv). Adds track ID/status/confidence "
        "markers on top of --radar-log. Optional.",
    )
    parser.add_argument(
        "--vision-log", default=None,
        help="Path to the combined vision detection CSV from vision_like_model.py "
        "(e.g. logs/vision_log.csv). Adds vision measurement position, "
        "confidence, sensor-reliability ('trust'), covariance ellipse, and "
        "stale-measurement markers. Optional.",
    )
    parser.add_argument(
        "--lidar-log", default=None,
        help="Path to the combined LiDAR detection CSV from lidar_like_model.py "
        "(e.g. logs/lidar_log.csv). Same overlay as --vision-log, for LiDAR. "
        "Optional.",
    )
    parser.add_argument(
        "--fused-log", default=None,
        help="Path to the combined fused-track CSV from fusion_model.py "
        "(e.g. logs/fused_track_log.csv). Adds fused-track markers "
        "(centralized shared estimate, or per-UAV distributed estimates), "
        "fusion architecture labeling, and - for distributed rows - "
        "inter-UAV communication links with packet-dropout indication. "
        "Optional.",
    )
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
        "--advanced-demos", action="store_true",
        help="Generate the full advanced demo video suite (Kalman tracking, "
        "target crossing, clutter stress test, sensor-dropout recovery, "
        "overconfident faulty sensor, centralized-vs-distributed fusion, "
        "dynamic trust adaptation, communication outage) into --media-dir "
        "and exit. Builds everything in memory - no logs need to exist yet.",
    )
    parser.add_argument(
        "--final-demos", action="store_true",
        help="Task 26: generate the final demo video suite (calibrated vs "
        "overconfident radar, ghost-return, Doppler ambiguity, registration "
        "failure, adaptive safety margin, abstention, centralized handoff, "
        "combined-fault recovery, swarm-size comparison) into --media-dir "
        "and exit. Builds everything in memory - no logs need to exist yet.",
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

    if args.advanced_demos:
        generate_advanced_demo_videos(args.config, args.media_dir, args.fps, tuple(args.figsize))
        return 0

    if args.final_demos:
        generate_final_demo_videos(args.config, args.media_dir, args.fps, tuple(args.figsize))
        return 0

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
                               show_popups=not args.no_popup,
                               radar_log=args.radar_log, track_log=args.track_log,
                               vision_log=args.vision_log, lidar_log=args.lidar_log,
                               fused_log=args.fused_log)

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

    radar_data, radar_range, radar_fov = None, 15.0, 360.0
    if args.radar_log:
        radar_data = RadarData.from_csvs(data.scenario_name, args.radar_log, args.track_log)
        if os.path.exists(args.config):
            with open(args.config) as f:
                radar_config = json.load(f)
            radar_range, radar_fov = _resolve_radar_params(radar_config, data.scenario_name)

    vision_data = AuxSensorData.from_csv(args.vision_log, data.scenario_name, "vision") if args.vision_log else None
    lidar_data = AuxSensorData.from_csv(args.lidar_log, data.scenario_name, "lidar") if args.lidar_log else None
    fused_data = FusedTrackData.from_csv(args.fused_log, data.scenario_name) if args.fused_log else None

    viz = SimulationVisualizer(data, figsize=tuple(args.figsize), radar_data=radar_data,
                                radar_range=radar_range, radar_fov_deg=radar_fov,
                                vision_data=vision_data, lidar_data=lidar_data,
                                fused_data=fused_data)

    if args.mode == "interactive":
        viz.show_interactive()
    elif args.mode == "mp4":
        viz.save_animation(args.output, fps=args.fps, dpi=100)
    elif args.mode == "gif":
        viz.save_animation(args.output, fps=args.fps, dpi=80)

    return 0


if __name__ == "__main__":
    exit(main())