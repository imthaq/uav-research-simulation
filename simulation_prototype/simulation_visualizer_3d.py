"""
Simulation Visualizer 3D: a simple 3D rendering layer for the same UAV
swarm scenario logs that simulation_visualizer.py replays in 2D.

READ THIS FIRST - what "3D" means here
---------------------------------------
The underlying math simulator (Simulation / RadarLikeModel / the Kalman
tracker / fusion_model.py) works in two dimensions: every position in the
CSV logs is an (x, y) pair. There is no z-position, no vertical velocity,
no 3D sensor model, no 3D obstacle geometry, no vertical control, and no
3D metric anywhere in that pipeline.

This module does NOT change that. It is a *visualization layer*: it takes
the same 2D logs and renders them in a tilted 3D matplotlib axes (fixed
camera angle) so trajectories, formation shape, and overlays are easier to
read at a glance. Every entity is drawn at z=0 unless the log itself
proves otherwise (see `_detect_true_3d` below). Concretely, unless that
check passes:
  * UAV, obstacle/target, and track positions are plotted at z=0.
  * There is no vertical motion, no vertical avoidance, and no altitude
    separation being modeled - the "3D" is a camera angle, not physics.
The on-screen title and info panel always say so explicitly, every frame.

If a future version of the simulator starts logging real z-position for
UAVs *and* the tracked obstacle/target, `_detect_true_3d()` will pick that
up automatically and the visualizer will render real altitude and switch
its on-screen claim to "3D simulation" instead of "3D visualization of a
2D simulation". Until then, it stays honest about being the latter.

Design choice: kept deliberately simple
----------------------------------------
matplotlib's 3D text does not "billboard" to face the camera, so dense
per-point text (confidence values, track IDs, status strings) becomes
unreadable the moment the view is tilted. Rather than fight that, this
module drops almost all per-point text and leans on marker shape + color
+ one legend instead - the same lesson learned polishing the 2D overlay,
applied harder here because 3D text is worse. The camera angle is fixed
for the whole render (no spinning/orbiting), so the view never drifts.
"""

import copy
import json
import os
import shutil
import argparse
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import art3d  # noqa: F401 (needed for 3D projection)
import numpy as np

# Reuse the existing, already-tested log parsing and in-memory pipeline
# helpers instead of re-implementing CSV/row handling. simulation_visualizer.py
# must be importable (same directory / on PYTHONPATH).
from simulation_visualizer import (
    SimulationData,
    RadarData,
    AuxSensorData,
    FusedTrackData,
    _parse_contributor_uav_ids,
    _covariance_ellipse,
    _run_full_stack,
)


# ---------------------------------------------------------------------------
# 2D-simulation-in-a-3D-view honesty check
# ---------------------------------------------------------------------------

# Column names that would indicate the *simulator itself* has gone 3D.
# uav_pos_z / actual_obstacle_z are the load-bearing ones - without both,
# nothing downstream (velocity, sensors, control, metrics) can be real 3D
# either, so those two alone gate the claim. The rest are only used to
# decide whether to also draw 3D sensor measurements / covariance.
_Z_UAV = "uav_pos_z"
_Z_GOAL = "goal_pos_z"
_Z_OBSTACLE = "actual_obstacle_z"
_Z_DETECTION = "detected_z"
_Z_MEASURED = "measured_z"


def _detect_true_3d(data: "SimulationData") -> bool:
    """True only if the log rows actually carry vertical position for both
    UAVs and the tracked obstacle/target - the minimum evidence that the
    *simulator*, not just this renderer, models a vertical dimension."""
    if not data.rows:
        return False
    row = data.rows[0]
    return _Z_UAV in row and _Z_OBSTACLE in row and row[_Z_UAV] not in (None, "")


def _zval(row: Dict, field: str, is_true_3d: bool, default: float = 0.0) -> float:
    """Reads a z-coordinate field if this run is genuinely 3D and the
    field is present; otherwise returns `default` (0.0 = ground plane)."""
    if not is_true_3d:
        return default
    v = row.get(field)
    if v in (None, ""):
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# Small 3D drawing helpers
# ---------------------------------------------------------------------------

def _flat_patch(ax, patch, z: float = 0.0):
    """Embeds a normal 2D matplotlib patch (Circle, Ellipse, ...) flat in
    the 3D scene at height z. This is the one trick this whole module
    leans on: reuse matplotlib's ordinary 2D patches instead of building
    real 3D surfaces (spheres, meshes, ...) - much simpler, and the data
    is only 2D anyway, so a flat disc is an honest representation of it."""
    ax.add_patch(patch)
    art3d.pathpatch_2d_to_3d(patch, z=z, zdir="z")
    return patch


def _safe_remove(artist):
    try:
        artist.remove()
    except (ValueError, NotImplementedError, AttributeError):
        pass


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class SimulationVisualizer3D:
    """Simple 3D visualization layer over the same scenario logs
    SimulationVisualizer (2D) reads. See the module docstring for what
    "3D" does and does not mean here.

    Everything drawn each frame is rebuilt from scratch and removed
    before the next frame - no incremental artist updates - which costs a
    little performance but keeps the drawing code easy to follow, and
    that mattered more here than shaving redraw time for these clip
    lengths (tens to low hundreds of steps).
    """

    FONT_TITLE = 13
    FONT_SUBTITLE = 9
    FONT_ENTITY_ID = 9
    FONT_PANEL = 9

    # Used when a per-step safety-margin value isn't present in the log.
    DEFAULT_SAFETY_RADIUS = 5.0

    def __init__(self, data: SimulationData, fig=None, ax=None,
                 figsize: Tuple[int, int] = (10, 8),
                 radar_data: Optional[RadarData] = None,
                 vision_data: Optional[AuxSensorData] = None,
                 lidar_data: Optional[AuxSensorData] = None,
                 fused_data: Optional[FusedTrackData] = None,
                 show_covariance: bool = True,
                 show_track_history: bool = True,
                 show_safety_radius: bool = True,
                 show_mission_path: bool = True,
                 elev: float = 22.0, azim: float = -60.0,
                 title_y: float = 0.97, disclaimer_y: float = 0.925):
        # Figure-fraction y-coordinates for the per-panel title/disclaimer
        # (see setup_axis). Single-panel videos use the defaults; side-by-
        # side comparison videos (multiple 3D axes + one shared
        # fig.suptitle on top) pass lower values so there's room for the
        # suptitle above without the two colliding - see _side_by_side_video.
        self.title_y = title_y
        self.disclaimer_y = disclaimer_y
        self.data = data
        self.radar_data = radar_data
        self.vision_data = vision_data
        self.lidar_data = lidar_data
        self.fused_data = fused_data
        self.show_covariance = show_covariance
        self.show_track_history = show_track_history
        self.show_safety_radius = show_safety_radius
        self.show_mission_path = show_mission_path
        self.current_step = 0

        self.is_true_3d = _detect_true_3d(data)

        if fig is not None and ax is not None:
            self.fig, self.ax = fig, ax
        else:
            self.fig = plt.figure(figsize=figsize)
            self.ax = self.fig.add_subplot(111, projection="3d")

        self.ax.view_init(elev=elev, azim=azim)  # fixed camera, set once

        # Everything is redrawn every frame; this is the one bucket that
        # gets cleared and rebuilt each render_step() call.
        self._dynamic_artists = []
        self.info_text = None

        # Precompute obstacle/target track history once, the same way the
        # 2D visualizer precomputes radar track history: one pass over all
        # rows up front, sorted by step. Handles both a static obstacle
        # (identical position every step) and a moving one for free.
        self._obstacle_history: List[Tuple[int, float, float, float]] = []
        seen_steps = set()
        # Skip entirely for target-only scenarios with no real obstacle
        # entity (see SimulationData.from_rows in simulation_visualizer.py) -
        # otherwise this draws a "Target/Obstacle" disc at the (0.0, 0.0)
        # bookkeeping default, which sits right on top of the UAVs' start
        # position for those scenarios.
        for row in (self.data.rows if getattr(self.data, "has_obstacle", True) else []):
            step = int(row["step"])
            if step in seen_steps:
                continue
            seen_steps.add(step)
            ox = row.get("actual_obstacle_x")
            oy = row.get("actual_obstacle_y")
            if ox in (None, "") or oy in (None, ""):
                continue
            oz = _zval(row, _Z_OBSTACLE, self.is_true_3d)
            self._obstacle_history.append((step, float(ox), float(oy), oz))
        self._obstacle_history.sort(key=lambda r: r[0])

        # Radar track history, same approach as the 2D visualizer.
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

        self.setup_axis()

    # -- setup ---------------------------------------------------------

    def setup_axis(self):
        """Static scene setup: bounds, labels, title, disclaimer. Called
        once; bounds and camera angle are never changed again afterward,
        so the view stays put across the whole render/animation."""
        margin = 5
        self.ax.set_xlim(-margin, self.data.world_width + margin)
        self.ax.set_ylim(-margin, self.data.world_height + margin)
        z_top = max(self.data.world_width, self.data.world_height) * 0.3
        self.ax.set_zlim(0, max(z_top, 10))

        self.ax.set_xlabel("X (meters)", fontsize=self.FONT_SUBTITLE)
        self.ax.set_ylabel("Y (meters)", fontsize=self.FONT_SUBTITLE)
        self.ax.set_zlabel(
            "Z (real altitude)" if self.is_true_3d else "Z (unused - visualization only)",
            fontsize=self.FONT_SUBTITLE)

        # Title and disclaimer used to both be positioned relative to the
        # 3D Axes' own bounding box (ax.set_title(...) + ax.text2D at
        # transAxes y=1.03) - mplot3d's title anchor shifts with the
        # camera-relative bbox of the tilted 3D axes, so in practice the two
        # landed on almost the same pixel row and rendered interleaved/
        # illegible. Anchored to the *figure* instead, at fixed, clearly
        # separated figure-fraction y-coordinates, so their vertical
        # spacing is stable no matter the camera angle or axes bbox. Uses
        # this axes' own horizontal center (not the whole figure's) so
        # side-by-side comparison videos - multiple 3D axes sharing one
        # figure - still get one correctly-placed title per panel instead
        # of both panels' titles landing on top of each other at the
        # figure's center.
        bbox = self.ax.get_position()
        x_center = (bbox.x0 + bbox.x1) / 2
        self.fig.text(
            x_center, self.title_y, f"UAV Swarm \u2014 {self.data.scenario_name}",
            fontsize=self.FONT_TITLE, fontweight="bold", ha="center", va="top")

        # The honesty disclaimer this whole module exists to make explicit.
        disclaimer = (
            "3D simulation: vertical position, sensing, and control are "
            "modeled by the simulator."
            if self.is_true_3d else
            "3D VISUALIZATION LAYER of a 2D simulation \u2014 no vertical "
            "dynamics or 3D obstacle avoidance are modeled."
        )
        self.fig.text(
            x_center, self.disclaimer_y, disclaimer,
            fontsize=self.FONT_SUBTITLE, ha="center", va="top", color="dimgray", style="italic")

        # Ground plane grid at z=0 for a visual floor reference.
        gx = np.linspace(0, self.data.world_width, 2)
        gy = np.linspace(0, self.data.world_height, 2)
        gx_grid, gy_grid = np.meshgrid(gx, gy)
        self.ax.plot_surface(gx_grid, gy_grid, np.zeros_like(gx_grid),
                              color="lightgray", alpha=0.15, shade=False)

        self.ax.grid(True, alpha=0.2)

    def _get_colors(self) -> List:
        n = max(self.data.num_uavs, 1)
        return [plt.cm.hsv(i / n) for i in range(self.data.num_uavs)]

    def _mission_status(self, step: int) -> str:
        is_last_step = step >= self.data.steps - 1 and not self.data.is_live
        if self.data.mission_success and step >= self.data.mission_success_step:
            return "SUCCESS"
        if is_last_step and not self.data.mission_success:
            return "FAILURE"
        return "In Progress"

    # -- per-frame summaries --------------------------------------------

    def _comm_status(self, step: int) -> str:
        """Short communication-link summary for the info panel: which
        fusion architecture is active and, for distributed, how many
        inter-UAV broadcasts actually arrived this step."""
        if self.fused_data is None:
            return "n/a (no fusion log)"
        rows = self.fused_data.rows_by_step.get(step, [])
        if not rows:
            return "n/a"
        archs = {r.get("architecture", "centralized") for r in rows}
        if "distributed" not in archs:
            return "centralized (no inter-UAV link needed)"
        attempted = next((r.get("comm_messages") for r in rows
                           if r.get("comm_messages") not in (None, "")), None)
        delivered = next((r.get("comm_messages_delivered") for r in rows
                           if r.get("comm_messages_delivered") not in (None, "")), None)
        if attempted not in (None, "") and delivered not in (None, ""):
            return f"distributed, {delivered}/{attempted} links delivered"
        return "distributed"

    def _trust_state(self, step: int) -> str:
        """Short trust-state summary: whether fusion is using fixed or
        adaptive/trust-weighted weighting (from the fusion_mode name) plus
        the average sensor reliability reported this step, if any sensor
        log reports one. Returns 'n/a' rather than guessing when neither
        signal is available."""
        step_rows = self.data.get_step_data(step)
        fusion_mode = step_rows[0].get("fusion_mode", "") if step_rows else ""
        mode_s = None
        if "trust" in fusion_mode.lower() or "adaptive" in fusion_mode.lower():
            mode_s = "adaptive/trust-weighted"
        elif fusion_mode:
            mode_s = "fixed"

        reliabilities = []
        for sensor in (self.vision_data, self.lidar_data):
            if sensor is None:
                continue
            for uav_id in range(self.data.num_uavs):
                for d in sensor.detections.get((step, uav_id), []):
                    rel = d.get("sensor_reliability")
                    if rel not in (None, ""):
                        try:
                            reliabilities.append(float(rel))
                        except (TypeError, ValueError):
                            pass

        parts = []
        if mode_s:
            parts.append(mode_s)
        if reliabilities:
            parts.append(f"avg reliability {sum(reliabilities) / len(reliabilities):.2f}")
        return ", ".join(parts) if parts else "n/a"

    # -- rendering --------------------------------------------------------

    def render_step(self, step: int):
        """Render one simulation step. Every artist drawn here (except
        the static scene from setup_axis) is tracked in
        self._dynamic_artists and wiped at the start of the next call."""
        self.current_step = step
        step_data = self.data.get_step_data(step)
        if not step_data:
            return

        for artist in self._dynamic_artists:
            _safe_remove(artist)
        self._dynamic_artists.clear()

        colors = self._get_colors()
        uav_positions: Dict[int, Tuple[float, float, float]] = {}

        # ---- obstacle / target (ground truth) ----
        obst_upto = [r for r in self._obstacle_history if r[0] <= step]
        if obst_upto:
            _, ox, oy, oz = obst_upto[-1]
            disc = patches.Circle((ox, oy), self.data.obstacle_radius,
                                   facecolor="red", edgecolor="darkred", alpha=0.55)
            self._dynamic_artists.append(_flat_patch(self.ax, disc, z=oz))
            if len(obst_upto) > 1:
                hxs = [r[1] for r in obst_upto]
                hys = [r[2] for r in obst_upto]
                hzs = [r[3] for r in obst_upto]
                (trail,) = self.ax.plot(hxs, hys, hzs, "--", color="darkred", alpha=0.35, linewidth=1)
                self._dynamic_artists.append(trail)
            label = self.ax.text(ox, oy, oz + 1.0, "Target/Obstacle",
                                  fontsize=self.FONT_ENTITY_ID, color="darkred", ha="center")
            self._dynamic_artists.append(label)

        # ---- per-UAV entities ----
        for row in step_data:
            uav_id = int(row["uav_id"])
            color = colors[uav_id % len(colors)]
            x, y = float(row["uav_pos_x"]), float(row["uav_pos_y"])
            z = _zval(row, _Z_UAV, self.is_true_3d)
            gx, gy = float(row["goal_pos_x"]), float(row["goal_pos_y"])
            gz = _zval(row, _Z_GOAL, self.is_true_3d)
            uav_positions[uav_id] = (x, y, z)

            (dot,) = self.ax.plot([x], [y], [z], "o", markersize=9, color=color)
            self._dynamic_artists.append(dot)
            label = self.ax.text(x, y, z + 1.2, f"U{uav_id}", fontsize=self.FONT_ENTITY_ID,
                                  color=color, ha="center", fontweight="bold")
            self._dynamic_artists.append(label)

            # Trajectory trail up to this step.
            trail_xy = self.data.uav_trajectories[uav_id][: step + 1]
            if len(trail_xy) > 1:
                txs, tys = zip(*trail_xy)
                tzs = [z] * len(txs)
                (trail,) = self.ax.plot(txs, tys, tzs, "-", color=color, alpha=0.4, linewidth=1.2)
                self._dynamic_artists.append(trail)

            # Goal marker + mission path (dashed line to goal).
            (goal,) = self.ax.plot([gx], [gy], [gz], "s", markersize=7, color=color,
                                    markerfacecolor="none", alpha=0.6)
            self._dynamic_artists.append(goal)
            if self.show_mission_path:
                (path,) = self.ax.plot([x, gx], [y, gy], [z, gz], ":", color=color, alpha=0.35, linewidth=1)
                self._dynamic_artists.append(path)

            # Safety radius: flat ring at UAV altitude.
            if self.show_safety_radius:
                radius = self.DEFAULT_SAFETY_RADIUS
                sm_val = row.get("safety_margin_applied")
                if sm_val not in (None, ""):
                    try:
                        radius = float(sm_val)
                    except (TypeError, ValueError):
                        pass
                ring = patches.Circle((x, y), radius, edgecolor=color, facecolor="none",
                                       linestyle=":", alpha=0.35, linewidth=1)
                self._dynamic_artists.append(_flat_patch(self.ax, ring, z=z))

            if self.radar_data is not None:
                self._draw_radar_overlay(step, uav_id, color)
            if self.vision_data is not None:
                self._draw_aux_overlay(self.vision_data, "*", step, uav_id, "teal")
            if self.lidar_data is not None:
                self._draw_aux_overlay(self.lidar_data, "P", step, uav_id, "saddlebrown")

        # ---- swarm formation: spokes from centroid ----
        if len(uav_positions) > 1:
            cx = sum(p[0] for p in uav_positions.values()) / len(uav_positions)
            cy = sum(p[1] for p in uav_positions.values()) / len(uav_positions)
            cz = sum(p[2] for p in uav_positions.values()) / len(uav_positions)
            (centroid_pt,) = self.ax.plot([cx], [cy], [cz], "+", markersize=8,
                                           color="black", alpha=0.5)
            self._dynamic_artists.append(centroid_pt)
            for (x, y, z) in uav_positions.values():
                (spoke,) = self.ax.plot([cx, x], [cy, y], [cz, z], "-",
                                         color="gray", alpha=0.15, linewidth=0.8)
                self._dynamic_artists.append(spoke)

        # ---- fused estimates + comm links ----
        if self.fused_data is not None:
            self._draw_fusion_overlay(step, colors, uav_positions)

        self._update_info_panel(step, step_data)
        self.fig.canvas.draw_idle()

    def _draw_radar_overlay(self, step: int, uav_id: int, color):
        """Radar detections (real + false alarm), predicted vs. filtered
        tracks, and covariance - all flat at z=0 (or the track's reported
        z, if this run is genuinely 3D)."""
        for d in self.radar_data.detections.get((step, uav_id), []):
            status = d.get("detection_status")
            dx, dy = d.get("detected_x"), d.get("detected_y")
            if dx in (None, "") or dy in (None, ""):
                continue
            dx, dy = float(dx), float(dy)
            dz = _zval(d, _Z_DETECTION, self.is_true_3d)
            if status == "detected":
                (pt,) = self.ax.plot([dx], [dy], [dz], "x", color=color, markersize=6, markeredgewidth=1.5)
                self._dynamic_artists.append(pt)
            elif status == "false_alarm":
                (pt,) = self.ax.plot([dx], [dy], [dz], "^", color="magenta", markersize=6, alpha=0.85)
                self._dynamic_artists.append(pt)

        track_colors = {"tentative": "gray", "confirmed": color, "coasting": "slateblue",
                         "lost": "red", "deleted": "black"}
        for t in self.radar_data.tracks.get((step, uav_id), []):
            tx, ty = float(t["est_x"]), float(t["est_y"])
            tz = _zval(t, "est_z", self.is_true_3d)
            status = t.get("status", "tentative")
            tcolor = track_colors.get(status, "gray")
            is_predicted = status == "coasting"  # coasting = predicted, not detection-updated
            marker_shape = "d" if is_predicted else "D"
            facecolor = "none" if is_predicted else tcolor
            (pt,) = self.ax.plot([tx], [ty], [tz], marker_shape, color=tcolor,
                                  markerfacecolor=facecolor, markersize=7,
                                  alpha=0.5 if status in ("lost", "deleted") else 0.9)
            self._dynamic_artists.append(pt)

            if self.show_covariance:
                cov_json = t.get("covariance")
                if cov_json:
                    try:
                        cov4 = json.loads(cov_json)
                        cov2 = [[cov4[0][0], cov4[0][1]], [cov4[1][0], cov4[1][1]]]
                        ell = _covariance_ellipse(tx, ty, cov2, n_std=2.0, edgecolor=tcolor,
                                                   facecolor=tcolor, linestyle="-", linewidth=0.8, alpha=0.12)
                        if ell is not None:
                            self._dynamic_artists.append(_flat_patch(self.ax, ell, z=tz))
                    except (ValueError, TypeError, IndexError, json.JSONDecodeError):
                        pass

            if self.show_track_history:
                hist = self._track_history.get(t.get("track_id"), [])
                hist_upto = [(hx, hy) for (hs, hx, hy) in hist if hs <= step]
                if len(hist_upto) > 1:
                    hxs, hys = zip(*hist_upto)
                    hzs = [tz] * len(hxs)
                    (trail,) = self.ax.plot(hxs, hys, hzs, "-", color=tcolor, alpha=0.3, linewidth=1.0)
                    self._dynamic_artists.append(trail)

    def _draw_aux_overlay(self, sensor_data: AuxSensorData, marker_char: str,
                           step: int, uav_id: int, color: str):
        """Vision/LiDAR measured detections - marker only, no text (see
        module docstring on why 3D text labels are avoided)."""
        for d in sensor_data.detections.get((step, uav_id), []):
            mx, my = d.get("measured_x"), d.get("measured_y")
            if mx in (None, "") or my in (None, ""):
                continue
            mx, my = float(mx), float(my)
            mz = _zval(d, _Z_MEASURED, self.is_true_3d)
            is_stale = d.get("is_stale") in ("True", True)
            (pt,) = self.ax.plot([mx], [my], [mz], marker_char, color=color, markersize=7,
                                  alpha=0.3 if is_stale else 0.85)
            self._dynamic_artists.append(pt)

    def _draw_fusion_overlay(self, step: int, colors, uav_positions: Dict[int, Tuple[float, float, float]]):
        """Fused estimates (star marker) and, for the distributed
        architecture, inter-UAV communication links colored by whether
        each link's broadcast was actually delivered this step."""
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
            fz = _zval(row, "fused_z", self.is_true_3d)
            is_stale = row.get("is_stale") in ("True", True)

            if is_centralized:
                mcolor, anchor_uav = "gold", None
            else:
                anchor_uav = int(local_uav)
                mcolor = colors[anchor_uav % len(colors)]

            (pt,) = self.ax.plot([fx], [fy], [fz], "*", color=mcolor,
                                  markersize=14 if is_centralized else 11,
                                  alpha=0.4 if is_stale else 0.95,
                                  markeredgecolor="black", markeredgewidth=0.5)
            self._dynamic_artists.append(pt)

            if self.show_covariance:
                pos_var = row.get("position_variance")
                if pos_var not in (None, ""):
                    try:
                        var = float(pos_var)
                        if var > 0:
                            side = 2 * 2.0 * (var ** 0.5)
                            ell = patches.Ellipse((fx, fy), width=side, height=side, angle=0,
                                                   edgecolor=mcolor, facecolor="none",
                                                   linestyle="--", linewidth=1.0, alpha=0.5)
                            self._dynamic_artists.append(_flat_patch(self.ax, ell, z=fz))
                    except (TypeError, ValueError):
                        pass

            if arch == "distributed" and anchor_uav is not None and anchor_uav in uav_positions:
                contributors = _parse_contributor_uav_ids(row.get("source_track_ids"))
                contributors.discard(anchor_uav)
                ax_pos = uav_positions[anchor_uav]
                for other in all_uav_ids:
                    if other == anchor_uav or other not in uav_positions:
                        continue
                    ox2, oy2, oz2 = uav_positions[other]
                    delivered = other in contributors
                    (line,) = self.ax.plot(
                        [ax_pos[0], ox2], [ax_pos[1], oy2], [ax_pos[2], oz2],
                        color=(colors[other % len(colors)] if delivered else "red"),
                        linestyle="-" if delivered else ":",
                        alpha=0.35 if delivered else 0.55,
                        linewidth=1.2 if delivered else 1.0)
                    self._dynamic_artists.append(line)

    def _update_info_panel(self, step: int, step_data: List[Dict]):
        if self.info_text is None:
            self.info_text = self.ax.text2D(
                0.02, 0.98, "", transform=self.ax.transAxes, verticalalignment="top",
                fontsize=self.FONT_PANEL, family="monospace",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6))

        info_row = step_data[0]
        time_s = float(info_row.get("time_s", 0))
        mission_status = self._mission_status(step)

        lines = [
            f"Scenario: {self.data.scenario_name}",
            f"Step {step}  |  t = {time_s:.1f}s",
            f"Fusion mode: {info_row.get('fusion_mode', 'no_fusion')}",
            f"Comm status: {self._comm_status(step)}",
            f"Trust state: {self._trust_state(step)}",
            f"Mission: {mission_status}",
        ]
        self.info_text.set_text("\n".join(lines))
        if mission_status == "SUCCESS":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightgreen", alpha=0.7))
        elif mission_status == "FAILURE":
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="lightcoral", alpha=0.7))
        else:
            self.info_text.set_bbox(dict(boxstyle="round", facecolor="wheat", alpha=0.6))

    # -- legend, export, interaction ------------------------------------

    def add_legend(self):
        """One simple legend, built once, listing only what's actually
        drawn for this run (radar/vision/LiDAR/fusion entries only appear
        if that log was supplied)."""
        elements = [
            patches.Patch(facecolor="red", alpha=0.55, label="Target / Obstacle"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=8, label="UAV"),
            Line2D([0], [0], marker="s", color="w", markerfacecolor="none", markeredgecolor="gray", markersize=8, label="Goal"),
            Line2D([0], [0], linestyle=":", color="gray", label="Mission Path"),
            Line2D([0], [0], linestyle=":", color="gray", alpha=0.6, label="Safety Radius"),
        ]
        if self.radar_data is not None:
            elements += [
                Line2D([0], [0], marker="x", color="w", markeredgecolor="gray", markersize=8, label="Radar Detection"),
                Line2D([0], [0], marker="^", color="w", markerfacecolor="magenta", markersize=8, label="False Alarm"),
                Line2D([0], [0], marker="d", color="w", markerfacecolor="none", markeredgecolor="slateblue", markersize=8, label="Predicted Track"),
                Line2D([0], [0], marker="D", color="w", markerfacecolor="gray", markersize=8, label="Filtered Track"),
                patches.Patch(facecolor="gray", alpha=0.15, label="Track Covariance (2\u03c3)"),
            ]
        if self.vision_data is not None:
            elements.append(Line2D([0], [0], marker="*", color="w", markerfacecolor="teal", markersize=9, label="Vision Detection"))
        if self.lidar_data is not None:
            elements.append(Line2D([0], [0], marker="P", color="w", markerfacecolor="saddlebrown", markersize=9, label="LiDAR Detection"))
        if self.fused_data is not None:
            elements += [
                Line2D([0], [0], marker="*", color="w", markerfacecolor="gold", markersize=11, label="Fused Estimate (centralized)"),
                Line2D([0], [0], marker="*", color="w", markerfacecolor="gray", markersize=11, label="Fused Estimate (distributed)"),
                Line2D([0], [0], color="gray", alpha=0.4, label="Comm Link (delivered)"),
                Line2D([0], [0], color="red", linestyle=":", label="Comm Link (dropped)"),
            ]
        self.ax.legend(handles=elements, loc="upper left", fontsize=8, bbox_to_anchor=(1.02, 1.0))

    def save_animation(self, output_path: str, fps: int = 5, dpi: int = 100):
        self.add_legend()
        self.fig.tight_layout()

        def animate(frame):
            self.render_step(frame)
            return []

        anim = animation.FuncAnimation(self.fig, animate, frames=self.data.steps,
                                        interval=1000 // fps, repeat=True)
        ext = Path(output_path).suffix.lower()
        if ext == ".mp4":
            writer = animation.FFMpegWriter(fps=fps)
        elif ext == ".gif":
            writer = animation.PillowWriter(fps=fps)
        else:
            raise ValueError(f"Unsupported format: {ext}. Use .mp4 or .gif")
        anim.save(output_path, writer=writer, dpi=dpi)
        print(f"Saved 3D animation to {output_path}")

    def show_interactive(self):
        """Static camera by default; drag with the mouse to rotate freely,
        arrow keys to step through frames, 'q' to quit."""
        print("Interactive 3D mode - drag to rotate, arrow keys to step, 'q' to quit")

        def on_key(event):
            if event.key == "left":
                self.current_step = max(0, self.current_step - 1)
                self.render_step(self.current_step)
            elif event.key == "right":
                self.current_step = min(self.data.steps - 1, self.current_step + 1)
                self.render_step(self.current_step)
            elif event.key == "q":
                plt.close(self.fig)

        self.fig.canvas.mpl_connect("key_press_event", on_key)
        self.render_step(0)
        self.add_legend()
        self.fig.tight_layout()
        plt.show()


# ---------------------------------------------------------------------------
# Demo video generation: baseline, heavy clutter, target crossing, radar
# dropout, communication outage, faulty sensor, centralized-vs-distributed
# fusion, fixed-vs-dynamic trust.
# ---------------------------------------------------------------------------

# name -> (scenario key in simulation_config.json, architecture, title).
# Scenario keys mirror the ones simulation_visualizer.py's own demo suite
# already uses (see ADVANCED_DEMOS / FINAL_DEMOS there), so a config that
# works with the 2D demos works with these too.
SINGLE_PANEL_DEMOS_3D = [
    dict(name="baseline_3d", scenario="baseline",
         architecture="centralized", title="Baseline"),
    dict(name="heavy_clutter_3d", scenario="high_clutter",
         architecture="centralized", title="Heavy Clutter"),
    dict(name="target_crossing_3d", scenario="target_crossing",
         architecture="centralized", title="Target Crossing"),
    dict(name="radar_dropout_3d", scenario="target_reappearing_after_dropout",
         architecture="centralized", title="Radar Dropout / Recovery"),
    dict(name="communication_outage_3d", scenario="communication_outage",
         architecture="distributed", title="Communication Outage"),
    dict(name="faulty_sensor_3d", scenario="overconfident_faulty_sensor",
         architecture="centralized", title="Faulty Sensor"),
]

# Scenario used for the two side-by-side comparison videos.
FUSION_COMPARISON_SCENARIO = "communication_outage"
TRUST_COMPARISON_SCENARIO = "faulty_sensor_trust_weighted_fusion_dynamic"


def _panel_video(config: dict, scenario: str, architecture: str, title: str,
                  media_dir: str, name: str, fps: int, figsize: Tuple[int, int],
                  seed: Optional[int], use_adaptive_trust: bool = True) -> bool:
    """Runs one scenario in memory and saves it as a single-panel 3D video."""
    try:
        sim_data, radar_data, fused_data, radar_model = _run_full_stack(
            config, scenario, architecture=architecture, seed=seed,
            use_adaptive_trust=use_adaptive_trust)
    except Exception as e:
        print(f"  FAILED to build data for {name}: {e}")
        return False

    sim_data.scenario_name = title
    viz = SimulationVisualizer3D(sim_data, figsize=figsize, radar_data=radar_data,
                                  fused_data=fused_data)
    out_path = os.path.join(media_dir, f"{name}.mp4")
    try:
        viz.save_animation(out_path, fps=fps)
        return True
    except Exception as e:
        print(f"  video save failed for {name}: {e}")
        return False
    finally:
        plt.close(viz.fig)


def _side_by_side_video(config: dict, scenario: str, name: str, title: str,
                         panel_specs: List[dict], media_dir: str, fps: int,
                         figsize: Tuple[int, int], seed: Optional[int]) -> bool:
    """Runs `scenario` once per entry in panel_specs (each overriding
    architecture and/or use_adaptive_trust) and renders them as side-by-side
    3D panels sharing one animation - same pattern as the 2D visualizer's
    centralized-vs-distributed comparison video."""
    try:
        panels = []
        for spec in panel_specs:
            sim_data, radar_data, fused_data, radar_model = _run_full_stack(
                config, scenario, architecture=spec.get("architecture", "centralized"),
                seed=seed, use_adaptive_trust=spec.get("use_adaptive_trust", True))
            sim_data.scenario_name = spec["label"]
            panels.append((sim_data, radar_data, fused_data))

        fig = plt.figure(figsize=(figsize[0] * len(panels), figsize[1]))
        visualizers = []
        for i, (sim_data, radar_data, fused_data) in enumerate(panels):
            ax = fig.add_subplot(1, len(panels), i + 1, projection="3d")
            # Lower than the single-panel default (0.97/0.925): this figure
            # also carries one shared fig.suptitle across the top (set
            # below), so each panel's own title/disclaimer need to sit
            # further down to leave it clear room instead of stacking
            # directly on top of it.
            viz = SimulationVisualizer3D(sim_data, fig=fig, ax=ax, radar_data=radar_data,
                                          fused_data=fused_data,
                                          title_y=0.89, disclaimer_y=0.84)
            visualizers.append(viz)
        max_steps = max(v.data.steps for v in visualizers)

        def animate(frame):
            for v in visualizers:
                v.render_step(min(frame, v.data.steps - 1))
            return []

        anim = animation.FuncAnimation(fig, animate, frames=max_steps,
                                        interval=1000 // fps, repeat=True)
        fig.suptitle(title, fontsize=14, fontweight="bold", y=0.99)
        out_path = os.path.join(media_dir, f"{name}.mp4")
        anim.save(out_path, writer=animation.FFMpegWriter(fps=fps), dpi=100)
        plt.close(fig)
        print(f"  saved {out_path}")
        return True
    except Exception as e:
        print(f"  FAILED {name}: {e}")
        return False


def generate_demo_videos_3d(config_path: str = "simulation_prototype/simulation_config.json",
                             media_dir: str = "media", fps: int = 5,
                             figsize: Tuple[int, int] = (7, 6),
                             seed: Optional[int] = None) -> Dict[str, bool]:
    """Generates the full 3D demo suite: 6 single-panel videos (baseline,
    heavy clutter, target crossing, radar dropout, communication outage,
    faulty sensor) plus 2 side-by-side comparison videos (centralized vs.
    distributed fusion, fixed vs. dynamic trust). Everything is built
    in-memory via _run_full_stack - no pre-existing CSV logs required."""
    with open(config_path) as f:
        config = json.load(f)
    os.makedirs(media_dir, exist_ok=True)
    scenario_names = set(config["scenarios"].keys())

    if shutil.which("ffmpeg") is None:
        print("NOTE: ffmpeg not found on PATH - 3D demo videos will be skipped.")
        return {}

    results: Dict[str, bool] = {}

    for demo in SINGLE_PANEL_DEMOS_3D:
        name, scenario = demo["name"], demo["scenario"]
        if scenario not in scenario_names:
            print(f"SKIP {name}: scenario '{scenario}' not found in {config_path}")
            results[name] = False
            continue
        print(f"{name}: running '{scenario}' [{demo['architecture']}] ...")
        results[name] = _panel_video(
            config, scenario, demo["architecture"], demo["title"],
            media_dir, name, fps, figsize, seed)

    print("centralized_vs_distributed_fusion_3d: building both architectures ...")
    fusion_scn = FUSION_COMPARISON_SCENARIO if FUSION_COMPARISON_SCENARIO in scenario_names else next(iter(scenario_names))
    results["centralized_vs_distributed_fusion_3d"] = _side_by_side_video(
        config, fusion_scn, "centralized_vs_distributed_fusion_3d",
        "Centralized vs Distributed Fusion",
        [dict(architecture="centralized", label=f"{fusion_scn} [centralized]"),
         dict(architecture="distributed", label=f"{fusion_scn} [distributed]")],
        media_dir, fps, figsize, seed)

    print("fixed_vs_dynamic_trust_3d: building both trust modes ...")
    trust_scn = TRUST_COMPARISON_SCENARIO if TRUST_COMPARISON_SCENARIO in scenario_names else next(iter(scenario_names))
    results["fixed_vs_dynamic_trust_3d"] = _side_by_side_video(
        config, trust_scn, "fixed_vs_dynamic_trust_3d",
        "Fixed vs Dynamic Trust Weighting",
        [dict(architecture="centralized", use_adaptive_trust=False, label=f"{trust_scn} [fixed trust]"),
         dict(architecture="centralized", use_adaptive_trust=True, label=f"{trust_scn} [dynamic trust]")],
        media_dir, fps, figsize, seed)

    success = sum(1 for v in results.values() if v)
    print(f"\n3D demo videos: {success}/{len(results)} saved to {media_dir}/")
    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="3D visualization layer for UAV swarm simulation logs "
        "(see the module docstring: the simulator itself is 2D unless "
        "z-position columns are present in the log).")
    parser.add_argument("--log", default=None, help="Path to a simulation CSV log to replay.")
    parser.add_argument("--radar-log", default=None, help="Combined radar detection CSV (optional).")
    parser.add_argument("--track-log", default=None, help="Combined radar track CSV (optional).")
    parser.add_argument("--vision-log", default=None, help="Combined vision detection CSV (optional).")
    parser.add_argument("--lidar-log", default=None, help="Combined LiDAR detection CSV (optional).")
    parser.add_argument("--fused-log", default=None, help="Combined fused-track CSV (optional).")
    parser.add_argument("--mode", choices=["interactive", "mp4", "gif"], default="interactive")
    parser.add_argument("--output", default=None, help="Output path for --mode mp4/gif.")
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--figsize", type=int, nargs=2, default=[10, 8])
    parser.add_argument("--elev", type=float, default=22.0, help="Fixed camera elevation angle.")
    parser.add_argument("--azim", type=float, default=-60.0, help="Fixed camera azimuth angle.")
    parser.add_argument("--config", default="simulation_prototype/simulation_config.json")
    parser.add_argument("--media-dir", default="media")
    parser.add_argument("--demo-videos", action="store_true",
                         help="Generate the 8-video 3D demo suite (baseline, heavy "
                         "clutter, target crossing, radar dropout, communication "
                         "outage, faulty sensor, centralized-vs-distributed fusion, "
                         "fixed-vs-dynamic trust) into --media-dir and exit.")
    args = parser.parse_args()

    if args.demo_videos:
        generate_demo_videos_3d(args.config, args.media_dir, args.fps, tuple(args.figsize))
        return 0

    if not args.log:
        print("Error: --log is required (or use --demo-videos).")
        return 1
    if not os.path.exists(args.log):
        print(f"Error: log file not found: {args.log}")
        return 1
    if args.mode in ("mp4", "gif") and not args.output:
        print(f"Error: --output required for {args.mode} mode")
        return 1

    print(f"Loading {args.log}...")
    data = SimulationData.from_csv(args.log)
    print(f"Loaded {data.num_uavs} UAVs, {data.steps} steps, scenario: {data.scenario_name}")

    radar_data = RadarData.from_csvs(data.scenario_name, args.radar_log, args.track_log) if args.radar_log else None
    vision_data = AuxSensorData.from_csv(args.vision_log, data.scenario_name, "vision") if args.vision_log else None
    lidar_data = AuxSensorData.from_csv(args.lidar_log, data.scenario_name, "lidar") if args.lidar_log else None
    fused_data = FusedTrackData.from_csv(args.fused_log, data.scenario_name) if args.fused_log else None

    viz = SimulationVisualizer3D(
        data, figsize=tuple(args.figsize), radar_data=radar_data, vision_data=vision_data,
        lidar_data=lidar_data, fused_data=fused_data, elev=args.elev, azim=args.azim)

    if not viz.is_true_3d:
        print("NOTE: this log has no z-position data - rendering as a 3D "
              "visualization layer over the 2D simulation (see module docstring).")

    if args.mode == "interactive":
        viz.show_interactive()
    else:
        viz.save_animation(args.output, fps=args.fps, dpi=100 if args.mode == "mp4" else 80)

    return 0


if __name__ == "__main__":
    exit(main())