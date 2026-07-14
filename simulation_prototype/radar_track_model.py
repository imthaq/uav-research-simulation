"""
radar_track_model.py

Converts the raw, per-step radar detections produced by radar_like_model.py
into simple radar tracks over time.

Each radar (one per UAV, since every UAV only sees its own radar's
detections) maintains its own independent set of tracks.

Track fields
------------
track_id       - unique id for this track (scoped to one radar/UAV)
est_x, est_y   - smoothed position estimate
est_vx, est_vy - smoothed velocity estimate; None until the track has been
                 matched to a detection at least twice (a single detection
                 gives a position but not a velocity yet)
confidence     - confidence carried over from the most recent matched
                 detection (held steady while the track is coasting on a
                 miss)
age            - number of steps since the track was created
missed_count   - consecutive steps with no matching detection (resets to 0
                 on every match)
status         - "tentative"  - just created, not yet matched enough times
                                 in a row to be trusted
                 "confirmed"  - matched CONFIRM_HITS times in a row
                 "lost"       - missed MAX_MISSED times in a row; this is
                                 the track's final row, after which it is
                                 dropped

How tracks are formed (nearest-neighbor association)
-----------------------------------------------------
At each time step, for every radar:
  1. Existing tracks are matched to this step's detections by nearest
     neighbor: candidate (track, detection) pairs within GATE_DISTANCE are
     considered in order of increasing distance, and each track/detection
     is used at most once (best matches get first pick).
  2. Matched tracks fold in the new detection (position/velocity smoothed
     with simple exponential filters) and their missed_count resets to 0.
  3. Unmatched tracks are coasted forward using their last known velocity
     and missed_count goes up; too many misses in a row -> status "lost".
  4. Unmatched detections spawn new tentative tracks.

This is intentionally a simple nearest-neighbor + exponential-smoothing
tracker (not a Kalman filter) to keep the model easy to follow.
"""

import argparse
import csv
import json
import math

from radar_like_model import RadarLikeModel

TENTATIVE = "tentative"
CONFIRMED = "confirmed"
LOST = "lost"

# How close a detection has to be to an existing track's (coasted)
# position to be considered a match for it.
GATE_DISTANCE = 4.0

# Consecutive matches needed before a tentative track becomes confirmed.
CONFIRM_HITS = 3

# Consecutive misses allowed before a track is dropped as lost.
MAX_MISSED = 3

# Exponential smoothing weights (0-1): higher = trust the new detection
# more, lower = trust the track's existing estimate more.
POSITION_ALPHA = 0.6
VELOCITY_ALPHA = 0.5


class RadarTrack:
    """One tracked object as seen by a single radar (UAV)."""

    def __init__(self, radar_id, track_num, x, y, confidence, step):
        self.track_id = f"r{radar_id}_t{track_num}"
        self.radar_id = radar_id
        self.x = x
        self.y = y
        self.vx = None
        self.vy = None
        self.confidence = confidence
        self.age = 1
        self.hit_streak = 1
        self.missed_count = 0
        self.status = TENTATIVE

    def predicted_position(self, dt):
        """Where the track would be this step if nothing updates it,
        coasting on its last known velocity (0 if none established yet)."""
        vx = self.vx or 0.0
        vy = self.vy or 0.0
        return self.x + vx * dt, self.y + vy * dt

    def apply_match(self, det_x, det_y, confidence, dt):
        """Fold in a matched detection: smooth position and velocity with
        simple alpha filters."""
        raw_vx = (det_x - self.x) / dt if dt > 0 else 0.0
        raw_vy = (det_y - self.y) / dt if dt > 0 else 0.0

        self.x = POSITION_ALPHA * det_x + (1 - POSITION_ALPHA) * self.x
        self.y = POSITION_ALPHA * det_y + (1 - POSITION_ALPHA) * self.y

        if self.vx is None:
            # First re-observation of this track - take the raw velocity
            # outright, there's nothing to smooth it against yet.
            self.vx, self.vy = raw_vx, raw_vy
        else:
            self.vx = VELOCITY_ALPHA * raw_vx + (1 - VELOCITY_ALPHA) * self.vx
            self.vy = VELOCITY_ALPHA * raw_vy + (1 - VELOCITY_ALPHA) * self.vy

        self.confidence = confidence
        self.missed_count = 0
        self.hit_streak += 1
        self.age += 1
        if self.status == TENTATIVE and self.hit_streak >= CONFIRM_HITS:
            self.status = CONFIRMED

    def apply_miss(self, dt):
        """No detection matched this track this step: coast it forward and
        count the miss."""
        self.x, self.y = self.predicted_position(dt)
        self.missed_count += 1
        self.hit_streak = 0
        self.age += 1
        if self.missed_count >= MAX_MISSED:
            self.status = LOST

    def as_row(self, scenario, step):
        return {
            "scenario": scenario,
            "time_step": step,
            "radar_id": self.radar_id,
            "track_id": self.track_id,
            "est_x": round(self.x, 4),
            "est_y": round(self.y, 4),
            "est_vx": round(self.vx, 4) if self.vx is not None else None,
            "est_vy": round(self.vy, 4) if self.vy is not None else None,
            "confidence": self.confidence,
            "age": self.age,
            "missed_count": self.missed_count,
            "status": self.status,
        }


class RadarTracker:
    """Runs nearest-neighbor tracking for a single radar (one UAV) across
    a sequence of steps."""

    def __init__(self, radar_id):
        self.radar_id = radar_id
        self.tracks = []
        self._next_track_num = 1

    def _match(self, detections, dt):
        """Greedy nearest-neighbor association: build every (track,
        detection) pair within GATE_DISTANCE, then claim pairs in order of
        increasing distance so the closest matches win first."""
        candidates = []
        for ti, track in enumerate(self.tracks):
            pred_x, pred_y = track.predicted_position(dt)
            for di, det in enumerate(detections):
                d = math.hypot(det["x"] - pred_x, det["y"] - pred_y)
                if d <= GATE_DISTANCE:
                    candidates.append((d, ti, di))
        candidates.sort(key=lambda c: c[0])

        matched_track = {}
        matched_det = {}
        for d, ti, di in candidates:
            if ti in matched_track or di in matched_det:
                continue
            matched_track[ti] = di
            matched_det[di] = ti
        return matched_track

    def update(self, step, detections, dt):
        """Process one step's detections (list of dicts with at least
        x, y, confidence) for this radar. Returns the rows to log for this
        step: one per active track (including any that just went lost)."""
        matched_track = self._match(detections, dt)

        rows = []
        still_active = []
        for ti, track in enumerate(self.tracks):
            if ti in matched_track:
                det = detections[matched_track[ti]]
                track.apply_match(det["x"], det["y"], det.get("confidence"), dt)
            else:
                track.apply_miss(dt)

            rows.append(track.as_row(None, step))
            if track.status != LOST:
                still_active.append(track)
        self.tracks = still_active

        matched_det_indices = set(matched_track.values())
        for di, det in enumerate(detections):
            if di in matched_det_indices:
                continue
            new_track = RadarTrack(
                self.radar_id, self._next_track_num, det["x"], det["y"],
                det.get("confidence"), step)
            self._next_track_num += 1
            self.tracks.append(new_track)
            rows.append(new_track.as_row(None, step))

        return rows


def build_tracks(scenario_name, detection_rows, dt):
    """Runs one RadarTracker per radar_id over detection_rows (as produced
    by radar_like_model.RadarLikeModel.run(), already limited to one
    scenario) and returns the full list of track log rows, in step order.

    Only rows with an actual x/y this step (status "detected" or
    "false_alarm") are fed to the tracker - "missed"/"dropout" rows have no
    position to associate, so a real radar simply wouldn't see anything to
    report that step."""
    by_radar_step = {}
    for row in detection_rows:
        if row["detected_x"] is None or row["detected_y"] is None:
            continue
        key = (row["radar_id"], row["time_step"])
        by_radar_step.setdefault(key, []).append({
            "x": row["detected_x"],
            "y": row["detected_y"],
            "confidence": row["confidence_score"],
        })

    radar_ids = sorted({r for r, _ in by_radar_step})
    max_step = max((s for _, s in by_radar_step), default=-1)

    trackers = {radar_id: RadarTracker(radar_id) for radar_id in radar_ids}

    all_rows = []
    for step in range(max_step + 1):
        for radar_id in radar_ids:
            detections = by_radar_step.get((radar_id, step), [])
            rows = trackers[radar_id].update(step, detections, dt)
            for row in rows:
                row["scenario"] = scenario_name
                all_rows.append(row)

    return all_rows


def main():
    parser = argparse.ArgumentParser(
        description="Turn raw radar detections into simple radar tracks")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default="logs/radar_track_log.csv")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    dt = config["sim"]["dt"]
    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        model = RadarLikeModel(config, name)
        detection_rows = model.run()
        track_rows = build_tracks(name, detection_rows, dt)
        all_rows.extend(track_rows)
        print(f"{name}: {len(detection_rows)} radar detections -> {len(track_rows)} track rows")

    if all_rows:
        import os
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote {len(all_rows)} rows to {args.log}")


if __name__ == "__main__":
    main()
