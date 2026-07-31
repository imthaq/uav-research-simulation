"""
radar_track_model.py

Converts the raw, per-step radar detections produced by radar_like_model.py
into radar tracks over time, using a constant-velocity Kalman filter per
track and nearest-neighbor data association gated on the filter's own
innovation covariance (Mahalanobis distance), rather than a fixed-radius
Euclidean gate.

Each radar (one per UAV, since every UAV only sees its own radar's
detections) maintains its own independent set of tracks.

Track fields
------------
track_id            - unique id for this track (scoped to one radar/UAV)
est_x, est_y        - filtered position estimate
est_vx, est_vy      - filtered velocity estimate
covariance          - the filter's 4x4 state covariance, as a JSON-encoded
                       list of lists (over [pos_x, pos_y, vel_x, vel_y])
confidence          - confidence carried over from the most recent matched
                       detection (held steady while coasting on a miss)
age                 - number of steps since the track was created
hit_count           - total number of matched detections over the track's
                       life
missed_count        - consecutive steps with no matching detection (resets
                       to 0 on every match)
existence_probability - recursive belief that this is a real target: rises
                       toward 1 on every hit, decays on every miss
status              - "tentative" - just created, not yet matched
                                     CONFIRM_HITS times in a row
                     - "confirmed" - matched CONFIRM_HITS times in a row
                     - "coasting"  - a previously-matched track with no
                                      detection this step, predicted forward
                                      on its Kalman motion model
                     - "lost"      - missed MAX_MISSED times in a row (or
                                      existence_probability collapsed); this
                                      row is the last one where the track is
                                      still predicted/matched against
                     - "deleted"   - final row for a track, emitted the step
                                      after it went "lost"; it is then
                                      dropped for good

How tracks are formed (Kalman filter + gated nearest-neighbor association)
----------------------------------------------------------------------------
At each time step, for every radar:
  1. Every existing track predicts its next state (position/velocity/
     covariance) forward with a constant-velocity motion model.
  2. Tracks are matched to this step's detections by nearest neighbor:
     candidate (track, detection) pairs whose Mahalanobis distance (using
     the track's innovation covariance) falls under GATE_CHI2 are
     considered in order of increasing distance, and each track/detection
     is used at most once (best matches get first pick).
  3. Matched tracks run a Kalman update from the matched detection, reset
     missed_count, and bump existence_probability up.
  4. Unmatched tracks stay at their predicted state, missed_count goes up,
     and existence_probability decays; too many misses (or a collapsed
     existence_probability) -> status "lost", then "deleted" next step.
  5. Unmatched detections spawn new tentative tracks.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_MODELS_DIR = os.path.join(_ROOT_DIR, "models")
for _p in (_ROOT_DIR, _MODELS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from models.radar_like_model import RadarLikeModel

# Consecutive matches needed before a tentative track becomes confirmed.
CONFIRM_HITS = 3

# Consecutive misses allowed before a track is dropped as lost.
MAX_MISSED = 3

# Mahalanobis-distance-squared gate (~99% confidence region, 2 DOF).
GATE_CHI2 = 9.21

# Existence-probability recursion: how much a hit/miss moves the belief,
# and the floor below which a track is lost outright even before MAX_MISSED.
EXIST_PROB_INIT = 0.65
EXIST_PROB_HIT_GAIN = 0.15
EXIST_PROB_MISS_DECAY = 0.3
EXIST_PROB_DELETE_FLOOR = 0.1

# Constant-velocity process noise (accel std, world units/s^2). Tunable -
# higher trusts new detections more, lower trusts the motion model more.
PROCESS_ACCEL_STD = 1.0

# Initial velocity uncertainty for a freshly spawned track (position
# uncertainty starts at the measurement noise itself).
INIT_VELOCITY_VAR = 25.0

_H = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])
_I4 = np.eye(4)


def _F(dt):
    return np.array([[1.0, 0.0, dt, 0.0],
                      [0.0, 1.0, 0.0, dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])


def _Q(dt, accel_std=PROCESS_ACCEL_STD):
    """Discrete white-noise-acceleration process noise for a constant-
    velocity model."""
    q = accel_std ** 2
    dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
    return q * np.array([
        [dt4 / 4, 0.0, dt3 / 2, 0.0],
        [0.0, dt4 / 4, 0.0, dt3 / 2],
        [dt3 / 2, 0.0, dt2, 0.0],
        [0.0, dt3 / 2, 0.0, dt2],
    ])


class RadarTrack:
    """One tracked object as seen by a single radar (UAV), estimated with
    a constant-velocity Kalman filter."""

    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    COASTING = "coasting"
    LOST = "lost"
    DELETED = "deleted"

    def __init__(self, radar_id, track_num, x, y, confidence, r_std, step):
        self.track_id = f"r{radar_id}_t{track_num}"
        self.radar_id = radar_id
        self.state = np.array([x, y, 0.0, 0.0])
        self.P = np.diag([r_std ** 2, r_std ** 2, INIT_VELOCITY_VAR, INIT_VELOCITY_VAR])
        self.confidence = confidence
        self.age = 1
        self.hit_count = 1
        self.missed_count = 0
        self.existence_prob = EXIST_PROB_INIT
        self.status = self.TENTATIVE
        self._hit_streak = 1
        self._last_det = None

    def predict(self, dt):
        F = _F(dt)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + _Q(dt)

    def _innovation(self, det_x, det_y, r_std):
        z = np.array([det_x, det_y])
        R = np.eye(2) * (r_std ** 2)
        S = _H @ self.P @ _H.T + R
        y = z - _H @ self.state
        return y, S

    def mahalanobis_sq(self, det_x, det_y, r_std):
        y, S = self._innovation(det_x, det_y, r_std)
        try:
            return float(y @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return float("inf")

    def apply_match(self, det_x, det_y, confidence, r_std):
        y, S = self._innovation(det_x, det_y, r_std)
        K = self.P @ _H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (_I4 - K @ _H) @ self.P

        self.confidence = confidence
        self.missed_count = 0
        self._hit_streak += 1
        self.hit_count += 1
        self.age += 1
        self.existence_prob = min(0.99, self.existence_prob + (1 - self.existence_prob) * EXIST_PROB_HIT_GAIN)
        self.status = self.CONFIRMED if (self.status == self.CONFIRMED
                                          or self._hit_streak >= CONFIRM_HITS) else self.TENTATIVE

    def apply_miss(self):
        self.missed_count += 1
        self._hit_streak = 0
        self.age += 1
        self.existence_prob = max(0.01, self.existence_prob * (1 - EXIST_PROB_MISS_DECAY))
        if self.missed_count >= MAX_MISSED or self.existence_prob < EXIST_PROB_DELETE_FLOOR:
            self.status = self.LOST
        else:
            self.status = self.COASTING

    def as_row(self, step):
        return {
            "time_step": step,
            "radar_id": self.radar_id,
            "track_id": self.track_id,
            "est_x": round(float(self.state[0]), 4),
            "est_y": round(float(self.state[1]), 4),
            "est_vx": round(float(self.state[2]), 4),
            "est_vy": round(float(self.state[3]), 4),
            "covariance": json.dumps(np.round(self.P, 6).tolist()),
            "confidence": self.confidence,
            "age": self.age,
            "hit_count": self.hit_count,
            "missed_count": self.missed_count,
            "existence_probability": round(float(self.existence_prob), 4),
            "status": self.status,
        }
        if getattr(self, "_last_det", None) is not None:
            row["detection_status"] = self._last_det.get("detection_status")
            row["probability_of_detection"] = self._last_det.get("probability_of_detection")
            row["radar_pd_miss_flag"] = self._last_det.get("radar_pd_miss_flag", False)
            row["false_alarm_flag"] = self._last_det.get("false_alarm_flag", False)
            row["missed_detection_flag"] = self._last_det.get("missed_detection_flag", False)
            row["dropout_flag"] = self._last_det.get("dropout_flag", False)
            row["ghost_track_flag"] = self._last_det.get("ghost_track_flag", False)
            row["extended_target_fragmentation_flag"] = self._last_det.get("extended_target_fragmentation_flag", False)
            row["doppler_ambiguity_flag"] = self._last_det.get("doppler_ambiguity_flag", False)
            row["multipath_false_track_flag"] = self._last_det.get("multipath_false_track_flag", False)
        return row


class RadarTracker:
    """Runs Kalman-filter tracking with gated nearest-neighbor association
    for a single radar (one UAV) across a sequence of steps."""

    def __init__(self, radar_id, r_std=1.0):
        self.radar_id = radar_id
        self.r_std = r_std
        self.tracks = []
        self._next_track_num = 1

    def _match(self, tracks, detections):
        """Greedy nearest-neighbor association: build every (track,
        detection) pair inside the Mahalanobis gate, then claim pairs in
        order of increasing distance so the closest (most statistically
        likely) matches win first."""
        candidates = []
        for ti, track in enumerate(tracks):
            for di, det in enumerate(detections):
                d2 = track.mahalanobis_sq(det["x"], det["y"], self.r_std)
                if d2 <= GATE_CHI2:
                    candidates.append((d2, ti, di))
        candidates.sort(key=lambda c: c[0])

        matched_track, matched_det = {}, {}
        for d2, ti, di in candidates:
            if ti in matched_track or di in matched_det:
                continue
            matched_track[ti] = di
            matched_det[di] = ti
        return matched_track

    def update(self, step, detections, dt):
        """Process one step's detections (list of dicts with at least
        x, y, confidence) for this radar. Returns the rows to log for this
        step: one per track still being processed, including a final
        "deleted" row for any track that went "lost" last step."""
        rows = []

        # Tracks that were marked "lost" last step get one final "deleted"
        # row this step, then are dropped for good.
        active = []
        for track in self.tracks:
            if track.status == RadarTrack.LOST:
                track.status = RadarTrack.DELETED
                track.age += 1
                rows.append(track.as_row(step))
            else:
                active.append(track)

        for track in active:
            track.predict(dt)

        # Task 8: two-pass gated association. Pass 1 matches tracks
        # against real (non-extended) detections only - with no extended
        # returns present this is exactly the original single-pass
        # algorithm, so behavior for every pre-existing scenario is
        # unchanged. Pass 2 then matches whatever's left (any leftover
        # real detections plus every extra return) against tracks pass 1
        # didn't claim, so an extra return can still reinforce a track
        # whose own dominant return was missed this scan - but it can
        # never steal the match its dominant return would otherwise have
        # made, which is what let a single extended target spawn
        # duplicate tracks before this fix (an extra winning the gate
        # this scan left the genuine dominant unmatched, and unlike an
        # extra, an unmatched dominant is allowed to seed a new track).
        real_idx = [i for i, d in enumerate(detections) if not d.get("is_extended_return")]

        matched_track, matched_det = {}, {}
        for ti, local_di in self._match(active, [detections[i] for i in real_idx]).items():
            di = real_idx[local_di]
            matched_track[ti] = di
            matched_det[di] = ti

        leftover_tracks_idx = [ti for ti in range(len(active)) if ti not in matched_track]
        leftover_dets_idx = [di for di in range(len(detections)) if di not in matched_det]
        if leftover_tracks_idx and leftover_dets_idx:
            sub_tracks = [active[ti] for ti in leftover_tracks_idx]
            sub_dets = [detections[di] for di in leftover_dets_idx]
            for sub_ti, sub_di in self._match(sub_tracks, sub_dets).items():
                ti = leftover_tracks_idx[sub_ti]
                di = leftover_dets_idx[sub_di]
                matched_track[ti] = di
                matched_det[di] = ti

        still_active = []
        for ti, track in enumerate(active):
            if ti in matched_track:
                det = detections[matched_track[ti]]
                track.apply_match(det["x"], det["y"], det.get("confidence"), self.r_std)
                track._last_det = det
            else:
                track.apply_miss()
                track._last_det = None
            rows.append(track.as_row(step))
            still_active.append(track)

        matched_det_indices = set(matched_track.values())
        for di, det in enumerate(detections):
            if di in matched_det_indices:
                continue
            # Task 8: extended-target radar returns. An extra return that
            # didn't match any existing track represents the same
            # physical object as its (separately-reported) dominant
            # return, not a new one - so unlike an ordinary unmatched
            # detection, it must not spawn a track of its own. It can
            # still strengthen/update an existing track above (it went
            # through the same gated match as any other detection), and
            # is simply dropped here when it doesn't. Without this, a
            # single extended target's scattered extra returns would
            # each seed their own permanent track.
            if det.get("is_extended_return"):
                continue
            new_track = RadarTrack(
                self.radar_id, self._next_track_num, det["x"], det["y"],
                det.get("confidence"), self.r_std, step)
            self._next_track_num += 1
            new_track._last_det = det
            still_active.append(new_track)
            rows.append(new_track.as_row(step))

        self.tracks = still_active
        return rows


def build_tracks(scenario_name, detection_rows, dt, measurement_std=1.0):
    """Runs one RadarTracker per radar_id over detection_rows (as produced
    by radar_like_model.RadarLikeModel.run(), already limited to one
    scenario) and returns the full list of track log rows, in step order.

    Only rows with an actual x/y this step (status "detected" or
    "false_alarm") are fed to the tracker - "missed"/"dropout" rows have no
    position to associate, so a real radar simply wouldn't see anything to
    report that step. Task 8: each detection also carries the row's
    is_extended_return flag (False if the row/key predates that field), so
    RadarTracker.update can let extra returns match/reinforce an existing
    track without letting an unmatched one spawn a new one."""
    by_radar_step = {}
    for row in detection_rows:
        if row["detected_x"] is None or row["detected_y"] is None:
            continue
        key = (row["radar_id"], row["time_step"])
        by_radar_step.setdefault(key, []).append({
            "x": row["detected_x"],
            "y": row["detected_y"],
            "confidence": row["confidence_score"],
            "is_extended_return": bool(row.get("is_extended_return", False)),
        })

    radar_ids = sorted({r for r, _ in by_radar_step})
    max_step = max((s for _, s in by_radar_step), default=-1)

    trackers = {radar_id: RadarTracker(radar_id, measurement_std) for radar_id in radar_ids}

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
        description="Turn raw radar detections into Kalman-filter radar tracks")
    parser.add_argument("--config", default=os.path.join(_ROOT_DIR, "simulation_config.json"))
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default=os.path.join(_ROOT_DIR, "logs", "radar_track_log.csv"))
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    dt = config["sim"]["dt"]
    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        model = RadarLikeModel(config, name)
        detection_rows = model.run()
        track_rows = build_tracks(name, detection_rows, dt, model.range_noise_std)
        all_rows.extend(track_rows)
        print(f"{name}: {len(detection_rows)} radar detections -> {len(track_rows)} track rows")

    if all_rows:
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote {len(all_rows)} rows to {args.log}")


if __name__ == "__main__":
    main()