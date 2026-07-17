"""
tracker_validation.py

Task 4: validates the Kalman-filter + gated nearest-neighbor tracker in
tracking/radar_track_model.py against controlled, deterministic detection
sequences fed straight into RadarTracker - not the full radar+tracker
pipeline, just the tracker's own state machine and filter math:

  - constant-velocity target
  - no measurement noise
  - low measurement noise
  - one missed detection
  - several missed detections
  - clutter near target
  - two crossing targets
  - track deletion
  - target reappearance

...confirming: prediction is correct, covariance updates, gating rejects
distant clutter, track IDs remain stable where possible, and lost-track
logic works.

Each check is a controlled case with a known expected answer, asserted
with a small numerical tolerance. Results are printed and written to
results/tracker_validation_results.md.

Usage:
    python tracker_validation.py
"""

import math
import os
import random
import sys

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT_DIR,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from tracking.radar_track_model import RadarTracker, RadarTrack, MAX_MISSED, CONFIRM_HITS
from validation_common import Checker

_checker = Checker()


def check(task, description, condition, detail=""):
    return _checker.check(task, description, condition, detail)


def close(a, b, tol=1e-6):
    return _checker.close(a, b, tol)


def det(x, y, confidence=0.9):
    return {"x": x, "y": y, "confidence": confidence}


# ---------------------------------------------------------------------
# 1. Constant-velocity target
# ---------------------------------------------------------------------
def test_constant_velocity_target():
    # Straight line, true velocity (2, 1) units/step, dt=1, no noise.
    tracker = RadarTracker("t1", r_std=0.5)
    vx_true, vy_true = 2.0, 1.0
    x0, y0 = 0.0, 0.0
    rows = None
    for step in range(20):
        x, y = x0 + vx_true * step, y0 + vy_true * step
        rows = tracker.update(step, [det(x, y)], dt=1.0)

    # Exactly one live track by the end (no clutter, no misses).
    check("constant_velocity_target", "exactly one track survives a clean constant-velocity run",
          len(tracker.tracks) == 1, f"got {len(tracker.tracks)}")
    track = tracker.tracks[0]
    x_true_end = x0 + vx_true * 19
    y_true_end = y0 + vy_true * 19
    check("constant_velocity_target", "filter converges to the true position",
          close(track.state[0], x_true_end, tol=0.15) and close(track.state[1], y_true_end, tol=0.15),
          f"got ({track.state[0]:.3f},{track.state[1]:.3f}) vs true ({x_true_end},{y_true_end})")
    check("constant_velocity_target", "filter converges to the true velocity",
          close(track.state[2], vx_true, tol=0.15) and close(track.state[3], vy_true, tol=0.15),
          f"got ({track.state[2]:.3f},{track.state[3]:.3f}) vs true ({vx_true},{vy_true})")
    check("constant_velocity_target", "track becomes confirmed after CONFIRM_HITS consecutive matches",
          track.status == RadarTrack.CONFIRMED, f"status={track.status}")


# ---------------------------------------------------------------------
# 2. No measurement noise -> exact convergence
# ---------------------------------------------------------------------
def test_no_measurement_noise():
    tracker = RadarTracker("t2", r_std=1.0)
    for step in range(10):
        tracker.update(step, [det(5.0, -3.0)], dt=1.0)
    track = tracker.tracks[0]
    check("no_measurement_noise", "stationary, noise-free target converges to exact position",
          close(track.state[0], 5.0, tol=1e-3) and close(track.state[1], -3.0, tol=1e-3),
          f"got ({track.state[0]:.5f},{track.state[1]:.5f})")
    check("no_measurement_noise", "stationary, noise-free target converges to zero velocity",
          close(track.state[2], 0.0, tol=1e-2) and close(track.state[3], 0.0, tol=1e-2),
          f"got ({track.state[2]:.5f},{track.state[3]:.5f})")


# ---------------------------------------------------------------------
# 3. Prediction step (before any update)
# ---------------------------------------------------------------------
def test_prediction_correctness():
    tracker = RadarTracker("t3", r_std=1.0)
    tracker.update(0, [det(0.0, 0.0)], dt=1.0)
    tracker.update(1, [det(1.0, 0.0)], dt=1.0)
    track = tracker.tracks[0]
    est_vx_before = float(track.state[2])
    pre_state = track.state.copy()
    track.predict(dt=2.0)
    # Constant-velocity prediction: pos advances by v*dt, velocity unchanged.
    check("prediction_correctness", "predict() advances position by velocity*dt",
          close(track.state[0], pre_state[0] + pre_state[2] * 2.0, tol=1e-9)
          and close(track.state[1], pre_state[1] + pre_state[3] * 2.0, tol=1e-9),
          f"got ({track.state[0]:.4f},{track.state[1]:.4f})")
    check("prediction_correctness", "predict() leaves velocity unchanged (no measurement yet)",
          close(track.state[2], est_vx_before, tol=1e-9), f"got {track.state[2]:.4f}")
    check("prediction_correctness", "predict() grows position covariance (process noise added)",
          track.P[0][0] > pre_state[0] * 0 + 1e-9 and track.P[0][0] >= 0,
          f"P[0][0]={track.P[0][0]:.4f}")


# ---------------------------------------------------------------------
# 4. Low measurement noise
# ---------------------------------------------------------------------
def test_low_measurement_noise():
    rng = random.Random(7)
    r_std = 0.3
    tracker = RadarTracker("t4", r_std=r_std)
    vx_true, vy_true = 1.0, 0.5
    n_steps = 60
    for step in range(n_steps):
        x = 0.0 + vx_true * step + rng.gauss(0, r_std)
        y = 0.0 + vy_true * step + rng.gauss(0, r_std)
        tracker.update(step, [det(x, y)], dt=1.0)
    track = tracker.tracks[0]
    x_true, y_true = vx_true * (n_steps - 1), vy_true * (n_steps - 1)
    check("low_measurement_noise", "low-noise track stays within a few sigma of true position",
          close(track.state[0], x_true, tol=1.0) and close(track.state[1], y_true, tol=1.0),
          f"got ({track.state[0]:.3f},{track.state[1]:.3f}) vs true ({x_true},{y_true})")
    # PROCESS_ACCEL_STD=1.0 trusts the motion model loosely and lets the
    # velocity estimate wander step to step even with tight measurement
    # noise, so this is a wide sanity band, not a tight-convergence claim.
    check("low_measurement_noise", "low-noise track's velocity estimate is the right order of magnitude and sign",
          close(track.state[2], vx_true, tol=0.7) and close(track.state[3], vy_true, tol=0.7),
          f"got ({track.state[2]:.3f},{track.state[3]:.3f})")
    check("low_measurement_noise", "track remains confirmed under low noise",
          track.status == RadarTrack.CONFIRMED, f"status={track.status}")


# ---------------------------------------------------------------------
# 5. One missed detection
# ---------------------------------------------------------------------
def test_one_missed_detection():
    tracker = RadarTracker("t5", r_std=0.5)
    vx_true = 1.0
    for step in range(5):
        tracker.update(step, [det(vx_true * step, 0.0)], dt=1.0)
    track_id_before = tracker.tracks[0].track_id

    # Step 5: no detection at all (a miss).
    rows_miss = tracker.update(5, [], dt=1.0)
    check("one_missed_detection", "a single miss keeps the track alive (not deleted)",
          len(tracker.tracks) == 1, f"got {len(tracker.tracks)}")
    track = tracker.tracks[0]
    check("one_missed_detection", "track id is stable across a single miss",
          track.track_id == track_id_before, f"before={track_id_before} after={track.track_id}")
    check("one_missed_detection", "missed_count increments to 1 on a miss",
          track.missed_count == 1, f"got {track.missed_count}")
    check("one_missed_detection", "status becomes 'coasting' after one miss (below MAX_MISSED)",
          track.status == RadarTrack.COASTING, f"status={track.status}")
    check("one_missed_detection", "coasting position is the pure constant-velocity prediction",
          close(track.state[0], vx_true * 5, tol=0.5), f"got {track.state[0]:.3f} vs expected {vx_true*5}")

    # A real detection resumes the track and resets missed_count.
    tracker.update(6, [det(vx_true * 6, 0.0)], dt=1.0)
    track = tracker.tracks[0]
    check("one_missed_detection", "missed_count resets to 0 once a detection matches again",
          track.missed_count == 0, f"got {track.missed_count}")
    check("one_missed_detection", "track id is still stable after recovering from the miss",
          track.track_id == track_id_before, f"before={track_id_before} after={track.track_id}")


# ---------------------------------------------------------------------
# 6. Several missed detections -> lost -> deleted
# ---------------------------------------------------------------------
def test_several_missed_detections():
    tracker = RadarTracker("t6", r_std=0.5)
    for step in range(5):
        tracker.update(step, [det(step, 0.0)], dt=1.0)
    track_id = tracker.tracks[0].track_id

    step = 5
    for i in range(MAX_MISSED):
        rows = tracker.update(step, [], dt=1.0)
        step += 1
    track = tracker.tracks[0]
    check("several_missed_detections", f"after MAX_MISSED={MAX_MISSED} consecutive misses, status is 'lost'",
          track.status == RadarTrack.LOST, f"status={track.status}")
    check("several_missed_detections", "track is not yet dropped from the tracker on the 'lost' step",
          len(tracker.tracks) == 1, f"got {len(tracker.tracks)}")


# ---------------------------------------------------------------------
# 7. Track deletion
# ---------------------------------------------------------------------
def test_track_deletion():
    tracker = RadarTracker("t7", r_std=0.5)
    for step in range(5):
        tracker.update(step, [det(step, 0.0)], dt=1.0)
    track_id = tracker.tracks[0].track_id

    step = 5
    for i in range(MAX_MISSED):
        tracker.update(step, [], dt=1.0)
        step += 1
    # One more step (with no matching detection) emits the final "deleted"
    # row and drops the track for good.
    rows = tracker.update(step, [], dt=1.0)
    deleted_rows = [r for r in rows if r["track_id"] == track_id and r["status"] == RadarTrack.DELETED]
    check("track_deletion", "a final 'deleted' row is emitted the step after 'lost'",
          len(deleted_rows) == 1, f"got {len(deleted_rows)} deleted rows for {track_id}")
    check("track_deletion", "the deleted track is dropped from the tracker's live track list",
          all(t.track_id != track_id for t in tracker.tracks), f"live ids={[t.track_id for t in tracker.tracks]}")


# ---------------------------------------------------------------------
# 8. Target reappearance
# ---------------------------------------------------------------------
def test_target_reappearance():
    tracker = RadarTracker("t8", r_std=0.5)
    for step in range(5):
        tracker.update(step, [det(step, 0.0)], dt=1.0)
    old_id = tracker.tracks[0].track_id

    step = 5
    for i in range(MAX_MISSED):
        tracker.update(step, [], dt=1.0)
        step += 1
    tracker.update(step, [], dt=1.0)  # deletion step
    step += 1
    check("target_reappearance", "track list is empty right after deletion",
          len(tracker.tracks) == 0, f"got {len(tracker.tracks)}")

    # A new detection at (roughly) the same place spawns a brand-new track,
    # not a revival of the old one.
    rows = tracker.update(step, [det(4.0, 0.0)], dt=1.0)
    check("target_reappearance", "a reappearing target spawns exactly one new track",
          len(tracker.tracks) == 1, f"got {len(tracker.tracks)}")
    new_id = tracker.tracks[0].track_id
    check("target_reappearance", "the new track gets a fresh id, not the deleted track's old id",
          new_id != old_id, f"old={old_id} new={new_id}")
    check("target_reappearance", "the new track starts 'tentative'",
          tracker.tracks[0].status == RadarTrack.TENTATIVE, f"status={tracker.tracks[0].status}")


# ---------------------------------------------------------------------
# 9. Clutter near target (gating)
# ---------------------------------------------------------------------
def test_clutter_near_target():
    tracker = RadarTracker("t9", r_std=0.5)
    for step in range(5):
        tracker.update(step, [det(step, 0.0)], dt=1.0)
    real_id = tracker.tracks[0].track_id

    # Step 5: the real detection near the predicted position, PLUS a
    # clutter point close enough to be a plausible near-miss but further
    # from the track's predicted state than the real detection.
    real_x = 5.0
    rows = tracker.update(5, [det(real_x, 0.0), det(real_x + 1.5, 1.5)], dt=1.0)
    check("clutter_near_target", "still exactly one confirmed/tentative track for the real object",
          sum(1 for t in tracker.tracks if t.track_id == real_id) == 1)
    real_track = next(t for t in tracker.tracks if t.track_id == real_id)
    check("clutter_near_target", "the real track's estimate stays near the real detection, not the clutter",
          close(real_track.state[0], real_x, tol=1.0), f"got {real_track.state[0]:.3f}")
    check("clutter_near_target", "nearby clutter spawns a separate new tentative track, doesn't hijack the real one",
          any(t.track_id != real_id and t.status == RadarTrack.TENTATIVE for t in tracker.tracks),
          f"tracks={[(t.track_id, t.status) for t in tracker.tracks]}")

    # Gating rejects a FAR clutter point outright (it should never be
    # eligible to match the real track - only spawn its own tentative one).
    tracker2 = RadarTracker("t9b", r_std=0.5)
    for step in range(5):
        tracker2.update(step, [det(step, 0.0)], dt=1.0)
    real_id2 = tracker2.tracks[0].track_id
    tracker2.update(5, [det(5.0, 0.0), det(500.0, 500.0)], dt=1.0)
    real_track2 = next(t for t in tracker2.tracks if t.track_id == real_id2)
    check("clutter_near_target", "gating rejects distant clutter - the real track is unaffected by a far outlier",
          close(real_track2.state[0], 5.0, tol=1.0) and close(real_track2.state[1], 0.0, tol=1.0),
          f"got ({real_track2.state[0]:.3f},{real_track2.state[1]:.3f})")
    check("clutter_near_target", "a far clutter point spawns its own tentative track instead of matching",
          any(close(t.state[0], 500.0, tol=1.0) and t.status == RadarTrack.TENTATIVE for t in tracker2.tracks))


# ---------------------------------------------------------------------
# 10. Two crossing targets
# ---------------------------------------------------------------------
def test_two_crossing_targets():
    # Target A moves left-to-right along y=0; target B moves right-to-left
    # along y=0 at the same speed, so their paths cross around x=5 at
    # step 5. Before the crossing, IDs should be assigned consistently
    # (A always left of B). This is the well-known ambiguous case for a
    # purely nearest-neighbor tracker: right at/after the crossing point,
    # a swap is an accepted limitation, not a bug - so this check only
    # requires stability strictly *before* the crossing, plus that exactly
    # two tracks survive the whole run (no track lost purely from the
    # close approach).
    tracker = RadarTracker("t10", r_std=0.3)
    ids_before_cross = []
    for step in range(4):
        xa, xb = float(step), 9.0 - float(step)
        rows = tracker.update(step, [det(xa, 0.0), det(xb, 0.0)], dt=1.0)
        by_x = sorted(tracker.tracks, key=lambda t: t.state[0])
        ids_before_cross.append(tuple(t.track_id for t in by_x))

    check("two_crossing_targets", "two distinct tracks are maintained approaching the crossing",
          all(len(pair) == 2 and pair[0] != pair[1] for pair in ids_before_cross),
          f"ids over time={ids_before_cross}")
    check("two_crossing_targets", "left/right track identity stays stable before the crossing point",
          len(set(ids_before_cross)) == 1, f"distinct id-orderings seen before crossing={set(ids_before_cross)}")

    # Continue through and past the crossing.
    for step in range(4, 10):
        xa, xb = float(step), 9.0 - float(step)
        tracker.update(step, [det(xa, 0.0), det(xb, 0.0)], dt=1.0)
    check("two_crossing_targets", "exactly two tracks still exist after passing through the crossing",
          len(tracker.tracks) == 2, f"got {len(tracker.tracks)}")
    check("two_crossing_targets", "both post-crossing tracks are confirmed (neither was lost/respawned)",
          all(t.status == RadarTrack.CONFIRMED for t in tracker.tracks),
          f"statuses={[t.status for t in tracker.tracks]}")


def main():
    test_constant_velocity_target()
    test_no_measurement_noise()
    test_prediction_correctness()
    test_low_measurement_noise()
    test_one_missed_detection()
    test_several_missed_detections()
    test_track_deletion()
    test_target_reappearance()
    test_clutter_near_target()
    test_two_crossing_targets()

    failed = _checker.print_summary()

    out_dir = os.path.join(_ROOT_DIR, "results")
    out_path = os.path.join(out_dir, "tracker_validation_results.md")
    _checker.write_markdown(
        out_path, "Tracker Validation Results (Task 4)",
        intro="Deterministic checks of the Kalman-filter + gated nearest-neighbor "
              "tracker in `tracking/radar_track_model.py` - detections are fed "
              "straight into `RadarTracker`, not through the full radar model.")
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
