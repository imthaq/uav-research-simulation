"""
metrics_validation.py

Validates each metric calculation against a small, hand-computable
example - the goal is a human can check the expected number on paper,
not that the simulation "looks reasonable".

Where a metric is already a standalone function (position RMSE, missed-
detection/false-alarm counts, track continuity/fragmentation,
communication load), this imports and calls the real function from
metrics_analysis.py. Where a metric is only ever computed inline inside
Simulation.step() (collision-risk count, near-miss count, mission
success, formation error), this replicates the exact formula in a tiny
local function with a comment pointing at the source lines it mirrors,
so a reviewer can cross-check the two stay in sync.

Run directly:
    python metrics_validation.py
"""
import math
import statistics
import sys
import os

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from metrics_analysis import _rmse, perception_metrics, communication_metrics

FAILURES = []


def check(name, actual, expected, tol=1e-6):
    ok = (abs(actual - expected) <= tol) if isinstance(expected, float) else (actual == expected)
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}: got {actual!r}, expected {expected!r}")
    if not ok:
        FAILURES.append(name)


def base_row(**overrides):
    """Every field perception_metrics() touches, defaulted to a harmless
    no-op value so each test only needs to override what it's testing."""
    row = dict(
        detected_x=None, detected_y=None, true_target_x=0.0, true_target_y=0.0,
        fused_x=None, fused_y=None,
        measured_radial_velocity=None, true_radial_velocity=None,
        uav_id=0, time_step=0,
        radar_track_id=None, track_status=None, missed_detection_flag=False,
        clutter_flag=False, false_alarm_flag=False,
        true_range=None, track_covariance_trace=None,
    )
    row.update(overrides)
    return row


# --- 1. position RMSE (metrics_analysis._rmse) -----------------------------
def test_position_rmse():
    # errors (3,4) -> distance 5; (0,0) -> distance 0
    # RMSE = sqrt(mean(5^2, 0^2)) = sqrt(12.5) = 3.535533...
    actual = _rmse([(3, 4), (0, 0)])
    check("position_rmse", actual, round(math.sqrt(12.5), 4))


# --- 2. average range error (not exposed elsewhere as an aggregate; ---
# --- built from the same true_range/measured_range fields the radar --
# --- model and pipeline rows already carry) ---------------------------
def avg_range_error(rows):
    errs = [abs(r["measured_range"] - r["true_range"]) for r in rows
            if r["measured_range"] is not None and r["true_range"] is not None]
    return round(statistics.mean(errs), 4) if errs else None


def test_avg_range_error():
    # true=[10,20,30], measured=[11,18,33] -> abs errors [1,2,3] -> mean 2.0
    rows = [
        {"true_range": 10, "measured_range": 11},
        {"true_range": 20, "measured_range": 18},
        {"true_range": 30, "measured_range": 33},
    ]
    check("avg_range_error", avg_range_error(rows), 2.0)


# --- 3 & 4. missed-detection count / false-alarm count (perception_metrics) -
def test_missed_and_false_alarm_counts():
    flags = [
        (True, False), (False, True), (True, False), (False, True), (True, False),
    ]
    rows = [base_row(uav_id=0, time_step=t, missed_detection_flag=m, false_alarm_flag=f)
            for t, (m, f) in enumerate(flags)]
    m = perception_metrics(rows)
    check("missed_detection_count", m["missed_track_count"], 3)
    check("false_alarm_count", m["false_track_count"], 2)


# --- 5. track continuity (perception_metrics) -------------------------
def test_track_continuity():
    statuses = ["tentative", "confirmed", "coasting", "lost", "lost"]
    rows = [base_row(uav_id=0, time_step=t, true_range=10.0, track_status=s)
            for t, s in enumerate(statuses)]
    m = perception_metrics(rows)
    # 3 of 5 rows are tentative/confirmed/coasting -> 0.6
    check("track_continuity", m["track_continuity"], 0.6)


# --- 6. track fragmentation (perception_metrics) -----------------------
def test_track_fragmentation():
    track_ids = [1, 1, 2, 2, 3]
    rows = [base_row(uav_id=0, time_step=t, radar_track_id=tid, track_status="confirmed")
            for t, tid in enumerate(track_ids)]
    m = perception_metrics(rows)
    # id changes at step2 (1->2) and step4 (2->3) = 2 fragmentations
    check("track_fragmentation", m["track_fragmentation"], 2)


# --- 7. collision-risk count (mirrors simple_swarm_sim.py _log_row(), ---
# --- ~line 822: collision_risk_flag = nearest_entity_distance <= near_miss_distance,
# --- counted the same way run_experiments.py's run_level_row() does) ---
def collision_risk_count(rows, near_miss_distance):
    return sum(1 for r in rows if r["nearest_entity_distance"] <= near_miss_distance)


def test_collision_risk_count():
    distances = [1.0, 3.5, 3.6, 10.0, 0.0]
    rows = [{"nearest_entity_distance": d} for d in distances]
    # <= 3.5: 1.0, 3.5, 0.0 -> 3
    check("collision_risk_count", collision_risk_count(rows, near_miss_distance=3.5), 3)


# --- 8. near-miss count (mirrors simple_swarm_sim.py step(), lines ~693-703:
# --- d <= collision_distance -> collision; collision_distance < d <= near_miss_distance -> near miss)
def classify_proximity(d, collision_distance, near_miss_distance):
    if d <= collision_distance:
        return "collision"
    if d <= near_miss_distance:
        return "near_miss"
    return "clear"


def test_near_miss_count():
    distances = [1.0, 1.5, 2.0, 3.5, 4.0]
    classes = [classify_proximity(d, collision_distance=1.5, near_miss_distance=3.5) for d in distances]
    # 1.0,1.5 -> collision; 2.0,3.5 -> near_miss; 4.0 -> clear
    check("near_miss_count", classes.count("near_miss"), 2)
    check("collision_count", classes.count("collision"), 2)


# --- 9. mission success (mirrors simple_swarm_sim.py _metrics(), line ~860:
# --- mission_success = all(reached_goal) and collision_count == 0) -----
def mission_success(reached_goal, collision_count):
    return bool(all(reached_goal) and collision_count == 0)


def test_mission_success():
    check("mission_success_all_reached_no_collision", mission_success([True, True], 0), True)
    check("mission_success_collision_blocks_it", mission_success([True, True], 1), False)
    check("mission_success_not_all_reached", mission_success([True, False], 0), False)


# --- 10. formation error (mirrors simple_swarm_sim.py step(), lines ~746-748:
# --- rmse = sqrt(mean((pairwise_dist - desired_spacing)^2))) -----------
def formation_rmse(pairwise_dists, spacing):
    return math.sqrt(sum((d - spacing) ** 2 for d in pairwise_dists) / len(pairwise_dists))


def test_formation_error():
    # spacing=8; dists=[8,10,6] -> deviations [0,2,-2] -> mean sq = 8/3 -> sqrt
    actual = formation_rmse([8.0, 10.0, 6.0], spacing=8.0)
    check("formation_error", actual, math.sqrt(8 / 3))


# --- 11. communication load (metrics_analysis.communication_metrics) ---
def test_communication_load():
    rows = [
        {"fusion_comm_messages": 2, "fusion_num_sources": 2, "fusion_response_time_steps": 1},
        {"fusion_comm_messages": 4, "fusion_num_sources": 3, "fusion_response_time_steps": 2},
        {"fusion_comm_messages": 6, "fusion_num_sources": 4, "fusion_response_time_steps": 3},
    ]
    m = communication_metrics(rows)
    check("communication_load", m["communication_load"], 4.0)
    check("messages_sent", m["messages_sent"], 12)
    check("messages_dropped", m["messages_dropped"], 3)  # (2-2)+(4-3)+(6-4)
    check("avg_message_delay_steps", m["avg_message_delay_steps"], 2.0)


def main():
    test_position_rmse()
    test_avg_range_error()
    test_missed_and_false_alarm_counts()
    test_track_continuity()
    test_track_fragmentation()
    test_collision_risk_count()
    test_near_miss_count()
    test_mission_success()
    test_formation_error()
    test_communication_load()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED: {', '.join(FAILURES)}")
        return 1
    print("All metric calculations validated.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
