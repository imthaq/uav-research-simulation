import math
import sys

FAILURES = []

def check(name, actual, expected, tol=1e-5):
    if actual is None and expected is None:
        ok = True
    elif actual is None or expected is None:
        ok = False
    else:
        ok = (abs(actual - expected) <= tol) if isinstance(expected, float) else (actual == expected)
    
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}: got {actual!r}, expected {expected!r}")
    if not ok:
        FAILURES.append(name)


def rmse(pairs):
    """Generic RMSE for a list of (actual, expected) tuples."""
    if not pairs: return 0.0
    return math.sqrt(sum((a - e)**2 for a, e in pairs) / len(pairs))


# =====================================================================
# 1. TRACKING METRICS
# =====================================================================
def test_tracking_metrics():
    # Position RMSE: points (3,4)->dist 5, (0,0)->dist 0. RMSE = sqrt(25/2) = 3.5355...
    pos_rmse = rmse([(5.0, 0.0), (0.0, 0.0)]) # passing error magnitudes instead of points
    check("position_RMSE", pos_rmse, math.sqrt(12.5))

    # Velocity RMSE: vx err=2, vy err=2 -> squared err = 8. Other point exact -> 0. RMSE = sqrt(4) = 2.0
    vel_err_mags = [math.sqrt(2**2 + 2**2), 0.0]
    vel_rmse = rmse([(mag, 0.0) for mag in vel_err_mags])
    check("velocity_RMSE", vel_rmse, 2.0)

    # Boolean flag counters
    flags = [
        {"missed_det": True, "false_det": False, "false_trk": False, "missed_trk": True},
        {"missed_det": False, "false_det": True, "false_trk": True, "missed_trk": False},
        {"missed_det": True, "false_det": False, "false_trk": False, "missed_trk": False},
    ]
    check("missed_detections", sum(f["missed_det"] for f in flags), 2)
    check("false_detections", sum(f["false_det"] for f in flags), 1)
    check("false_tracks", sum(f["false_trk"] for f in flags), 1)
    check("missed_tracks", sum(f["missed_trk"] for f in flags), 1)

    # Track continuity (ratio of time tracked vs total target lifespan)
    tracked_steps = 8
    lifespan = 10
    check("track_continuity", tracked_steps / lifespan, 0.8)

    # Track fragmentation (count of ID changes for the same true target)
    track_ids = [1, 1, 2, 2, 3] # ID changes at idx 2 and idx 4
    fragments = sum(1 for i in range(1, len(track_ids)) if track_ids[i] != track_ids[i-1])
    check("track_fragmentation", fragments, 2)

    # Association errors (wrong true target ID associated with track)
    # E.g., true ids: [1, 1, 1], associated ids: [1, 2, 1] -> 1 error
    true_ids = [1, 1, 1]
    assoc_ids = [1, 2, 1]
    assoc_errs = sum(1 for t, a in zip(true_ids, assoc_ids) if t != a)
    check("association_errors", assoc_errs, 1)

    # Track lifetime (steps from birth to death/loss)
    birth_step, loss_step = 12, 45
    check("track_lifetime", loss_step - birth_step, 33)


# =====================================================================
# 2. FUSION METRICS
# =====================================================================
def test_fusion_metrics():
    # Fused-position RMSE
    fused_err_mags = [2.0, 0.0, 4.0] # MSE = (4 + 0 + 16)/3 = 6.666... 
    fused_rmse = rmse([(mag, 0.0) for mag in fused_err_mags])
    check("fused_position_RMSE", fused_rmse, math.sqrt(20.0 / 3.0))

    # Covariance consistency (Normalized Estimation Error Squared - NEES <= threshold)
    # NEES = err_x^2 / cov_x + err_y^2 / cov_y. For (x_err=1, y_err=2, cov_x=1, cov_y=2), NEES = 1/1 + 4/2 = 3.
    nees = (1.0**2 / 1.0) + (2.0**2 / 2.0)
    check("covariance_consistency", nees, 3.0)

    # Sensor contribution (Weight assigned during fusion)
    weights = [0.2, 0.8]
    check("sensor_contribution_primary", weights[1], 0.8)

    # Stale-data count
    stale_flags = [False, True, True, False, False]
    check("stale_data_count", sum(stale_flags), 2)

    # Faulty-sensor influence (Deviation caused by injecting a faulty measurement)
    fused_normal = 10.0
    fused_with_fault = 14.5
    check("faulty_sensor_influence", abs(fused_with_fault - fused_normal), 4.5)


# =====================================================================
# 3. SWARM METRICS
# =====================================================================
def test_swarm_metrics():
    distances = [0.5, 2.0, 4.0, 10.0]
    coll_dist = 1.0
    near_miss_dist = 3.0

    # Classifications
    collisions = sum(1 for d in distances if d <= coll_dist)
    near_misses = sum(1 for d in distances if coll_dist < d <= near_miss_dist)
    risks = sum(1 for d in distances if d <= near_miss_dist)

    check("collision_count", collisions, 1)
    check("near_miss_count", near_misses, 1)
    check("collision_risk_count", risks, 2)

    # Minimum separation
    check("minimum_separation", min(distances), 0.5)

    # Response time & Completion time
    event_start = 5.0
    action_taken = 6.2
    goal_reached = 15.0
    check("response_time", action_taken - event_start, 1.2)
    check("mission_completion_time", goal_reached, 15.0)

    # Mission success
    all_reached = True
    check("mission_success", bool(all_reached and collisions == 0), False) # Fails because collisions=1

    # Formation error (RMSE of pairwise distances from ideal spacing)
    # ideal = 5.0, actuals = [5.0, 7.0] -> errs = [0, 2] -> MSE = 2.0 -> RMSE = sqrt(2)
    formation_rmse = rmse([(5.0, 5.0), (7.0, 5.0)])
    check("formation_error", formation_rmse, math.sqrt(2.0))

    # Unnecessary avoidance (avoidance triggered but min dist > near_miss)
    avoidance_triggered = True
    min_dist = 5.0
    unnecessary = 1 if (avoidance_triggered and min_dist > near_miss_dist) else 0
    check("unnecessary_avoidance", unnecessary, 1)

    # Hold duration
    hold_start = 10
    hold_end = 25
    check("hold_duration", hold_end - hold_start, 15)


# =====================================================================
# 4. COMMUNICATION METRICS
# =====================================================================
def test_communication_metrics():
    # Basic counters
    messages = {"sent": 100, "received": 80, "stale": 5}
    dropped = messages["sent"] - messages["received"]
    check("messages_sent", messages["sent"], 100)
    check("messages_received", messages["received"], 80)
    check("messages_dropped", dropped, 20)
    check("stale_messages", messages["stale"], 5)

    # Communication load (Bytes/step or msgs/step)
    bytes_sent = 5000
    steps = 10
    check("communication_load", bytes_sent / steps, 500.0)

    # Outage duration (max consecutive steps with 0 messages)
    # Received counts per step: [5, 5, 0, 0, 0, 5, 0]
    recv_history = [5, 5, 0, 0, 0, 5, 0]
    outages = []
    current_outage = 0
    for r in recv_history:
        if r == 0:
            current_outage += 1
        else:
            if current_outage > 0: outages.append(current_outage)
            current_outage = 0
    if current_outage > 0: outages.append(current_outage)
    
    max_outage = max(outages) if outages else 0
    check("outage_duration", max_outage, 3)

    # Recovery time (steps from outage end until performance/msgs nominal)
    outage_end_step = 4
    nominal_step = 6
    check("recovery_time", nominal_step - outage_end_step, 2)


def main():
    print("--- Tracking Metrics ---")
    test_tracking_metrics()
    print("\n--- Fusion Metrics ---")
    test_fusion_metrics()
    print("\n--- Swarm Metrics ---")
    test_swarm_metrics()
    print("\n--- Communication Metrics ---")
    test_communication_metrics()

    print("\n" + "="*30)
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED: {', '.join(FAILURES)}")
        return 1
    print("All metric calculations validated successfully.")
    return 0

if __name__ == "__main__":
    sys.exit(main())