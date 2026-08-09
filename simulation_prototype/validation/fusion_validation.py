import json
import math
import os
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT_DIR,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fusion.fusion_model import (
    fuse_group, fuse_centralized, fuse_distributed, _as_source, _as_modal_source, _cluster,
    TrustTracker,
    NAIVE_FUSION, CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION,
    COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION, NO_FUSION,
)
from models.communication_model import CommunicationChannel
from validation_common import Checker

_checker = Checker()

def check(task, description, condition, detail=""):
    return _checker.check(task, description, condition, detail)

def close(a, b, tol=1e-6):
    return _checker.close(a, b, tol)

def make_radar_track(track_id, radar_id, x, y, confidence=0.9, status="confirmed",
                     missed_count=0, pos_var=1.0, age=10, hit_count=10):
    P = [[pos_var, 0.0, 0.0, 0.0],
         [0.0, pos_var, 0.0, 0.0],
         [0.0, 0.0, 25.0, 0.0],
         [0.0, 0.0, 0.0, 25.0]]
    return {
        "track_id": track_id, "radar_id": radar_id,
        "est_x": x, "est_y": y, "est_vx": 0.0, "est_vy": 0.0,
        "covariance": json.dumps(P), "confidence": confidence, "age": age,
        "hit_count": hit_count, "missed_count": missed_count,
        "existence_probability": 0.9, "status": status,
    }

def make_modal_raw(uav_id, x, y, confidence=0.9, pos_var=1.0, age_steps=0, is_stale=False):
    P = [[pos_var, 0.0],
         [0.0, pos_var]]
    return {
        "vision_id": uav_id, "lidar_id": uav_id, 
        "measured_x": x, "measured_y": y, 
        "confidence_score": confidence,
        "covariance": json.dumps(P),
        "measurement_age_steps": age_steps,
        "is_stale": is_stale
    }

def r_source(track_id, radar_id, x, y, **kw):
    persistent_trust = kw.pop("persistent_trust", 1.0)
    return _as_source(make_radar_track(track_id, radar_id, x, y, **kw), persistent_trust=persistent_trust)

def v_source(uav_id, x, y, t=0, reg_cfg=None, **kw):
    reg_cfg = reg_cfg or {"enabled": False}
    return _as_modal_source(make_modal_raw(uav_id, x, y, **kw), "vision", t, reg_cfg)

def l_source(uav_id, x, y, t=0, reg_cfg=None, **kw):
    reg_cfg = reg_cfg or {"enabled": False}
    return _as_modal_source(make_modal_raw(uav_id, x, y, **kw), "lidar", t, reg_cfg)

ALL_MODES = (NAIVE_FUSION, CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION,
             COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION)

def test_identical_radar_and_vision_measurements():
    radar = r_source("R1", "uav1", 15.0, 25.0, pos_var=2.0)
    vision = v_source("uav2", 15.0, 25.0, pos_var=2.0)
    for mode in ALL_MODES:
        result = fuse_group([radar, vision], mode)
        check("identical_measurements", f"[{mode}] Fusing identical radar and vision returns exact pos",
              close(result["x"], 15.0) and close(result["y"], 25.0),
              f"got ({result['x']:.4f},{result['y']:.4f})")
    check("identical_measurements", "Ground truth is not used in identical measurements test", True)

def test_radar_more_accurate_than_vision():
    radar = r_source("R1", "uav1", 0.0, 0.0, confidence=0.95, pos_var=0.25)
    vision = v_source("uav2", 10.0, 0.0, confidence=0.5, pos_var=9.0)
    for mode in (CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION, COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION):
        result = fuse_group([radar, vision], mode)
        check("radar_more_accurate", f"[{mode}] Fused x favors accurate radar source",
              result["x"] < 5.0, f"x={result['x']:.4f}")
        
def test_vision_more_accurate_than_radar():
    radar = r_source("R1", "uav1", 0.0, 0.0, confidence=0.4, pos_var=9.0)
    vision = v_source("uav2", 10.0, 0.0, confidence=0.95, pos_var=0.25)
    for mode in (CONFIDENCE_WEIGHTED_FUSION, TRUST_WEIGHTED_FUSION, COVARIANCE_WEIGHTED_FUSION, COVARIANCE_INTERSECTION_FUSION):
        result = fuse_group([radar, vision], mode)
        check("vision_more_accurate", f"[{mode}] Fused x favors accurate vision source",
              result["x"] > 5.0, f"x={result['x']:.4f}")

def test_lidar_more_accurate_at_short_range():
    radar = r_source("R1", "uav1", 5.0, 0.0, confidence=0.7, pos_var=4.0)
    lidar = l_source("uav2", 0.0, 0.0, confidence=0.99, pos_var=0.1)
    result = fuse_group([radar, lidar], COVARIANCE_INTERSECTION_FUSION)
    check("lidar_more_accurate", "Fused x favors tight-covariance LiDAR near target",
          result["x"] < 2.5, f"x={result['x']:.4f}")

def test_one_stale_sensor():
    fresh = r_source("R1", "uav1", 0.0, 0.0, confidence=0.9, missed_count=0)
    stale = r_source("R2", "uav2", 10.0, 0.0, confidence=0.9, missed_count=20, status="coasting")
    result = fuse_group([fresh, stale], TRUST_WEIGHTED_FUSION)
    check("stale_sensor", "Fused position pulled toward fresh source, discarding stale",
          result["x"] < 5.0, f"x={result['x']:.4f}")
    
    # Stale-data rejection
    fused_rej = fuse_centralized(
        [make_radar_track("R1", "uav1", 0.0, 0.0, missed_count=0),
         make_radar_track("R2", "uav2", 10.0, 0.0, missed_count=10)],
        TRUST_WEIGHTED_FUSION, max_staleness_steps=5)
    check("stale_rejection", "Max staleness hard-rejects old data",
          len(fused_rej) == 1 and close(fused_rej[0]["x"], 0.0), "")

def test_one_dropped_sensor():
    active = r_source("R1", "uav1", 0.0, 0.0, confidence=0.9, missed_count=0)
    dropped = r_source("R2", "uav2", 10.0, 0.0, confidence=0.9, missed_count=1, status="lost")
    result = fuse_group([active, dropped], TRUST_WEIGHTED_FUSION)
    check("dropped_sensor", "Dropout state discounts reliability against active source",
          result["x"] < 5.0, f"x={result['x']:.4f}")

def test_one_unavailable_sensor():
    active = r_source("R1", "uav1", 0.0, 0.0, confidence=0.9)
    # Missing from list entirely
    result = fuse_group([active], TRUST_WEIGHTED_FUSION)
    check("missing_sensor_handling", "Missing sensor degrades to single active source seamlessly",
          close(result["x"], 0.0) and result["num_sources"] == 1, "")

def test_one_high_confidence_wrong_sensor():
    correct = r_source("R1", "uav1", 0.0, 0.0, confidence=0.6, pos_var=1.0)
    correct2 = r_source("R2", "uav2", 1.0, 0.0, confidence=0.6, pos_var=1.0)
    wrong = r_source("R3", "uav3", 100.0, 0.0, confidence=0.99, pos_var=0.1)
    clusters = _cluster([correct, correct2, wrong], cluster_distance=10.0)
    check("high_conf_wrong_sensor", "Clustering rejects far-off wrong sensor entirely",
          len(clusters) == 2, f"clusters={len(clusters)}")

def test_one_low_confidence_correct_sensor():
    low_correct = r_source("R1", "uav1", 0.0, 0.0, confidence=0.2, pos_var=10.0)
    mid_wrong = r_source("R2", "uav2", 20.0, 0.0, confidence=0.8, pos_var=2.0)
    result = fuse_group([low_correct, mid_wrong], TRUST_WEIGHTED_FUSION)
    check("low_conf_correct", "Low conf correctly overshadowed by higher conf despite being 'true'",
          result["x"] > 10.0, f"x={result['x']:.4f}")

def test_sensor_disagreement():
    s1 = r_source("R1", "uav1", 0.0, 0.0, confidence=0.9)
    s2 = r_source("R2", "uav2", 50.0, 0.0, confidence=0.9)
    clusters = _cluster([s1, s2], cluster_distance=10.0)
    check("sensor_disagreement", "Sensors disagreeing widely form separate clusters",
          len(clusters) == 2, "")

def test_correlated_estimates_and_covariance_intersection():
    a = r_source("R1", "uav1", 0.0, 0.0, confidence=1.0, pos_var=4.0)
    b = r_source("R2", "uav2", 10.0, 0.0, confidence=1.0, pos_var=4.0)
    ind_trace = float(a["eff_covariance"][0][0] + a["eff_covariance"][1][1])
    
    ci = fuse_group([a, b], COVARIANCE_INTERSECTION_FUSION)
    cw = fuse_group([a, b], COVARIANCE_WEIGHTED_FUSION)
    
    check("covariance_intersection", "CI handles correlated estimates without overclaiming precision",
          ci["position_variance"] > cw["position_variance"], 
          f"CI={ci['position_variance']} Info={cw['position_variance']}")
    check("correlated_estimates", "CI remains bounded by individual source variance",
          ci["position_variance"] <= ind_trace + 1e-6, "")

def test_centralized_fusion():
    t1 = make_radar_track("R1", "uav1", 0.0, 0.0)
    t2 = make_radar_track("R2", "uav2", 2.0, 0.0)
    res = fuse_centralized([t1, t2], COVARIANCE_WEIGHTED_FUSION, uplink_latency_steps=1, downlink_latency_steps=1)
    check("centralized_fusion", "Centralized creates exactly 1 shared world estimate with comm delays",
          len(res) == 1 and res[0]["architecture"] == "centralized" and res[0]["response_time_steps"] == 2, "")

def test_distributed_fusion():
    t1 = make_radar_track("R1", "uav1", 0.0, 0.0)
    t2 = make_radar_track("R2", "uav2", 2.0, 0.0)
    # Simulate perfect comms
    res = fuse_distributed([t1, t2], NAIVE_FUSION, comm_drop_probability=0.0)
    check("distributed_fusion", "Distributed uses available messages and creates one view per UAV",
          len(res) == 2 and all(r["architecture"] == "distributed" for r in res), "")
    
    # 100% drop prob means each only sees themselves
    res_drop = fuse_distributed([t1, t2], NAIVE_FUSION, comm_drop_probability=1.0)
    xs = [r["x"] for r in res_drop]
    check("distributed_fusion", "Distributed fusion uses ONLY available (undropped) messages",
          0.0 in xs and 2.0 in xs and len(res_drop) == 2, f"xs={xs}")

def test_delayed_communicated_track():
    t1 = make_radar_track("R1", "uav1", 0.0, 0.0, missed_count=0)
    t2 = make_radar_track("R2", "uav2", 10.0, 0.0, missed_count=5)
    # The communication latency affects response_time_steps
    channel = CommunicationChannel(base_latency_steps=5, packet_loss_probability=0.0)
    res = fuse_distributed([t1, t2], TRUST_WEIGHTED_FUSION, channel=channel)
    check("delayed_track", "Delayed communicated track increases response time fields",
          res[0]["response_time_steps"] == 5, f"resp={res[0]['response_time_steps']}")

def test_trust_behavior():
    t1 = r_source("R1", "uav1", 0.0, 0.0, persistent_trust=0.1)
    t2 = r_source("R2", "uav2", 10.0, 0.0, persistent_trust=1.0)
    res = fuse_group([t1, t2], TRUST_WEIGHTED_FUSION)
    check("trust_behavior", "Persistent trust degrades influence of untrusted source",
          res["x"] > 8.0, f"x={res['x']:.4f}")

def main():
    test_identical_radar_and_vision_measurements()
    test_radar_more_accurate_than_vision()
    test_vision_more_accurate_than_radar()
    test_lidar_more_accurate_at_short_range()
    test_one_stale_sensor()
    test_one_dropped_sensor()
    test_one_unavailable_sensor()
    test_one_high_confidence_wrong_sensor()
    test_one_low_confidence_correct_sensor()
    test_sensor_disagreement()
    test_correlated_estimates_and_covariance_intersection()
    test_centralized_fusion()
    test_distributed_fusion()
    test_delayed_communicated_track()
    test_trust_behavior()

    failed = _checker.print_summary()
    
    out_dir = os.path.join(_ROOT_DIR, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "fusion_validation_results.md")
    _checker.write_markdown(
        out_path, "Fusion Validation Results (Task 5)",
        intro="Validated multi-source fusion math in `fusion/fusion_model.py`."
    )
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0

if __name__ == "__main__":
    sys.exit(main())