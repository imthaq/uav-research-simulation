"""
lidar_like_model.py
LiDAR sensor: accurate position/range, shorter range, dropout in adverse weather,
limited FOV if configured.
"""

import csv
import json
import math
import random
from simple_swarm_sim import Simulation, dist, clamp


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class LiDARLikeModel:
    """Generates LiDAR-style (x/y/range, accurate position) detection rows."""
    
    NOISE_SEED_OFFSET = 99993
    DEFAULT_MAX_RANGE = 8.0
    DEFAULT_MIN_RANGE = 0.1
    DEFAULT_FOV_DEG = 120.0
    DEFAULT_PD = 0.97
    DEFAULT_RANGE_NOISE_STD = 0.08
    DEFAULT_POSITION_NOISE_STD = 0.15
    DEFAULT_ANGULAR_NOISE_STD = 0.03
    
    # Weather heavily impacts LiDAR (rain/snow dropout)
    ENV_FACTORS = {
        "clear": {"dropout_add": 0.0, "noise_mult": 1.0, "pd_mult": 1.0},
        "fog":   {"dropout_add": 0.15, "noise_mult": 1.3, "pd_mult": 0.92},
        "rain":  {"dropout_add": 0.45, "noise_mult": 1.8, "pd_mult": 0.70},
        "storm": {"dropout_add": 0.70, "noise_mult": 2.2, "pd_mult": 0.45},
    }
    
    RELIABILITY_FACTORS = {
        "nominal":  {"dropout_add": 0.0, "noise_mult": 1.0, "pd_mult": 1.0},
        "degraded": {"dropout_add": 0.15, "noise_mult": 1.5, "pd_mult": 0.90},
        "critical": {"dropout_add": 0.40, "noise_mult": 2.2, "pd_mult": 0.70},
    }
    
    def __init__(self, config, scenario_name):
        self.cfg = config
        self.scenario_name = scenario_name
        self.sim = Simulation(config, scenario_name)
        self.dt = config["sim"]["dt"]
        
        base_seed = config.get("sim", {}).get("seed", 0)
        self.lidar_rng = random.Random(base_seed + self.NOISE_SEED_OFFSET)
        
        lidar_cfg = config.get("lidar", {})
        scn = self.sim.scn
        
        self.max_range = scn.get("lidar_max_range",
                                lidar_cfg.get("lidar_max_range", self.DEFAULT_MAX_RANGE))
        self.min_range = scn.get("lidar_min_range",
                                lidar_cfg.get("lidar_min_range", self.DEFAULT_MIN_RANGE))
        self.fov_deg = scn.get("lidar_fov_deg",
                              lidar_cfg.get("lidar_fov_deg", self.DEFAULT_FOV_DEG))
        self.pd = scn.get("lidar_detection_probability",
                         lidar_cfg.get("lidar_detection_probability", self.DEFAULT_PD))
        self.range_noise_std = scn.get("lidar_range_noise_std",
                                      lidar_cfg.get("lidar_range_noise_std", self.DEFAULT_RANGE_NOISE_STD))
        self.position_noise_std = scn.get("lidar_position_noise_std",
                                         lidar_cfg.get("lidar_position_noise_std", self.DEFAULT_POSITION_NOISE_STD))
        self.angular_noise_std = scn.get("lidar_angular_noise_std",
                                        lidar_cfg.get("lidar_angular_noise_std", self.DEFAULT_ANGULAR_NOISE_STD))
        
        self.environmental_condition = scn.get("lidar_environmental_condition",
                                              lidar_cfg.get("lidar_environmental_condition", "clear"))
        self.reliability_state = scn.get("lidar_reliability_state",
                                        lidar_cfg.get("lidar_reliability_state", "nominal"))
        
        self._env_factors = self.ENV_FACTORS.get(self.environmental_condition, self.ENV_FACTORS["clear"])
        self._reliability_factors = self.RELIABILITY_FACTORS.get(self.reliability_state, self.RELIABILITY_FACTORS["nominal"])
        
        self.rows = []
    
    def _compute_covariance(self, true_range):
        """Range + position covariance (accurate for LiDAR)."""
        env_mult = self._env_factors["noise_mult"]
        rel_mult = self._reliability_factors["noise_mult"]
        
        range_var = (self.range_noise_std * env_mult * rel_mult) ** 2
        pos_var = (self.position_noise_std * env_mult * rel_mult) ** 2
        ang_var = (self.angular_noise_std * env_mult * rel_mult) ** 2
        
        # Covariance: [range_var, position_var_xy, angular_var]
        return [[range_var, 0, 0], [0, pos_var, 0], [0, 0, ang_var]]
    
    def _heading(self, uav_id, uav_pos):
        """LiDAR pointing direction (aligned with UAV goal direction)."""
        gx, gy = self.sim.targets[uav_id]
        dx, dy = gx - uav_pos[0], gy - uav_pos[1]
        return math.atan2(dy, dx) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else 0.0
    
    def _make_lidar_row(self, t, uav_id, true_det, measured_x, measured_y, 
                       measured_range, measured_bearing, observer_pos,
                       confidence, covariance, is_valid, dropout_reason=None):
        """Construct a LiDAR measurement row."""
        target_id = true_det["id"] if true_det is not None else None
        
        true_x = true_det["x"] if true_det is not None else None
        true_y = true_det["y"] if true_det is not None else None
        true_range = dist(observer_pos, (true_x, true_y)) if true_x is not None else None
        
        cov_json = json.dumps([[round(v, 8) for v in row] for row in covariance]) if covariance else None
        
        # Sensor reliability for LiDAR: affected by environment/dropout
        sensor_reliability = (1.0 - self._env_factors["dropout_add"]) * (1.0 - self._reliability_factors["dropout_add"])
        if not is_valid:
            sensor_reliability *= 0.2
        
        return {
            "time_step": t,
            "lidar_id": uav_id,
            "target_id": target_id,
            "true_target_x": round(true_x, 4) if true_x is not None else None,
            "true_target_y": round(true_y, 4) if true_y is not None else None,
            "true_range": round(true_range, 4) if true_range is not None else None,
            "measured_x": round(measured_x, 4) if measured_x is not None else None,
            "measured_y": round(measured_y, 4) if measured_y is not None else None,
            "measured_range": round(measured_range, 4) if measured_range is not None else None,
            "measured_bearing": round(measured_bearing, 5) if measured_bearing is not None else None,
            "confidence_score": round(confidence, 4) if confidence is not None else None,
            "covariance": cov_json,
            "sensor_reliability": round(sensor_reliability, 4),
            "validity_flag": bool(is_valid),
            "dropout_reason": dropout_reason,
            "lidar_environmental_condition": self.environmental_condition,
            "lidar_reliability_state": self.reliability_state,
        }
    
    def run(self):
        """Run simulation and generate LiDAR measurements."""
        half_fov = math.radians(self.fov_deg / 2.0) if self.fov_deg < 360 else math.pi
        
        for t in range(self.sim.max_steps):
            self.sim.step(t)
            if all(self.sim.reached_goal):
                break
            
            # Weather-induced dropout (environment + reliability stack)
            dropout_prob = (self._env_factors["dropout_add"] + 
                           self._reliability_factors["dropout_add"])
            weather_dropout = self.lidar_rng.random() < dropout_prob
            
            for uav_id in range(self.sim.num_uavs):
                if self.sim.reached_goal[uav_id]:
                    continue
                
                uav_pos = self.sim.pos[uav_id]
                heading = self._heading(uav_id, uav_pos)
                
                # Get true detections
                true_dets = self.sim._true_detections_for(uav_id)
                
                # Check for scan-level dropout first (weather or hardware)
                if weather_dropout:
                    # Report all true targets as undetected due to weather
                    for true_det in true_dets:
                        self.rows.append(self._make_lidar_row(
                            t, uav_id, true_det, None, None, None, None,
                            uav_pos, None, None, False, "weather_dropout"))
                    continue
                
                # Process real targets in range/FOV
                for true_det in true_dets:
                    tx, ty = true_det["x"], true_det["y"]
                    dx, dy = tx - uav_pos[0], ty - uav_pos[1]
                    true_range = math.hypot(dx, dy)
                    
                    # Range gate (LiDAR has both min and max range)
                    if true_range > self.max_range or true_range < self.min_range:
                        self.rows.append(self._make_lidar_row(
                            t, uav_id, true_det, None, None, None, None,
                            uav_pos, None, None, False, "range_gate"))
                        continue
                    
                    # FOV check (if configured)
                    if self.fov_deg < 360:
                        true_bearing = math.atan2(dy, dx)
                        angle_diff = abs(_wrap_angle(true_bearing - heading))
                        if angle_diff > half_fov:
                            self.rows.append(self._make_lidar_row(
                                t, uav_id, true_det, None, None, None, None,
                                uav_pos, None, None, False, "fov_gate"))
                            continue
                    
                    # Detection roll (range-independent but environment/reliability modulated)
                    pd_eff = (self.pd * 
                             self._env_factors["pd_mult"] * 
                             self._reliability_factors["pd_mult"])
                    
                    if self.lidar_rng.random() > pd_eff:
                        self.rows.append(self._make_lidar_row(
                            t, uav_id, true_det, None, None, None, None,
                            uav_pos, None, None, False, "detection_miss"))
                        continue
                    
                    # Measurement: range + bearing (converted to x/y)
                    true_bearing = math.atan2(dy, dx)
                    
                    # Noise: range and bearing independently
                    meas_range = max(self.min_range, 
                                    true_range + self.lidar_rng.gauss(0, self.range_noise_std))
                    meas_bearing = true_bearing + self.lidar_rng.gauss(0, self.angular_noise_std)
                    
                    # Convert to x/y
                    meas_x = uav_pos[0] + meas_range * math.cos(meas_bearing)
                    meas_y = uav_pos[1] + meas_range * math.sin(meas_bearing)
                    
                    # Add position noise independently (LiDAR gives accurate range + position)
                    meas_x += self.lidar_rng.gauss(0, self.position_noise_std)
                    meas_y += self.lidar_rng.gauss(0, self.position_noise_std)
                    
                    # High confidence for LiDAR (range-independent)
                    confidence = 0.94 * pd_eff
                    covariance = self._compute_covariance(true_range)
                    
                    self.rows.append(self._make_lidar_row(
                        t, uav_id, true_det, meas_x, meas_y, meas_range, meas_bearing,
                        uav_pos, confidence, covariance, True))
        
        return self.rows


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate LiDAR-like detection logs")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--log", default="logs/lidar_log.csv")
    args = parser.parse_args()
    
    with open(args.config) as f:
        config = json.load(f)
    
    scenarios = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    all_rows = []
    
    for name in scenarios:
        model = LiDARLikeModel(config, name)
        rows = model.run()
        for row in rows:
            row_with_scenario = {"scenario": name}
            row_with_scenario.update(row)
            all_rows.append(row_with_scenario)
        print(f"{name}: {len(rows)} LiDAR rows")
    
    if all_rows:
        import os
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote {len(all_rows)} rows to {args.log}")
