import csv
import json
import math
import random
from simple_swarm_sim import Simulation, dist, clamp


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class VisionLikeModel:
    """Generates vision-style (x/y position, classification) detection rows."""
    
    NOISE_SEED_OFFSET = 99992
    DEFAULT_MAX_RANGE = 12.0
    DEFAULT_FOV_DEG = 80.0
    DEFAULT_PD = 0.92
    DEFAULT_FALSE_POS_RATE = 0.02
    DEFAULT_POSITION_NOISE_STD = 0.5
    DEFAULT_HEADING_NOISE_STD = 0.1

    # Asynchronous update rate (Hz). Vision defaults to updating every
    # simulation step (dt=0.2s -> 5Hz), unlike radar's slower scan rate.
    DEFAULT_UPDATE_RATE = 5.0
    
    ENV_FACTORS = {
        "clear": {"attenuation": 0.0, "noise_mult": 1.0, "pd_mult": 1.0, "fp_mult": 1.0},
        "fog":   {"attenuation": 1.5, "noise_mult": 1.2, "pd_mult": 0.88, "fp_mult": 1.05},
        "rain":  {"attenuation": 2.5, "noise_mult": 1.4, "pd_mult": 0.80, "fp_mult": 1.15},
        "storm": {"attenuation": 4.0, "noise_mult": 1.7, "pd_mult": 0.65, "fp_mult": 1.3},
    }
    
    RELIABILITY_FACTORS = {
        "nominal":  {"noise_mult": 1.0, "pd_mult": 1.0, "fp_mult": 1.0},
        "degraded": {"noise_mult": 1.4, "pd_mult": 0.88, "fp_mult": 1.2},
        "critical": {"noise_mult": 2.0, "pd_mult": 0.70, "fp_mult": 1.5},
    }
    
    def __init__(self, config, scenario_name):
        self.cfg = config
        self.scenario_name = scenario_name
        self.sim = Simulation(config, scenario_name)
        self.dt = config["sim"]["dt"]
        
        base_seed = config.get("sim", {}).get("seed", 0)
        self.vision_rng = random.Random(base_seed + self.NOISE_SEED_OFFSET)
        
        vision_cfg = config.get("vision", {})
        scn = self.sim.scn
        
        self.max_range = scn.get("vision_max_range", 
                                vision_cfg.get("vision_max_range", self.DEFAULT_MAX_RANGE))
        self.fov_deg = scn.get("vision_fov_deg",
                              vision_cfg.get("vision_fov_deg", self.DEFAULT_FOV_DEG))
        self.pd = scn.get("vision_detection_probability",
                         vision_cfg.get("vision_detection_probability", self.DEFAULT_PD))
        self.false_pos_rate = scn.get("vision_false_pos_rate",
                                     vision_cfg.get("vision_false_pos_rate", self.DEFAULT_FALSE_POS_RATE))
        self.position_noise_std = scn.get("vision_position_noise_std",
                                         vision_cfg.get("vision_position_noise_std", self.DEFAULT_POSITION_NOISE_STD))
        self.heading_noise_std = scn.get("vision_heading_noise_std",
                                        vision_cfg.get("vision_heading_noise_std", self.DEFAULT_HEADING_NOISE_STD))
        
        self.environmental_condition = scn.get("vision_environmental_condition",
                                              vision_cfg.get("vision_environmental_condition", "clear"))
        self.reliability_state = scn.get("vision_reliability_state",
                                        vision_cfg.get("vision_reliability_state", "nominal"))
        
        self._env_factors = self.ENV_FACTORS.get(self.environmental_condition, self.ENV_FACTORS["clear"])
        self._reliability_factors = self.RELIABILITY_FACTORS.get(self.reliability_state, self.RELIABILITY_FACTORS["nominal"])

        # Asynchronous update rate: scenario override -> top-level "vision"
        # config section -> built-in default (every step).
        self.update_rate = scn.get("vision_update_rate",
                                  vision_cfg.get("vision_update_rate", self.DEFAULT_UPDATE_RATE))
        self.update_interval_steps = (
            max(1, round(1.0 / (self.update_rate * self.dt))) if self.update_rate > 0 else 1)
        
        self.latency_steps = scn.get("vision_latency_steps",
                                     vision_cfg.get("vision_latency_steps", 0))
        self.dropout_probability = scn.get("vision_dropout_probability",
                                           vision_cfg.get("vision_dropout_probability", 0.0))

        # Per-UAV hold buffer: (last generated step, list of rows produced
        # that step) - re-served on steps that fall between updates.
        self._held_rows = {}
        self._held_step = {}
        
        # Buffer for latency delays
        self._scan_buffer = {i: [] for i in range(self.sim.num_uavs)}

        self.rows = []
    
    def _compute_confidence(self, true_range, snr_factor, lighting_quality):
        """Classification confidence: high in vision, drops with range & lighting."""
        range_factor = max(0.3, 1.0 - (true_range / self.max_range) ** 1.5)
        base_conf = 0.88
        return base_conf * range_factor * lighting_quality * snr_factor
    
    def _compute_covariance(self, true_range):
        """Position covariance (2D, heading uncoupled)."""
        env_mult = self._env_factors["noise_mult"]
        rel_mult = self._reliability_factors["noise_mult"]
        range_factor = 1.0 + (true_range / self.max_range)
        
        pos_var = (self.position_noise_std * env_mult * rel_mult * range_factor) ** 2
        heading_var = (self.heading_noise_std * env_mult * rel_mult) ** 2
        
        return [[pos_var, 0, 0], [0, pos_var, 0], [0, 0, heading_var]]
    
    def _heading(self, uav_id, uav_pos):
        """Camera pointing (target direction from UAV to goal)."""
        gx, gy = self.sim.targets[uav_id]
        dx, dy = gx - uav_pos[0], gy - uav_pos[1]
        return math.atan2(dy, dx) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else 0.0
    
    def _make_vision_row(self, t, uav_id, true_det, measured_x, measured_y,
                        measured_bearing, observer_pos, confidence, covariance,
                        is_valid, is_clutter, timestamp=None, measurement_age_steps=0,
                        is_stale=False):
        """Construct a vision measurement row.

        timestamp is the step this measurement was actually generated
        (differs from `t` on held/re-served steps between updates);
        measurement_age_steps = t - timestamp; is_stale marks a re-served
        (not freshly generated) row, for asynchronous-fusion staleness
        checks downstream."""
        target_id = true_det["id"] if true_det is not None else None
        
        true_x = true_det["x"] if true_det is not None else None
        true_y = true_det["y"] if true_det is not None else None
        true_range = dist(observer_pos, (true_x, true_y)) if true_x is not None else None
        
        cov_json = json.dumps([[round(v, 6) for v in row] for row in covariance]) if covariance else None
        
        # Sensor reliability = measurement quality proxy (confidence-based for vision)
        sensor_reliability = min(confidence, 0.95) if confidence is not None else 0.0
        
        return {
            "time_step": t,
            "vision_id": uav_id,
            "target_id": target_id,
            "true_target_x": round(true_x, 4) if true_x is not None else None,
            "true_target_y": round(true_y, 4) if true_y is not None else None,
            "true_range": round(true_range, 4) if true_range is not None else None,
            "measured_x": round(measured_x, 4) if measured_x is not None else None,
            "measured_y": round(measured_y, 4) if measured_y is not None else None,
            "measured_bearing": round(measured_bearing, 5) if measured_bearing is not None else None,
            "confidence_score": round(confidence, 4) if confidence is not None else None,
            "covariance": cov_json,
            "sensor_reliability": round(sensor_reliability, 4),
            "validity_flag": bool(is_valid),
            "is_clutter": bool(is_clutter),
            "vision_environmental_condition": self.environmental_condition,
            "vision_reliability_state": self.reliability_state,
            "timestamp": timestamp if timestamp is not None else t,
            "measurement_age_steps": measurement_age_steps,
            "is_stale": bool(is_stale),
        }
    
    def run(self):
        """Run simulation and generate vision measurements for all steps."""
        half_fov = math.radians(self.fov_deg / 2.0)
        
        for t in range(self.sim.max_steps):
            self.sim.step(t)
            if all(self.sim.reached_goal):
                break
            
            # Lighting varies per step (affects confidence)
            lighting_quality = 1.0 - 0.3 * (0.3 + 0.7 * self.vision_rng.random())
            
            for uav_id in range(self.sim.num_uavs):
                if self.sim.reached_goal[uav_id]:
                    continue

                if t % self.update_interval_steps == 0:
                    uav_pos = self.sim.pos[uav_id]
                    heading = self._heading(uav_id, uav_pos)
                    true_dets = self.sim._true_detections_for(uav_id)
                    frame_rows = []
                    
                    dropout = self.vision_rng.random() < self.dropout_probability
                    
                    if dropout:
                        # Empty frame due to dropout
                        pass
                    else:
                        # Process real targets in FOV
                        for true_det in true_dets:
                            tx, ty = true_det["x"], true_det["y"]
                            dx, dy = tx - uav_pos[0], ty - uav_pos[1]
                            true_range = math.hypot(dx, dy)
                            
                            # Range gate
                            if true_range > self.max_range or true_range < 0.1:
                                frame_rows.append(self._make_vision_row(
                                    t, uav_id, true_det, None, None, None,
                                    uav_pos, None, None, False, False))
                                continue
                            
                            # FOV check (symmetric around heading)
                            true_bearing = math.atan2(dy, dx)
                            angle_diff = abs(_wrap_angle(true_bearing - heading))
                            if angle_diff > half_fov:
                                frame_rows.append(self._make_vision_row(
                                    t, uav_id, true_det, None, None, None,
                                    uav_pos, None, None, False, False))
                                continue
                            
                            # Occlusion (random)
                            if self.vision_rng.random() < 0.08:
                                frame_rows.append(self._make_vision_row(
                                    t, uav_id, true_det, None, None, None,
                                    uav_pos, None, None, False, False))
                                continue
                            
                            # Detection roll (range-dependent, environment/reliability modulated)
                            range_factor = max(0.3, 1.0 - (true_range / self.max_range) ** 2)
                            snr_factor = range_factor
                            pd_eff = (self.pd * range_factor * lighting_quality *
                                     self._env_factors["pd_mult"] * self._reliability_factors["pd_mult"])
                            
                            if self.vision_rng.random() > pd_eff:
                                frame_rows.append(self._make_vision_row(
                                    t, uav_id, true_det, None, None, None,
                                    uav_pos, None, None, False, False))
                                continue
                            
                            # Measurement (position with noise)
                            meas_x = tx + self.vision_rng.gauss(0, self.position_noise_std)
                            meas_y = ty + self.vision_rng.gauss(0, self.position_noise_std)
                            meas_bearing = math.atan2(dy, dx) + self.vision_rng.gauss(0, self.heading_noise_std)
                            
                            confidence = self._compute_confidence(true_range, snr_factor, lighting_quality)
                            covariance = self._compute_covariance(true_range)
                            
                            frame_rows.append(self._make_vision_row(
                                t, uav_id, true_det, meas_x, meas_y, meas_bearing,
                                uav_pos, confidence, covariance, True, False))
                        
                        # False positives (random detections in FOV)
                        if self.vision_rng.random() < (self.false_pos_rate * 
                                                      self._env_factors["fp_mult"] *
                                                      self._reliability_factors["fp_mult"]):
                            theta = heading + self.vision_rng.uniform(-half_fov, half_fov)
                            r = self.vision_rng.uniform(0.5, self.max_range)
                            fp_x = uav_pos[0] + r * math.cos(theta)
                            fp_y = uav_pos[1] + r * math.sin(theta)
                            fp_bearing = theta
                            
                            covariance = self._compute_covariance(r)
                            frame_rows.append(self._make_vision_row(
                                t, uav_id, None, fp_x, fp_y, fp_bearing,
                                uav_pos, 0.5, covariance, True, True))

                    self._scan_buffer[uav_id].append((t, frame_rows))

                # Process delayed frames
                ready_frame = None
                cutoff = t - self.latency_steps
                
                buf = self._scan_buffer[uav_id]
                idx_to_remove = -1
                for idx, (gen_t, f_rows) in enumerate(buf):
                    if gen_t <= cutoff:
                        ready_frame = (gen_t, f_rows)
                        idx_to_remove = idx
                    else:
                        break
                
                if idx_to_remove >= 0:
                    self._scan_buffer[uav_id] = buf[idx_to_remove+1:]
                
                if ready_frame:
                    gen_t, f_rows = ready_frame
                    self._held_rows[uav_id] = f_rows
                    self._held_step[uav_id] = gen_t
                
                # Report held frame
                held = self._held_rows.get(uav_id)
                if held is not None:
                    gen_t = self._held_step[uav_id]
                    for row in held:
                        out_row = dict(row)
                        out_row["time_step"] = t
                        out_row["timestamp"] = gen_t
                        out_row["measurement_age_steps"] = t - gen_t
                        out_row["is_stale"] = out_row["measurement_age_steps"] > 0
                        self.rows.append(out_row)

        return self.rows


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate vision-like detection logs")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--log", default="logs/vision_log.csv")
    args = parser.parse_args()
    
    with open(args.config) as f:
        config = json.load(f)
    
    scenarios = [args.scenario] if args.scenario else list(config["scenarios"].keys())
    all_rows = []
    
    for name in scenarios:
        model = VisionLikeModel(config, name)
        rows = model.run()
        for row in rows:
            row_with_scenario = {"scenario": name}
            row_with_scenario.update(row)
            all_rows.append(row_with_scenario)
        print(f"{name}: {len(rows)} vision rows")
    
    if all_rows:
        import os
        os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)
        print(f"Wrote {len(all_rows)} rows to {args.log}")