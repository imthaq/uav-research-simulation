import argparse
import csv
import json
import math
import random


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def normalize(vx, vy):
    m = math.hypot(vx, vy)
    if m < 1e-9:
        return 0.0, 0.0
    return vx / m, vy / m


class Perception:
    """Corrupts ground-truth detections into what a UAV actually perceives."""

    def __init__(self, params, rng, sensor_range):
        self.p = params
        self.rng = rng
        self.sensor_range = sensor_range
        self._dropout_remaining = 0  # steps left in an ongoing sensor blackout

        # Diagnostics describing what happened on the most recent process()
        # call, so the simulation can log *why* a detection set looks the
        # way it does (used to populate the "error_type" log column).
        self.last_dropout = False
        self.last_false_positive = False
        self.last_missed_ids = []
        self.last_noise_applied = False

    def _apply_noise(self, d, uav_pos):
        """Jitters a detection's perceived position (and re-derives distance)
        to emulate sensor/ranging noise, e.g. GPS or vision-based localization error."""
        std = self.p.get("position_noise_std", 0.0)
        if std <= 0:
            return d
        nx = d["x"] + self.rng.gauss(0, std)
        ny = d["y"] + self.rng.gauss(0, std)
        noisy = dict(d)
        noisy["x"] = nx
        noisy["y"] = ny
        noisy["distance"] = dist(uav_pos, (nx, ny))
        return noisy

    def _maybe_phantom(self, uav_pos):
        if self.rng.random() >= self.p.get("false_positive_rate", 0.0):
            return None
        angle = self.rng.uniform(0, 2 * math.pi)
        r = self.rng.uniform(2.0, self.sensor_range)
        x = uav_pos[0] + r * math.cos(angle)
        y = uav_pos[1] + r * math.sin(angle)
        return {"kind": "phantom", "id": "phantom", "x": x, "y": y,
                "distance": r, "is_phantom": True}

    def process(self, true_detections, uav_pos):
        # Reset per-step diagnostics.
        self.last_dropout = False
        self.last_false_positive = False
        self.last_missed_ids = []
        self.last_noise_applied = False

        # Sensor dropout: an ongoing or newly-triggered blackout means the
        # sensor delivers nothing at all this step (no real detections, no phantoms).
        if self._dropout_remaining > 0:
            self._dropout_remaining -= 1
            self.last_dropout = True
            return []
        if self.rng.random() < self.p.get("dropout_prob", 0.0):
            self._dropout_remaining = max(self.p.get("dropout_duration_steps", 5) - 1, 0)
            self.last_dropout = True
            return []

        false_negative_rate = self.p.get("false_negative_rate", 0.0)
        perceived = []
        for d in true_detections:
            if self.rng.random() >= false_negative_rate:
                perceived.append(d)
            else:
                self.last_missed_ids.append(d["id"])

        if self.p.get("position_noise_std", 0.0) > 0:
            perceived = [self._apply_noise(d, uav_pos) for d in perceived]
            if perceived:
                self.last_noise_applied = True

        phantom = self._maybe_phantom(uav_pos)
        if phantom is not None:
            perceived.append(phantom)
            self.last_false_positive = True

        return perceived


class Simulation:
    def __init__(self, config, scenario_name):
        self.cfg = config
        self.scenario_name = scenario_name
        self.scn = config["scenarios"][scenario_name]
        self.rng = random.Random(config["sim"]["seed"])

        w = config["world"]
        shared_target = (w["target"]["x"], w["target"]["y"])
        self.obstacle = (w["obstacle"]["x"], w["obstacle"]["y"], w["obstacle"]["radius"])

        s = config["swarm"]
        self.num_uavs = s["num_uavs"]
        self.pos = [list(p) for p in s["start_positions"][: self.num_uavs]]
        self.speed = s["uav_speed"]
        self.formation_spacing = s["desired_formation_spacing"]

        self.dt = config["sim"]["dt"]
        self.max_steps = config["sim"]["max_steps"]

        se = config["sensing"]
        self.sensor_range = se["sensor_range"]
        self.collision_distance = se["collision_distance"]
        self.near_miss_distance = se["near_miss_distance"]
        self.goal_tolerance = se["goal_tolerance"]

        # Each UAV gets its own goal slot arranged around the shared target
        # point instead of all UAVs steering toward one identical coordinate.
        # Converging on a single point causes permanent crowding/orbiting once
        # more than one UAV arrives, since mutual avoidance never lets any of
        # them settle. A small circular formation around the target lets the
        # whole swarm reach "the goal" while keeping safe spacing.
        slot_radius = max(self.formation_spacing / 2.0, self.collision_distance * 1.5)
        self.targets = []
        for i in range(self.num_uavs):
            if self.num_uavs == 1:
                self.targets.append(shared_target)
            else:
                angle = 2 * math.pi * i / self.num_uavs
                self.targets.append((
                    shared_target[0] + slot_radius * math.cos(angle),
                    shared_target[1] + slot_radius * math.sin(angle),
                ))

        c = config["control"]
        self.goal_gain = c["goal_gain"]
        self.avoidance_gain = c["avoidance_gain"]
        # tangential term keeps a UAV sliding around a threat instead of stalling head-on against the goal-seeking vector
        self.tangential_gain = c.get("tangential_gain", 4.0)

        self.perception = [Perception(self.scn, self.rng, self.sensor_range)
                            for _ in range(self.num_uavs)]
        self.reached_goal = [False] * self.num_uavs

        # Latency: detections are generated each step but only become available
        # to the controller `latency_steps` steps later. Each buffer holds
        # (generation_step, perceived_list) entries awaiting delivery.
        self.latency_steps = self.scn.get("latency_steps", 0)
        self.detection_buffer = [[] for _ in range(self.num_uavs)]

        self.collision_count = 0
        self.near_miss_count = 0
        self.unnecessary_avoidance_count = 0
        self.missed_response_count = 0
        self.avoidance_action_count = 0
        self._threat_first_true_step = {}
        self._threat_first_perceived_step = {}
        self.formation_error_samples = []
        self.log_rows = []

    def _true_detections_for(self, i):
        dets = []
        ox, oy, orad = self.obstacle
        d_obs = dist(self.pos[i], (ox, oy)) - orad
        if d_obs <= self.sensor_range:
            dets.append({"kind": "obstacle", "id": "obstacle_0", "x": ox, "y": oy,
                         "distance": max(d_obs, 0.0)})
        for j in range(self.num_uavs):
            if j == i:
                continue
            d_uav = dist(self.pos[i], self.pos[j])
            if d_uav <= self.sensor_range:
                dets.append({"kind": "uav", "id": f"uav_{j}", "x": self.pos[j][0],
                             "y": self.pos[j][1], "distance": d_uav})
        return dets

    def _steer(self, i, perceived):
        tgt = self.targets[i]
        gx, gy = normalize(tgt[0] - self.pos[i][0], tgt[1] - self.pos[i][1])
        vx, vy = gx * self.goal_gain, gy * self.goal_gain

        triggered_real = False
        phantom_only = True

        for d in perceived:
            rx, ry = self.pos[i][0] - d["x"], self.pos[i][1] - d["y"]
            r = max(d["distance"], 0.3)
            if r > self.sensor_range:
                continue
            rx, ry = normalize(rx, ry)
            strength = self.avoidance_gain / r
            tx, ty = -ry, rx
            tangential_strength = self.tangential_gain / r
            vx += rx * strength + tx * tangential_strength
            vy += ry * strength + ty * tangential_strength
            if strength > 0.05 and not d.get("is_phantom"):
                triggered_real = True
                phantom_only = False

        vx, vy = normalize(vx, vy)
        return vx * self.speed, vy * self.speed, len(perceived) > 0, triggered_real, phantom_only

    def _get_delayed_perception(self, i, t):
        """Returns the most recent perceived-detection set that has had time
        to 'arrive' by step t, given self.latency_steps of delay. Consumed
        and stale buffer entries are dropped so the buffer stays bounded."""
        buf = self.detection_buffer[i]
        cutoff = t - self.latency_steps
        used = None
        used_idx = None
        for idx, (gen_t, data) in enumerate(buf):
            if gen_t <= cutoff:
                used = data
                used_idx = idx
            else:
                break
        if used_idx is not None:
            del buf[: used_idx + 1]
        return used if used is not None else []

    def step(self, t):
        new_pos = [list(p) for p in self.pos]
        formation_dists = []
        step_info = [None] * self.num_uavs  # per-UAV data needed for logging, filled in below

        for i in range(self.num_uavs):
            if self.reached_goal[i]:
                step_info[i] = {
                    "perceived": [],
                    "perceived_obstacle": None,
                    "event": "at_goal",
                    "error_type": "none",
                    "unnecessary_avoidance": False,
                    "missed_response": False,
                }
                continue

            true_dets = self._true_detections_for(i)
            raw_perceived = self.perception[i].process(true_dets, tuple(self.pos[i]))

            # Sensor output is generated now but only "arrives" at the controller
            # after latency_steps (0 latency behaves exactly as before).
            self.detection_buffer[i].append((t, raw_perceived))
            perceived = self._get_delayed_perception(i, t)

            for d in true_dets:
                if d["distance"] <= self.near_miss_distance:
                    key = (i, d["id"])
                    if key not in self._threat_first_true_step:
                        self._threat_first_true_step[key] = t
            perceived_ids = {d["id"] for d in perceived if not d.get("is_phantom")}
            for pid in perceived_ids:
                key = (i, pid)
                if key in self._threat_first_true_step and key not in self._threat_first_perceived_step:
                    self._threat_first_perceived_step[key] = t

            vx, vy, any_det, triggered_real, phantom_only = self._steer(i, perceived)

            unnecessary_avoidance = any_det and phantom_only
            if unnecessary_avoidance:
                self.unnecessary_avoidance_count += 1
            if triggered_real:
                self.avoidance_action_count += 1

            missed_response = False
            for d in true_dets:
                if d["distance"] <= self.near_miss_distance and d["id"] not in perceived_ids:
                    missed_response = True
                    self.missed_response_count += 1

            new_pos[i][0] = min(max(new_pos[i][0] + vx * self.dt, 0.0), self.cfg["world"]["width"])
            new_pos[i][1] = min(max(new_pos[i][1] + vy * self.dt, 0.0), self.cfg["world"]["height"])

            event = "avoidance" if triggered_real else ("false_avoidance" if any_det and phantom_only else "move")

            # Which perception-error mechanism(s) fired for this UAV this step
            # (based on the freshly generated sensor reading, not the delayed
            # one the controller acts on).
            perc = self.perception[i]
            errors = []
            if perc.last_dropout:
                errors.append("dropout")
            if perc.last_false_positive:
                errors.append("false_positive")
            if perc.last_missed_ids:
                errors.append("false_negative")
            if perc.last_noise_applied:
                errors.append("position_noise")
            if self.latency_steps > 0:
                errors.append("latency")
            error_type = "+".join(errors) if errors else "none"

            perceived_obstacle = next((d for d in perceived if d.get("id") == "obstacle_0"), None)

            step_info[i] = {
                "perceived": perceived,
                "perceived_obstacle": perceived_obstacle,
                "event": event,
                "error_type": error_type,
                "unnecessary_avoidance": unnecessary_avoidance,
                "missed_response": missed_response,
            }

        ox, oy, orad = self.obstacle
        for i in range(self.num_uavs):
            d_obs = dist(new_pos[i], (ox, oy)) - orad
            if d_obs <= self.collision_distance:
                self.collision_count += 1
            elif d_obs <= self.near_miss_distance:
                self.near_miss_count += 1
            for j in range(i + 1, self.num_uavs):
                d_uav = dist(new_pos[i], new_pos[j])
                if d_uav <= self.collision_distance:
                    self.collision_count += 1
                elif d_uav <= self.near_miss_distance:
                    self.near_miss_count += 1
                formation_dists.append(d_uav)

        # Distance/identity of the single closest entity (obstacle or other
        # UAV) to each UAV, at the post-move positions. Read-only pass, kept
        # separate from the counting loop above so collision/near-miss totals
        # are unaffected.
        nearest_info = [None] * self.num_uavs
        for i in range(self.num_uavs):
            nearest_type = "obstacle"
            nearest_dist = dist(new_pos[i], (ox, oy)) - orad
            for j in range(self.num_uavs):
                if j == i:
                    continue
                d_uav = dist(new_pos[i], new_pos[j])
                if d_uav < nearest_dist:
                    nearest_dist = d_uav
                    nearest_type = f"uav_{j}"
            nearest_info[i] = (nearest_type, nearest_dist)

        if formation_dists:
            rmse = math.sqrt(sum((fd - self.formation_spacing) ** 2 for fd in formation_dists) / len(formation_dists))
            self.formation_error_samples.append(rmse)

        self.pos = new_pos
        for i in range(self.num_uavs):
            if not self.reached_goal[i] and dist(self.pos[i], self.targets[i]) <= self.goal_tolerance:
                self.reached_goal[i] = True

        mission_completed = bool(all(self.reached_goal) and self.collision_count == 0)

        for i in range(self.num_uavs):
            self.log_rows.append(self._log_row(t, i, step_info[i], nearest_info[i], mission_completed))

    def _log_row(self, t, i, info, nearest, mission_completed):
        perceived = info["perceived"]
        perceived_obstacle = info["perceived_obstacle"]
        ox, oy, _ = self.obstacle
        tx, ty = self.targets[i]
        nearest_type, nearest_dist = nearest

        return {
            "scenario": self.scenario_name,
            "step": t,
            "time_s": round(t * self.dt, 3),
            "uav_id": i,
            "x": round(self.pos[i][0], 3),
            "y": round(self.pos[i][1], 3),

            # Perceived vs. actual obstacle position (perceived is None if the
            # obstacle wasn't detected this step, e.g. dropout/false negative/
            # out of range; otherwise it reflects any position noise applied).
            "actual_obstacle_x": round(ox, 3),
            "actual_obstacle_y": round(oy, 3),
            "perceived_obstacle_x": round(perceived_obstacle["x"], 3) if perceived_obstacle else None,
            "perceived_obstacle_y": round(perceived_obstacle["y"], 3) if perceived_obstacle else None,

            # The target/goal is assigned, not sensed, so it's always known
            # exactly - perceived and actual coincide by construction.
            "actual_target_x": round(tx, 3),
            "actual_target_y": round(ty, 3),
            "perceived_target_x": round(tx, 3),
            "perceived_target_y": round(ty, 3),

            "error_type": info["error_type"],
            "action_taken": info["event"],

            "num_perceived_detections": len(perceived),
            "num_phantom_detections": sum(1 for d in perceived if d.get("is_phantom")),
            "dist_to_target": round(dist(self.pos[i], self.targets[i]), 3),

            "nearest_entity_type": nearest_type,
            "nearest_entity_distance": round(nearest_dist, 3),
            "collision_risk_flag": bool(nearest_dist <= self.near_miss_distance),

            "unnecessary_avoidance_flag": bool(info["unnecessary_avoidance"]),
            "missed_response_flag": bool(info["missed_response"]),

            "reached_goal": self.reached_goal[i],
            "mission_completed_flag": mission_completed,
        }

    def run(self):
        t = 0
        for t in range(self.max_steps):
            self.step(t)
            if all(self.reached_goal):
                break
        return self._metrics(t)

    def _metrics(self, final_step):
        num_reached = sum(self.reached_goal)
        response_times = []
        for key, t_true in self._threat_first_true_step.items():
            t_perc = self._threat_first_perceived_step.get(key)
            if t_perc is not None:
                response_times.append((t_perc - t_true) * self.dt)
        avg_response_time = sum(response_times) / len(response_times) if response_times else None
        avg_formation_error = (sum(self.formation_error_samples) / len(self.formation_error_samples)
                                if self.formation_error_samples else None)
        return {
            "scenario": self.scenario_name,
            "steps_run": final_step + 1,
            "uavs_reached_goal": num_reached,
            "num_uavs": self.num_uavs,
            "mission_success": bool(num_reached == self.num_uavs and self.collision_count == 0),
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            "unnecessary_avoidance_count": self.unnecessary_avoidance_count,
            "missed_response_count": self.missed_response_count,
            "avoidance_action_count": self.avoidance_action_count,
            "avg_response_time_s": round(avg_response_time, 3) if avg_response_time is not None else None,
            "avg_formation_error": round(avg_formation_error, 3) if avg_formation_error is not None else None,
        }


def main():
    parser = argparse.ArgumentParser(description="2D UAV swarm perception-error simulation")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default="simulation_log.csv")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        sim = Simulation(config, name)
        metrics = sim.run()
        all_rows.extend(sim.log_rows)
        print(json.dumps(metrics, indent=2))

    if all_rows:
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)


if __name__ == "__main__":
    main()