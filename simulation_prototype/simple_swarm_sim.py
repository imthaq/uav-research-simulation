import argparse
import csv
import json
import math
import random

from perception_quality_monitor import PerceptionQualityMonitor, GOOD, DEGRADED, CRITICAL


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def normalize(vx, vy):
    m = math.hypot(vx, vy)
    if m < 1e-9:
        return 0.0, 0.0
    return vx / m, vy / m


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _covariance_trace(track_row):
    """Sum of the Kalman filter's diagonal state-covariance terms (pos_x,
    pos_y, vel_x, vel_y uncertainty) for one track row, or None if no
    track is active this step."""
    if not track_row or not track_row.get("covariance"):
        return None
    mat = json.loads(track_row["covariance"])
    return round(sum(mat[i][i] for i in range(len(mat))), 4)


class Perception:
    """Corrupts ground-truth detections into what a UAV actually perceives,
    and attaches a (possibly miscalibrated) confidence value to each one."""

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

    def _confidence_for(self, distance, is_phantom, noisy):
        """True (pre-miscalibration) confidence a sensor would report for a
        detection: higher for close, clean detections; a flat, moderately
        high value for phantoms (a false detection typically still LOOKS
        clean to the sensor that produced it - that's what makes phantom
        detections dangerous to trust)."""
        if is_phantom:
            base = 0.6
        else:
            base = 1.0 - (distance / self.sensor_range) * 0.6
            if noisy:
                base *= 0.8
        base = clamp(base, 0.05, 1.0)

        # confidence_error_level corrupts the *reported* confidence away from
        # the true value, i.e. the sensor's self-assessment of its own
        # reliability is itself unreliable.
        true_conf = round(base, 3)
        err = self.p.get("confidence_error_level", 0.0)
        reported = base
        if err > 0:
            reported += self.rng.gauss(0, err)
        reported = round(clamp(reported, 0.0, 1.0), 3)
        return reported, true_conf

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

        noise_on = self.p.get("position_noise_std", 0.0) > 0
        if noise_on:
            perceived = [self._apply_noise(d, uav_pos) for d in perceived]
            if perceived:
                self.last_noise_applied = True

        phantom = self._maybe_phantom(uav_pos)
        if phantom is not None:
            perceived.append(phantom)
            self.last_false_positive = True

        for d in perceived:
            reported_conf, true_conf = self._confidence_for(d["distance"], d.get("is_phantom", False), noise_on)
            d["confidence"] = reported_conf
            d["true_confidence"] = true_conf

        return perceived


def fuse_obstacle_detections(contributions, fusion_mode):
    """Combines each UAV's own perceived obstacle detection (if any) into a
    single shared belief about where the obstacle actually is.

    contributions: list of (uav_id, x, y, confidence) tuples - one per UAV
    that currently perceives the obstacle (real detection, not a phantom).

    Returns None if fusion doesn't apply or nothing to fuse, else a dict
    with the fused x/y/confidence and the list of contributing UAV ids.
    """
    if fusion_mode == "no_fusion" or not contributions:
        return None

    if fusion_mode == "trust_weighted_fusion":
        total_w = sum(c[3] for c in contributions)
        if total_w <= 1e-9:
            fx = sum(c[1] for c in contributions) / len(contributions)
            fy = sum(c[2] for c in contributions) / len(contributions)
        else:
            fx = sum(c[1] * c[3] for c in contributions) / total_w
            fy = sum(c[2] * c[3] for c in contributions) / total_w
    else:  # naive_fusion: unweighted average, ignores confidence entirely
        n = len(contributions)
        fx = sum(c[1] for c in contributions) / n
        fy = sum(c[2] for c in contributions) / n

    # More independent UAVs agreeing raises fused confidence a bit
    # (mirrors how corroborating sources reduce uncertainty), capped at 1.0.
    avg_conf = sum(c[3] for c in contributions) / len(contributions)
    fused_conf = clamp(avg_conf + 0.08 * (len(contributions) - 1), 0.0, 1.0)

    return {
        "x": fx,
        "y": fy,
        "confidence": round(fused_conf, 3),
        "contributors": [c[0] for c in contributions],
    }


class Simulation:
    def __init__(self, config, scenario_name):
        self.cfg = config
        self.scenario_name = scenario_name
        self.scn = config["scenarios"][scenario_name]
        self.rng = random.Random(config["sim"]["seed"])

        w = config["world"]
        shared_target = (w["target"]["x"], w["target"]["y"])

        # --- Multi-entity world state (Task 7: multiple moving targets,
        # multiple static/moving obstacles, crossing paths, entities
        # entering/leaving radar coverage, appearance/disappearance) ---
        #
        # An "entity" is anything sensed that isn't a UAV: a static
        # obstacle, a moving obstacle, or a moving target. Each is a plain
        # dict with a stable id/kind, a position, an optional constant
        # velocity or waypoint patrol path, and an optional
        # appear_step/disappear_step window controlling when it exists in
        # the world at all (distinct from simply being out of sensor
        # range - a not-yet-appeared or already-disappeared entity is
        # invisible to every detection mechanism, not just far away).
        #
        # Config precedence (scenario-first, matching every other
        # scenario-overridable value in this project): scenario["entities"]
        # -> world["entities"] -> a single legacy static obstacle built
        # from world["obstacle"], so every existing config/scenario that
        # predates this feature keeps behaving exactly as before.
        entities_cfg = self.scn.get("entities", w.get("entities"))
        if entities_cfg is None:
            entities_cfg = [{
                "id": "obstacle_0", "kind": "static_obstacle",
                "x": w["obstacle"]["x"], "y": w["obstacle"]["y"],
                "radius": w["obstacle"]["radius"],
            }]

        self.entities = []
        for idx, e in enumerate(entities_cfg):
            vel = e.get("velocity", [0.0, 0.0])
            self.entities.append({
                "id": e.get("id", f"entity_{idx}"),
                "kind": e.get("kind", "static_obstacle"),  # static_obstacle | moving_obstacle | moving_target
                "x": float(e["x"]), "y": float(e["y"]),
                "radius": float(e.get("radius", 0.0)),
                "vx": float(vel[0]), "vy": float(vel[1]),
                # Optional patrol path: a list of [x, y] waypoints the
                # entity drives toward in order, looping back to the
                # first once the last is reached. When set, this
                # overrides the constant vx/vy above (speed still comes
                # from vx/vy's magnitude, or "speed" if given directly).
                "waypoints": e.get("waypoints"),
                "waypoint_idx": 0,
                "speed": float(e.get("speed", math.hypot(vel[0], vel[1]))),
                # Steps are in units of sim steps (t), matching
                # appear_step/disappear_step elsewhere in this project
                # (e.g. dropout_duration_steps) rather than seconds.
                "appear_step": int(e.get("appear_step", 0)),
                "disappear_step": e.get("disappear_step"),  # None = never
                # Whether a moving entity with no waypoints bounces off
                # the world bounds (reflecting vx/vy) instead of drifting
                # off the edge and getting clamped in place.
                "bounce": bool(e.get("bounce", True)),
            })

        # self.obstacle stays a backward-compatible (x, y, radius) view of
        # the "primary" obstacle - the first static_obstacle/
        # moving_obstacle entity, or the legacy world["obstacle"] if none
        # is defined - since a lot of existing code (logging, the
        # radar/track/fusion demo pipeline) reads sim.obstacle directly.
        # It's refreshed every step in _advance_entities() as that entity
        # moves, so it always reflects current position for a moving
        # primary obstacle.
        self._primary_obstacle_id = next(
            (e["id"] for e in self.entities if e["kind"] in ("static_obstacle", "moving_obstacle")),
            None)
        self.obstacle = self._obstacle_view()

        # Per-entity radius lookup, used by fusion (surface-distance to a
        # fused/estimated center needs the same radius true detections use).
        self._entity_radius = {e["id"]: e["radius"] for e in self.entities}

        self._current_t = 0

        s = config["swarm"]
        self.num_uavs = s["num_uavs"]
        self.pos = [list(p) for p in s["start_positions"][: self.num_uavs]]
        self.speed = s["uav_speed"]
        self.formation_spacing = s["desired_formation_spacing"]
        self.safety_distance = s.get("safety_distance", 2.0)

        self.dt = config["sim"]["dt"]
        self.max_steps = config["sim"]["max_steps"]

        se = config["sensing"]
        self.sensor_range = se["sensor_range"]
        self.collision_distance = se["collision_distance"]
        self.near_miss_distance = se["near_miss_distance"]
        self.goal_tolerance = se["goal_tolerance"]

        # Range at which another UAV (not the obstacle) starts contributing a
        # repulsive steering force. This used to be self.sensor_range, i.e.
        # any teammate detected anywhere within the full sensor footprint
        # (up to 15 units away) pushed back - which meant UAVs converging on
        # goal slots only ~5-8 units apart (see slot_radius below) permanently
        # repelled each other and could never settle, even with zero
        # perception error. Reactive avoidance should only kick in once a
        # teammate is actually close enough to be a safety concern, so this
        # is tied to near_miss_distance/safety_distance instead. This also
        # gives self.safety_distance (previously read from config and never
        # referenced again) an actual effect on behavior.
        self.uav_avoidance_range = max(self.near_miss_distance, self.safety_distance * 1.5)

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

        # --- Task 13: uncertainty-aware safety margins ---
        #
        # How much extra separation a UAV keeps from a perceived contact
        # on top of the raw sensed distance, and how that extra margin is
        # derived. Scenario-overridable (same precedence as fusion_mode)
        # so different scenarios can be compared side by side:
        #   "fixed"           - always base_safety_margin, regardless of
        #                        how (un)trustworthy the detection is.
        #   "covariance"      - margin grows with sqrt(position
        #                        covariance/variance) attached to the
        #                        detection (e.g. fusion_position_variance
        #                        from the radar/track/fusion pipeline).
        #                        No covariance available -> treated as
        #                        maximally uncertain, not silently as
        #                        certain.
        #   "confidence"      - margin grows with (1 - reported
        #                        confidence).
        #   "quality_monitor" - margin is driven by the Task 12
        #                        PerceptionQualityMonitor's GOOD/
        #                        DEGRADED/CRITICAL verdict for the
        #                        detection: GOOD keeps the base margin,
        #                        DEGRADED scales the margin by how far
        #                        below "good" the composite quality score
        #                        is, and CRITICAL discards the avoidance
        #                        geometry entirely in favor of
        #                        critical_quality_action.
        self.safety_margin_mode = self.scn.get(
            "safety_margin_mode", c.get("safety_margin_mode", "fixed"))
        self.base_safety_margin = c.get("base_safety_margin", 0.0)
        self.uncertainty_margin_gain = c.get("uncertainty_margin_gain", 1.0)
        self.maximum_safety_margin = c.get("maximum_safety_margin", 5.0)
        # What a CRITICAL quality_monitor verdict does instead of normal
        # avoidance steering: "hold" (stop in place), "regroup" (head
        # toward the swarm centroid at reduced speed), or
        # "request_fresh_observation" (crawl toward the goal at greatly
        # reduced speed instead of acting on distrusted geometry).
        self.critical_quality_action = c.get("critical_quality_action", "hold")
        self.quality_monitor = PerceptionQualityMonitor()

        self.hold_count = 0
        self.regroup_count = 0
        self.request_fresh_observation_count = 0

        self.perception = [Perception(self.scn, self.rng, self.sensor_range)
                            for _ in range(self.num_uavs)]
        self.reached_goal = [False] * self.num_uavs

        # Sensor fusion: "no_fusion" (each UAV uses only its own perception),
        # "naive_fusion" (unweighted average of every UAV's obstacle detection
        # this step), or "trust_weighted_fusion" (same, weighted by each
        # detection's confidence). Fusion only combines *real* obstacle
        # detections across UAVs; phantom (false-positive) detections are
        # never fused since each is a distinct, uncorroborated ghost.
        self.fusion_mode = self.scn.get("fusion_mode", "no_fusion")

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
        self.fusion_recovery_count = 0  # steps where fusion supplied an obstacle detection a UAV had individually missed
        self._threat_first_true_step = {}
        self._threat_first_perceived_step = {}
        self.formation_error_samples = []
        self.confidence_error_samples = []
        self.log_rows = []

    def _obstacle_view(self):
        """Backward-compatible (x, y, radius) tuple for the primary
        obstacle entity, or the zero-radius origin if none exists (an
        all-targets, no-obstacle world is valid)."""
        for e in self.entities:
            if e["id"] == self._primary_obstacle_id:
                return (e["x"], e["y"], e["radius"])
        return (0.0, 0.0, 0.0)

    def _entity_active(self, entity, t):
        """An entity exists in the world (is sensable/collidable at all)
        only once t >= appear_step and, if disappear_step is set, only
        while t < disappear_step - modeling genuine appearance/
        disappearance, not just moving out of sensor range."""
        if t < entity["appear_step"]:
            return False
        if entity["disappear_step"] is not None and t >= entity["disappear_step"]:
            return False
        return True

    def _advance_entities(self, t):
        """Moves every active moving_obstacle/moving_target entity one
        step (waypoint patrol if given, else straight-line constant
        velocity that bounces off the world bounds), then refreshes
        self.obstacle so backward-compatible consumers see the primary
        obstacle's current position. Static obstacles and inactive
        (not-yet-appeared/already-disappeared) entities are left alone."""
        width = self.cfg["world"]["width"]
        height = self.cfg["world"]["height"]

        for e in self.entities:
            if e["kind"] not in ("moving_obstacle", "moving_target"):
                continue
            if not self._entity_active(e, t):
                continue

            if e["waypoints"]:
                wp = e["waypoints"][e["waypoint_idx"] % len(e["waypoints"])]
                dx, dy = wp[0] - e["x"], wp[1] - e["y"]
                step_dist = e["speed"] * self.dt
                remaining = math.hypot(dx, dy)
                if remaining <= max(step_dist, 1e-6):
                    e["x"], e["y"] = wp[0], wp[1]
                    e["waypoint_idx"] += 1
                else:
                    ux, uy = normalize(dx, dy)
                    e["x"] += ux * step_dist
                    e["y"] += uy * step_dist
                continue

            nx = e["x"] + e["vx"] * self.dt
            ny = e["y"] + e["vy"] * self.dt
            if e["bounce"]:
                if nx < 0.0 or nx > width:
                    e["vx"] *= -1.0
                    nx = clamp(nx, 0.0, width)
                if ny < 0.0 or ny > height:
                    e["vy"] *= -1.0
                    ny = clamp(ny, 0.0, height)
            else:
                nx = clamp(nx, 0.0, width)
                ny = clamp(ny, 0.0, height)
            e["x"], e["y"] = nx, ny

        self.obstacle = self._obstacle_view()

    def _true_detections_for(self, i):
        dets = []
        for e in self.entities:
            if not self._entity_active(e, self._current_t):
                continue  # not yet appeared / already disappeared this step
            d_ent = dist(self.pos[i], (e["x"], e["y"])) - e["radius"]
            if d_ent <= self.sensor_range:
                dets.append({"kind": e["kind"], "id": e["id"], "x": e["x"], "y": e["y"],
                             "distance": max(d_ent, 0.0)})
        for j in range(self.num_uavs):
            if j == i:
                continue
            d_uav = dist(self.pos[i], (self.pos[j]))
            if d_uav <= self.sensor_range:
                dets.append({"kind": "uav", "id": f"uav_{j}", "x": self.pos[j][0],
                             "y": self.pos[j][1], "distance": d_uav,
                             "is_parked": self.reached_goal[j]})
        return dets

    def _apply_fusion(self, raw_percepts):
        """Cross-UAV fusion step. Given each UAV's individually-perceived
        detection list (already corrupted by its own Perception), builds a
        fused position estimate PER ENTITY ID from every UAV that
        currently has a real (non-phantom) detection of that entity, then
        - if fusion is enabled - overwrites/injects each fused estimate
        into every UAV's own perceived list (recomputing distance/
        confidence per UAV). This generalizes the original single-obstacle
        fusion to however many obstacles/targets are in the world
        (self.entities): each is fused independently, since two different
        entities happening to be visible at once are not evidence about
        each other. This is how fusion can "recover" a detection an
        individual UAV's sensor missed via false_negative/dropout, as long
        as at least one other UAV still saw that same entity."""
        contributions_by_id = {}
        for i, dets in raw_percepts.items():
            for d in dets:
                eid = d.get("id")
                # Only real (non-phantom, non-clutter) entity detections
                # are fusable - UAV-to-UAV sightings ("uav_N") aren't
                # entities and are never fused, same as before.
                if d.get("is_phantom") or eid is None or eid.startswith("uav_"):
                    continue
                contributions_by_id.setdefault(eid, []).append((i, d["x"], d["y"], d.get("confidence", 0.5)))

        for eid, contributions in contributions_by_id.items():
            fused = fuse_obstacle_detections(contributions, self.fusion_mode)
            if fused is None:
                continue

            radius = self._entity_radius.get(eid, 0.0)
            kind = next((e["kind"] for e in self.entities if e["id"] == eid), "obstacle")
            for i, dets in raw_percepts.items():
                had_own = any(d.get("id") == eid for d in dets)
                fx, fy = fused["x"], fused["y"]
                d_fused = max(dist(self.pos[i], (fx, fy)) - radius, 0.0)
                if d_fused > self.sensor_range:
                    # Fused estimate is only usable if it's within this UAV's own
                    # sensing range - fusion shares detections, not omniscience.
                    continue
                if not had_own:
                    self.fusion_recovery_count += 1
                fused_det = {
                    "kind": kind, "id": eid, "x": fx, "y": fy,
                    "distance": d_fused, "confidence": fused["confidence"],
                    "is_fused": True, "fusion_contributors": len(fused["contributors"]),
                }
                dets[:] = [d for d in dets if d.get("id") != eid] + [fused_det]

    def _inject_external_estimates(self, raw_percepts, external_estimates):
        """Replaces each UAV's own detection of a given entity with an
        externally supplied estimate (radar_track_model + fusion_model
        output), the same way _apply_fusion replaces it with its own
        fused_det - just sourced from track fusion instead of per-
        detection fusion. Each estimate dict may include an "id" key
        naming which entity it estimates (defaults to "obstacle_0" since
        the existing radar/track/fusion demo pipeline only tracks the
        primary obstacle today; multi-entity external tracking works the
        same way once a caller supplies more than one id)."""
        for i, dets in raw_percepts.items():
            est = external_estimates.get(i)
            if est is None:
                continue
            eid = est.get("id", "obstacle_0")
            radius = self._entity_radius.get(eid, 0.0)
            fx, fy = est["x"], est["y"]
            d_fused = max(dist(self.pos[i], (fx, fy)) - radius, 0.0)
            kind = next((e["kind"] for e in self.entities if e["id"] == eid), "obstacle")
            fused_det = {
                "kind": kind, "id": eid, "x": fx, "y": fy,
                "distance": d_fused, "confidence": est.get("confidence", 0.5),
                "is_fused": True, "fusion_contributors": est.get("num_sources", 1),
                # Task 13: real fused-position uncertainty, when the
                # caller (run_radar_track_fusion_pipeline) has it, so
                # safety_margin_mode="covariance" isn't stuck assuming
                # maximal uncertainty for every externally-fused estimate.
                "position_variance": est.get("position_variance"),
            }
            dets[:] = [d for d in dets if d.get("id") != eid] + [fused_det]

    def _quality_level_for(self, d):
        """Maps one perceived detection dict to the (best-effort) subset
        of PerceptionQualityMonitor signals this sim can actually supply
        at decision time, and returns (level, composite_score). Only
        self-reported/derived signals are used - never ground truth - so
        this stays consistent with the monitor's own no-ground-truth
        contract. Signals this sim doesn't track per detection (missed-
        update count, innovation, calibration error, comms age, dropout
        rate) are simply omitted, which PerceptionQualityMonitor already
        handles by renormalizing over whatever *is* present."""
        covariance = d.get("position_variance")
        if covariance is None:
            covariance = d.get("covariance_trace")
        signals = {
            "track_covariance": covariance,
            "current_trust_value": d.get("confidence"),
            # More independent UAVs corroborating a fused detection is
            # itself a (weak) agreement signal; an own-sensor detection
            # (no fusion_contributors) has nothing to compare against, so
            # it's left out rather than guessed at.
            "sensor_agreement": (
                clamp(d["fusion_contributors"] / max(self.num_uavs, 2), 0.0, 1.0)
                if d.get("is_fused") and d.get("fusion_contributors") else None),
        }
        level, score, _ = self.quality_monitor.evaluate(signals)
        return level, score

    def _compute_safety_margin(self, d):
        """Task 13: returns (margin, critical_action) for one perceived
        detection. margin is added on top of the raw sensed distance
        when deciding how aggressively/how early to avoid a contact -
        the less this detection can be trusted, the further away it's
        treated as being. critical_action is None unless
        safety_margin_mode is "quality_monitor" and this detection's
        perception quality is judged CRITICAL, in which case it's
        self.critical_quality_action ("hold" / "regroup" /
        "request_fresh_observation") and margin is pinned to
        maximum_safety_margin."""
        mode = self.safety_margin_mode
        base = self.base_safety_margin
        gain = self.uncertainty_margin_gain
        cap = self.maximum_safety_margin

        if mode == "covariance":
            covariance = d.get("position_variance")
            if covariance is None:
                covariance = d.get("covariance_trace")
            # No covariance attached to this detection (e.g. own-sensor
            # detections outside the radar/track/fusion pipeline never
            # carry one) -> treat conservatively as maximally uncertain
            # rather than quietly defaulting to "certain".
            uncertainty = math.sqrt(max(covariance, 0.0)) if covariance is not None else 1.0
            margin = base + gain * uncertainty
            return clamp(margin, 0.0, cap), None

        if mode == "confidence":
            confidence = d.get("confidence")
            uncertainty = 1.0 - clamp(confidence, 0.0, 1.0) if confidence is not None else 1.0
            margin = base + gain * uncertainty
            return clamp(margin, 0.0, cap), None

        if mode == "quality_monitor":
            level, score = self._quality_level_for(d)
            if level == CRITICAL:
                return cap, self.critical_quality_action
            if level == DEGRADED:
                uncertainty = 1.0 - (score if score is not None else 0.0)
                margin = base + gain * uncertainty
            else:  # GOOD
                margin = base
            return clamp(margin, 0.0, cap), None

        # "fixed" (and any unrecognized mode, treated the same way):
        # constant margin, independent of how trustworthy this
        # particular detection is.
        return clamp(base, 0.0, cap), None

    def _steer(self, i, perceived):
        tgt = self.targets[i]
        gx, gy = normalize(tgt[0] - self.pos[i][0], tgt[1] - self.pos[i][1])
        vx, vy = gx * self.goal_gain, gy * self.goal_gain

        triggered_real = False
        triggered_phantom = False
        critical_action = None
        max_margin_active = 0.0

        for d in perceived:
            margin, quality_action = self._compute_safety_margin(d)
            max_margin_active = max(max_margin_active, margin)
            if quality_action is not None:
                # A single CRITICAL-quality contact is enough to distrust
                # this step's avoidance geometry as a whole - later
                # (possibly GOOD-quality) detections don't get to
                # override that back to normal steering.
                critical_action = quality_action

            rx, ry = self.pos[i][0] - d["x"], self.pos[i][1] - d["y"]
            r = max(d["distance"], 0.3)
            if d.get("kind") == "uav":
                cutoff = (self.collision_distance * 1.5 if d.get("is_parked")
                          else self.uav_avoidance_range) + margin
                if r > cutoff:
                    continue
            elif r > self.sensor_range + margin:
                continue
            # Widen the effective danger radius by the uncertainty
            # margin: a contact is treated as `margin` units closer than
            # measured, so avoidance kicks in earlier and harder the less
            # this particular detection can be trusted.
            r_eff = max(r - margin, 0.3)
            rx, ry = normalize(rx, ry)
            strength = self.avoidance_gain / r_eff
            tx, ty = -ry, rx
            tangential_strength = self.tangential_gain / r_eff
            vx += rx * strength + tx * tangential_strength
            vy += ry * strength + ty * tangential_strength
            if strength > 0.05:
                if d.get("is_phantom"):
                    triggered_phantom = True
                else:
                    triggered_real = True

        vx, vy = normalize(vx, vy)
        safety_info = {
            "mode": self.safety_margin_mode,
            "max_margin": round(max_margin_active, 4),
            "critical_action": critical_action,
        }

        if critical_action is not None:
            # Task 13: a CRITICAL-quality contact overrides normal
            # avoidance steering entirely - the swarm can't safely trust
            # the geometry it would otherwise steer on, so it falls back
            # to a conservative, perception-quality-independent behavior
            # rather than computing an avoidance vector from data it
            # doesn't trust.
            if critical_action == "hold":
                return 0.0, 0.0, triggered_real, triggered_phantom, safety_info
            if critical_action == "regroup":
                cx = sum(p[0] for p in self.pos) / self.num_uavs
                cy = sum(p[1] for p in self.pos) / self.num_uavs
                rvx, rvy = normalize(cx - self.pos[i][0], cy - self.pos[i][1])
                slow_speed = self.speed * 0.5
                return rvx * slow_speed, rvy * slow_speed, triggered_real, triggered_phantom, safety_info
            if critical_action == "request_fresh_observation":
                crawl_speed = self.speed * 0.25
                return vx * crawl_speed, vy * crawl_speed, triggered_real, triggered_phantom, safety_info

        # Non-critical uncertainty still throttles speed smoothly instead
        # of only widening the avoidance radius: "moderate uncertainty ->
        # increased separation, high uncertainty -> slower movement" from
        # the design brief. How much of maximum_safety_margin is
        # currently "in use" by the most uncertain nearby contact scales
        # the slowdown, up to a 50% reduction at the cap.
        speed = self.speed
        if self.maximum_safety_margin > 0:
            caution = clamp(max_margin_active / self.maximum_safety_margin, 0.0, 1.0)
            speed = self.speed * (1.0 - 0.5 * caution)

        return vx * speed, vy * speed, triggered_real, triggered_phantom, safety_info

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

    def sense(self, t):
        """Phase 1: each active UAV's own (uncorrupted-by-fusion) perception.
        Split out from step() so a caller (run_radar_track_fusion_pipeline)
        can run radar tracking/fusion on the raw per-UAV detections before
        decide_move() consumes them. Returns (true_dets_all, raw_percepts),
        both dicts keyed by uav id, covering only active (not-yet-arrived)
        UAVs."""
        active = [i for i in range(self.num_uavs) if not self.reached_goal[i]]
        self._current_t = t
        self._advance_entities(t)
        true_dets_all = {}
        raw_percepts = {}
        for i in active:
            true_dets = self._true_detections_for(i)
            true_dets_all[i] = true_dets
            raw_percepts[i] = self.perception[i].process(true_dets, tuple(self.pos[i]))
            for d in raw_percepts[i]:
                if "true_confidence" in d:
                    self.confidence_error_samples.append(abs(d["confidence"] - d["true_confidence"]))
        return true_dets_all, raw_percepts

    def decide_move(self, t, true_dets_all, raw_percepts, external_estimates=None):
        """Phases 2-3: fusion, steering, movement, collision/metric
        bookkeeping and logging, given the (true_dets_all, raw_percepts)
        produced by sense(t).

        external_estimates: optional {uav_id: {"x", "y", "confidence",
        "num_sources"}} obstacle position estimate to feed into that UAV's
        decision-making INSTEAD OF the built-in per-detection
        _apply_fusion (used by run_radar_track_fusion_pipeline() to hand
        off a radar_track_model + fusion_model track-fused estimate
        instead). None (the default) preserves the original behavior."""
        new_pos = [list(p) for p in self.pos]
        formation_dists = []
        step_info = [None] * self.num_uavs  # per-UAV data needed for logging, filled in below

        # --- Phase 2: cross-UAV sensor fusion of the obstacle detection ---
        if external_estimates is not None:
            self._inject_external_estimates(raw_percepts, external_estimates)
        else:
            self._apply_fusion(raw_percepts)

        # --- Phase 3: latency buffering, steering, logging ---
        for i in range(self.num_uavs):
            if self.reached_goal[i]:
                step_info[i] = {
                    "perceived": [],
                    "perceived_obstacle": None,
                    "event": "at_goal",
                    "error_type": "none",
                    "unnecessary_avoidance": False,
                    "missed_response": False,
                    "safety_margin_mode": self.safety_margin_mode,
                    "safety_margin_applied": 0.0,
                    "quality_action_taken": None,
                }
                continue

            true_dets = true_dets_all[i]
            raw_perceived = raw_percepts[i]

            # Sensor (post-fusion) output is generated now but only "arrives"
            # at the controller after latency_steps (0 latency = instant, as before).
            self.detection_buffer[i].append((t, raw_perceived))
            perceived = self._get_delayed_perception(i, t)

            # Clock starts the moment a detection exists at all (i.e. within
            # sensor_range, which true_dets is already filtered to) - not once
            # it closes to near_miss_distance. Latency/dropout/false-negative
            # have nothing left to delay by the time something is that close,
            # since it's already been sitting in the perceived list for many
            # steps (near_miss_distance 3.5 << sensor_range 15).
            for d in true_dets:
                key = (i, d["id"])
                if key not in self._threat_first_true_step:
                    self._threat_first_true_step[key] = t
            perceived_ids = {d["id"] for d in perceived if not d.get("is_phantom")}
            for pid in perceived_ids:
                key = (i, pid)
                if key in self._threat_first_true_step and key not in self._threat_first_perceived_step:
                    self._threat_first_perceived_step[key] = t

            vx, vy, triggered_real, triggered_phantom, safety_info = self._steer(i, perceived)

            unnecessary_avoidance = triggered_phantom and not triggered_real
            if unnecessary_avoidance:
                self.unnecessary_avoidance_count += 1
            if triggered_real:
                self.avoidance_action_count += 1

            critical_action = safety_info["critical_action"]
            if critical_action == "hold":
                self.hold_count += 1
            elif critical_action == "regroup":
                self.regroup_count += 1
            elif critical_action == "request_fresh_observation":
                self.request_fresh_observation_count += 1

            missed_response = False
            for d in true_dets:
                if d["distance"] <= self.near_miss_distance and d["id"] not in perceived_ids:
                    missed_response = True
                    self.missed_response_count += 1

            new_pos[i][0] = min(max(new_pos[i][0] + vx * self.dt, 0.0), self.cfg["world"]["width"])
            new_pos[i][1] = min(max(new_pos[i][1] + vy * self.dt, 0.0), self.cfg["world"]["height"])

            if critical_action is not None:
                # A critical-quality contact overrode normal steering
                # this step - that's more informative than "avoidance"/
                # "move" for what the UAV actually did.
                event = critical_action
            else:
                event = "avoidance" if triggered_real else ("false_avoidance" if triggered_phantom else "move")

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
            if self.scn.get("confidence_error_level", 0.0) > 0:
                errors.append("confidence_error")
            error_type = "+".join(errors) if errors else "none"

            perceived_obstacle = next((d for d in perceived if d.get("id") == "obstacle_0"), None)

            step_info[i] = {
                "perceived": perceived,
                "perceived_obstacle": perceived_obstacle,
                "event": event,
                "error_type": error_type,
                "unnecessary_avoidance": unnecessary_avoidance,
                "missed_response": missed_response,
                "safety_margin_mode": safety_info["mode"],
                "safety_margin_applied": safety_info["max_margin"],
                "quality_action_taken": critical_action,
            }

        active_entities = [e for e in self.entities if self._entity_active(e, t)]

        for i in range(self.num_uavs):
            for e in active_entities:
                d_ent = dist(new_pos[i], (e["x"], e["y"])) - e["radius"]
                if d_ent <= self.collision_distance:
                    self.collision_count += 1
                elif d_ent <= self.near_miss_distance:
                    self.near_miss_count += 1
            for j in range(i + 1, self.num_uavs):
                d_uav = dist(new_pos[i], new_pos[j])
                if d_uav <= self.collision_distance:
                    self.collision_count += 1
                elif d_uav <= self.near_miss_distance:
                    self.near_miss_count += 1
                formation_dists.append(d_uav)

        # Per-UAV distance to the nearest *other UAV* and distance to the
        # nearest active entity (surface distance), at the post-move
        # positions - kept as a read-only pass, separate from the
        # counting loop above so collision/near-miss totals are unaffected.
        nearest_uav_info = [None] * self.num_uavs
        obstacle_dist_info = [None] * self.num_uavs
        nearest_info = [None] * self.num_uavs
        for i in range(self.num_uavs):
            nearest_type = None
            nearest_dist = None
            nearest_entity_dist = None
            for e in active_entities:
                d_ent = dist(new_pos[i], (e["x"], e["y"])) - e["radius"]
                d_ent = max(d_ent, 0.0)
                if nearest_entity_dist is None or d_ent < nearest_entity_dist:
                    nearest_entity_dist = d_ent
                if nearest_dist is None or d_ent < nearest_dist:
                    nearest_dist = d_ent
                    nearest_type = e["id"]
            # obstacle_dist_info stays specifically the primary obstacle's
            # distance (backward compatible column meaning), separate from
            # nearest_entity_dist which is the closest of ALL entities.
            ox, oy, orad = self.obstacle
            obstacle_dist_info[i] = max(dist(new_pos[i], (ox, oy)) - orad, 0.0)
            if nearest_type is None:
                nearest_type, nearest_dist = "none", float("inf")

            nearest_uav_dist = None
            for j in range(self.num_uavs):
                if j == i:
                    continue
                d_uav = dist(new_pos[i], new_pos[j])
                if nearest_uav_dist is None or d_uav < nearest_uav_dist:
                    nearest_uav_dist = d_uav
                if d_uav < nearest_dist:
                    nearest_dist = d_uav
                    nearest_type = f"uav_{j}"
            nearest_uav_info[i] = nearest_uav_dist
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
            self.log_rows.append(self._log_row(
                t, i, step_info[i], nearest_info[i], nearest_uav_info[i],
                obstacle_dist_info[i], mission_completed,
            ))

    def step(self, t, external_estimates=None):
        """Advances the sim by one step (sense() + decide_move() combined).
        See decide_move() for what external_estimates does."""
        true_dets_all, raw_percepts = self.sense(t)
        self.decide_move(t, true_dets_all, raw_percepts, external_estimates)

    def _log_row(self, t, i, info, nearest, nearest_uav_dist, obstacle_dist, mission_completed):
        perceived = info["perceived"]
        perceived_obstacle = info["perceived_obstacle"]
        ox, oy, _ = self.obstacle
        tx, ty = self.targets[i]
        nearest_type, nearest_dist = nearest

        # Ground truth for every currently-active entity (obstacles and
        # targets alike), independent of what this UAV actually perceived
        # - lets a log consumer see the full multi-entity world state
        # (Task 7) without needing a separate per-entity table. Kept as one
        # compact JSON column rather than exploding into per-entity
        # columns, since the entity count/composition varies by scenario.
        active_entities_summary = json.dumps([
            {"id": e["id"], "kind": e["kind"], "x": round(e["x"], 3), "y": round(e["y"], 3),
             "radius": e["radius"]}
            for e in self.entities if self._entity_active(e, t)
        ])

        return {
            "scenario": self.scenario_name,
            "step": t,
            "time_s": round(t * self.dt, 3),
            "uav_id": i,
            "uav_pos_x": round(self.pos[i][0], 3),
            "uav_pos_y": round(self.pos[i][1], 3),

            # Goal position (assigned per-UAV, not sensed - always known exactly).
            "goal_pos_x": round(tx, 3),
            "goal_pos_y": round(ty, 3),

            # Perceived vs. actual obstacle position (perceived is None if the
            # obstacle wasn't detected this step, e.g. dropout/false negative/
            # out of range; otherwise it reflects any noise applied and/or fusion).
            "actual_obstacle_x": round(ox, 3),
            "actual_obstacle_y": round(oy, 3),
            "perceived_obstacle_x": round(perceived_obstacle["x"], 3) if perceived_obstacle else None,
            "perceived_obstacle_y": round(perceived_obstacle["y"], 3) if perceived_obstacle else None,

            "perception_error_type": info["error_type"],
            "confidence_value": perceived_obstacle["confidence"] if perceived_obstacle else None,
            "fusion_mode": self.fusion_mode,
            "action_taken": info["event"],

            # Task 13: which safety-margin strategy was active, how large
            # a margin it produced this step (max across perceived
            # contacts), and which critical_quality_action (if any) fired.
            "safety_margin_mode": info["safety_margin_mode"],
            "safety_margin_applied": info["safety_margin_applied"],
            "quality_action_taken": info["quality_action_taken"],

            "num_perceived_detections": len(perceived),
            "num_phantom_detections": sum(1 for d in perceived if d.get("is_phantom")),
            "dist_to_goal": round(dist(self.pos[i], self.targets[i]), 3),

            "distance_to_nearest_uav": round(nearest_uav_dist, 3) if nearest_uav_dist is not None else None,
            "distance_to_obstacle": round(obstacle_dist, 3),
            "nearest_entity_type": nearest_type,
            "nearest_entity_distance": round(nearest_dist, 3),

            "collision_risk_flag": bool(nearest_dist <= self.near_miss_distance),
            "unnecessary_avoidance_flag": bool(info["unnecessary_avoidance"]),
            "missed_response_flag": bool(info["missed_response"]),
            "mission_completed_flag": mission_completed,

            "reached_goal": self.reached_goal[i],

            # Task 7: multi-entity ground truth.
            "num_active_entities": sum(1 for e in self.entities if self._entity_active(e, t)),
            "active_entities_json": active_entities_summary,
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
        avg_confidence_error = (sum(self.confidence_error_samples) / len(self.confidence_error_samples)
                                 if self.confidence_error_samples else None)
        return {
            "scenario": self.scenario_name,
            "fusion_mode": self.fusion_mode,
            "steps_run": final_step + 1,
            "uavs_reached_goal": num_reached,
            "num_uavs": self.num_uavs,
            "mission_success": bool(num_reached == self.num_uavs and self.collision_count == 0),
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            "unnecessary_avoidance_count": self.unnecessary_avoidance_count,
            "missed_response_count": self.missed_response_count,
            "avoidance_action_count": self.avoidance_action_count,
            "fusion_recovery_count": self.fusion_recovery_count,
            "avg_response_time_s": round(avg_response_time, 3) if avg_response_time is not None else None,
            "avg_formation_error": round(avg_formation_error, 3) if avg_formation_error is not None else None,
            "avg_confidence_error": round(avg_confidence_error, 3) if avg_confidence_error is not None else None,

            # Task 13: uncertainty-aware safety margins.
            "safety_margin_mode": self.safety_margin_mode,
            "hold_count": self.hold_count,
            "regroup_count": self.regroup_count,
            "request_fresh_observation_count": self.request_fresh_observation_count,
        }


def _generate_vision_lidar_detections(uav_id, t, sim):
    """Extension point for pipeline step 6 ("optionally generate
    vision-like/LiDAR-like detections"). No vision or LiDAR sensor model
    exists in this project - fusion_model.py fuses radar tracks only, and
    stays that way. This always returns [] on purpose; a future vision/
    LiDAR model would return detections shaped like Perception.process()'s
    output ({"id", "x", "y", "confidence", ...}) to be tracked/fused
    alongside the radar detections below."""
    return []


def run_radar_track_fusion_pipeline(config, scenario_name):
    """Drives one scenario through the full pipeline (Task 11): true
    state -> radar detections (P_D/P_FA/clutter/noise/latency/dropout,
    all via radar_like_model.py) -> per-radar tracks
    (radar_track_model.py) -> optional vision/LiDAR detections (no-op,
    see above) -> cross-UAV track fusion (fusion_model.py) -> UAV
    decision + movement -> metrics -> logs, run per step instead of as
    three separate batch passes so the fused estimate can be handed back
    to decision-making.

    ponytail: the fused estimate a UAV steers on is one step (dt) stale -
    built from step t's tracks, applied starting at step t+1. That's the
    same one-tick-lag shape latency_steps already models elsewhere in
    this project. Genuine same-tick feedback would need the tracker
    threaded into the middle of a single step with no coast/gate slack,
    which RadarTracker isn't built for; splitting Simulation's decide/move
    phase further is the upgrade path if zero-lag fusion feedback is ever
    required. Ground truth is never used for decisions either way - see
    the sense()/decide_move() split and _inject_external_estimates().

    Returns (rows, metrics): rows is a list of dicts matching the Task 12
    CSV schema (one row per active UAV per step); metrics is the same
    summary dict Simulation.run() returns.
    """
    from models.radar_like_model import RadarLikeModel, _range_bearing_radial
    from tracking.radar_track_model import RadarTracker
    from fusion.fusion_model import fuse_step, TrustTracker

    model = RadarLikeModel(config, scenario_name)
    sim = model.sim
    dt = sim.dt
    fusion_mode = sim.fusion_mode

    trackers = {i: RadarTracker(i) for i in range(sim.num_uavs)}
    obstacle_track_id = {}   # uav_id -> track_id currently believed to be the obstacle
    pending_estimates = {}   # last step's fused_by_uav, applied to this step's decision
    rows = []

    # Task 15: dynamic trust adaptation was previously only wired into
    # fusion_model.build_fused_log's offline evaluation path, never into
    # the live decision loop below - so trust_weighted_fusion here always
    # ran with persistent_trust fixed at 1.0 ("fixed" trust-weighting),
    # regardless of config. Same trust_adaptation.enabled config key as
    # build_fused_log; defaults on.
    trust_cfg = sim.scn.get("trust_adaptation", config.get("trust_adaptation", {}))
    trust_tracker = TrustTracker() if trust_cfg.get("enabled", True) else None

    t = 0
    for t in range(sim.max_steps):
        if all(sim.reached_goal):
            break

        model._capture = {}
        model._current_t = t
        pos_before = {i: tuple(sim.pos[i]) for i in range(sim.num_uavs)}

        # 1-5: true state update + radar detections, already carrying
        # P_D/P_FA/clutter/noise/confidence-error/update-rate/latency/
        # dropout (all applied inside the patched Perception.process
        # radar_like_model installs on construction).
        true_dets_all, raw_percepts = sim.sense(t)
        active_this_step = list(raw_percepts.keys())

        # 6: optional vision-like/LiDAR-like detections (no-op today).
        for i in active_this_step:
            raw_percepts[i] = raw_percepts[i] + _generate_vision_lidar_detections(i, t, sim)

        # Snapshot the raw per-radar detections now, before decide_move()
        # (below) overwrites raw_percepts in place with the fused
        # estimate - the tracker must only ever see genuine sensor
        # output, never its own previous fused output.
        raw_snapshot = {i: [dict(d) for d in raw_percepts[i]] for i in active_this_step}

        # 6 (track model): per-radar nearest-neighbor tracking.
        obstacle_track_row_by_uav = {}
        for i in active_this_step:
            dets_for_tracker = [{"x": d["x"], "y": d["y"], "confidence": d.get("confidence")}
                                 for d in raw_snapshot[i]]
            track_rows = trackers[i].update(t, dets_for_tracker, dt)

            obstacle_det = next((d for d in raw_snapshot[i] if d.get("id") == "obstacle_0"), None)
            if obstacle_det is not None and track_rows:
                match = min(track_rows, key=lambda r: math.hypot(
                    r["est_x"] - obstacle_det["x"], r["est_y"] - obstacle_det["y"]))
                obstacle_track_id[i] = match["track_id"]
            # else: keep whatever track id was last associated with the
            # obstacle for this radar, if it's still alive (coasting
            # through a miss) - checked just below.

            obs_row = next((r for r in track_rows if r["track_id"] == obstacle_track_id.get(i)), None)
            if obs_row is None:
                obstacle_track_id.pop(i, None)
            else:
                obstacle_track_row_by_uav[i] = obs_row

        # 7: fuse this step's obstacle tracks across UAVs.
        fused_clusters = fuse_step(list(obstacle_track_row_by_uav.values()), fusion_mode,
                                    trust_tracker=trust_tracker)
        track_id_to_uav = {tid: uav for uav, tid in obstacle_track_id.items()}
        fused_by_uav = {}
        for cluster in fused_clusters:
            for tid in cluster["source_ids"]:
                uav = track_id_to_uav.get(tid)
                if uav is not None:
                    fused_by_uav[uav] = cluster

        # 8-9: hand last step's fused estimate to decision-making, move,
        # then hand this step's estimate off for next step.
        sim.decide_move(t, true_dets_all, raw_percepts, external_estimates=pending_estimates)
        pending_estimates = {i: {"x": c["x"], "y": c["y"], "confidence": c["confidence"],
                                  "num_sources": c["num_sources"],
                                  # Task 13: carried through to
                                  # _inject_external_estimates so
                                  # safety_margin_mode="covariance" has
                                  # real per-UAV uncertainty to work with
                                  # in the full radar/track/fusion
                                  # pipeline, not just the simplified
                                  # no_fusion path.
                                  "position_variance": c.get("position_variance")}
                             for i, c in fused_by_uav.items()}
        this_step_logs = sim.log_rows[-sim.num_uavs:]
        formation_error_this_step = sim.formation_error_samples[-1] if sim.formation_error_samples else None

        # 10-11: metrics + one combined log row per sensing UAV. Parked
        # UAVs (already at their goal) don't get a row this step, since
        # sense() doesn't run their radar either.
        for i in active_this_step:
            pos_now = sim.pos[i]
            observer_vel = ((pos_now[0] - pos_before[i][0]) / dt, (pos_now[1] - pos_before[i][1]) / dt)

            ox, oy, _ = sim.obstacle
            true_range, true_bearing, true_radial_velocity = _range_bearing_radial(
                pos_before[i], observer_vel, (ox, oy), (0.0, 0.0))

            obstacle_det = next((d for d in raw_snapshot[i] if d.get("id") == "obstacle_0"), None)
            if obstacle_det is not None:
                detected_x, detected_y = obstacle_det["x"], obstacle_det["y"]
                measured_range = obstacle_det.get("measured_range")
                measured_bearing = obstacle_det.get("measured_bearing")
                radar_confidence = obstacle_det.get("confidence")
                _, _, base_radial = _range_bearing_radial(
                    pos_before[i], observer_vel, (detected_x, detected_y), (0.0, 0.0))
                measured_radial_velocity = (
                    round(base_radial + model.radar_rng.gauss(0.0, model.radial_velocity_noise_std), 4)
                    if base_radial is not None else None)
            else:
                detected_x = detected_y = measured_range = measured_bearing = None
                measured_radial_velocity = radar_confidence = None

            # ponytail: simplified distance-based SNR proxy (falls off with
            # range relative to radar_max_range), not a full radar-equation
            # SNR model - there's no RCS/power/noise-floor model here to
            # derive a physical SNR from.
            range_for_snr = measured_range if measured_range is not None else true_range
            radar_snr = (round(clamp(20 * math.log10(model.radar_max_range / max(range_for_snr, 0.05)),
                                      0.0, 60.0), 2)
                         if range_for_snr else None)

            obstacle_in_range = any(d["id"] == "obstacle_0" for d in true_dets_all.get(i, []))
            obs_track_row = obstacle_track_row_by_uav.get(i)
            fused = fused_by_uav.get(i)
            log_row = this_step_logs[i]
            nearest_dist = log_row["nearest_entity_distance"]

            rows.append({
                "time_step": t,
                "uav_id": i,
                "uav_x": round(pos_now[0], 4),
                "uav_y": round(pos_now[1], 4),
                "true_target_x": round(ox, 4),
                "true_target_y": round(oy, 4),
                "true_target_vx": 0.0,
                "true_target_vy": 0.0,
                "radar_id": i,
                "true_range": round(true_range, 4) if true_range is not None else None,
                "true_bearing": round(true_bearing, 5) if true_bearing is not None else None,
                "true_radial_velocity": round(true_radial_velocity, 4) if true_radial_velocity is not None else None,
                "measured_range": round(measured_range, 4) if measured_range is not None else None,
                "measured_bearing": round(measured_bearing, 5) if measured_bearing is not None else None,
                "measured_radial_velocity": measured_radial_velocity,
                "detected_x": round(detected_x, 4) if detected_x is not None else None,
                "detected_y": round(detected_y, 4) if detected_y is not None else None,
                "radar_confidence": radar_confidence,
                "radar_snr": radar_snr,
                "radar_track_id": obs_track_row["track_id"] if obs_track_row else None,
                "track_status": obs_track_row["status"] if obs_track_row else None,
                "false_alarm_flag": any(d.get("id") == "phantom" or d.get("is_radar_clutter")
                                         for d in raw_snapshot[i]),
                "missed_detection_flag": bool(obstacle_in_range and obstacle_det is None),
                "clutter_flag": any(d.get("is_radar_clutter") for d in raw_snapshot[i]),
                "dropout_flag": bool(model._held_dropout.get(i, False) or sim.perception[i].last_dropout),
                "latency_steps": model.radar_latency_steps,
                "fusion_mode": fusion_mode,
                "fused_x": round(fused["x"], 4) if fused else None,
                "fused_y": round(fused["y"], 4) if fused else None,
                "action_taken": log_row["action_taken"],
                "collision_risk_flag": bool(nearest_dist <= sim.collision_distance),
                "near_miss_flag": bool(sim.collision_distance < nearest_dist <= sim.near_miss_distance),
                "unnecessary_avoidance_flag": bool(log_row["unnecessary_avoidance_flag"]),
                "missed_response_flag": bool(log_row["missed_response_flag"]),
                "mission_success_flag": bool(log_row["mission_completed_flag"]),
                "formation_error": (round(formation_error_this_step, 4)
                                    if formation_error_this_step is not None else None),
                "track_covariance_trace": _covariance_trace(obs_track_row),
                "track_est_vx": obs_track_row.get("est_vx") if obs_track_row else None,
                "track_est_vy": obs_track_row.get("est_vy") if obs_track_row else None,
                "fusion_num_sources": fused.get("num_sources") if fused else None,
                "fusion_position_variance": fused.get("position_variance") if fused else None,
                "fusion_comm_messages": fused.get("comm_messages") if fused else None,
                "fusion_response_time_steps": fused.get("response_time_steps") if fused else None,

                # Task 13: uncertainty-aware safety margins.
                "safety_margin_mode": log_row["safety_margin_mode"],
                "safety_margin_applied": log_row["safety_margin_applied"],
                "quality_action_taken": log_row["quality_action_taken"],
            })

    metrics = sim._metrics(t)
    return rows, metrics


def main():
    parser = argparse.ArgumentParser(
        description="2D UAV swarm simulation. Default mode (Task 11) runs the full "
                     "radar -> track -> fusion -> decision pipeline: the UAV never "
                     "sees ground truth, only radar-detected/fused position estimates.")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default=None,
                         help="Output CSV path. Defaults to logs/full_pipeline_log.csv "
                              "(or logs/simulation_log.csv with --legacy-pipeline).")
    parser.add_argument("--legacy-pipeline", action="store_true",
                         help="Run the OLD perception-only sim (pre-radar milestone, "
                              "frozen in current_simulation_milestone.md) instead of the "
                              "radar -> track -> fusion -> decision pipeline. The UAV in "
                              "this mode uses Perception's false-positive/negative/noise "
                              "model directly, with no radar sensor model in front of it. "
                              "Kept only for reproducing/comparing against the pre-radar "
                              "baseline; not the default because it no longer reflects how "
                              "the project's UAVs are meant to sense the world.")
    args = parser.parse_args()

    if args.log is None:
        args.log = "logs/simulation_log.csv" if args.legacy_pipeline else "logs/full_pipeline_log.csv"

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        if args.legacy_pipeline:
            sim = Simulation(config, name)
            metrics = sim.run()
            rows = sim.log_rows
        else:
            rows, metrics = run_radar_track_fusion_pipeline(config, name)
        all_rows.extend(rows)
        print(json.dumps(metrics, indent=2))

    if all_rows:
        fieldnames = list(all_rows[0].keys())
        with open(args.log, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)


if __name__ == "__main__":
    main()