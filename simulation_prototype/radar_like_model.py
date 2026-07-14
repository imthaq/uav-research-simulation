"""
radar_like_model.py

Generates radar-style detection rows (range/bearing/Doppler-like measurements)
for every UAV in the swarm simulation, without changing simple_swarm_sim.py.

How it hooks in
----------------
Simulation already computes, every step, per UAV:
  - what's truly in range (Simulation._true_detections_for)
  - what the sensor perceives after false pos/neg, dropout, noise, confidence
    (Perception.process)
  - the cross-UAV fused version of that, if fusion is enabled
    (Simulation._apply_fusion)

This module wraps three existing methods at the instance level so it can
watch their inputs/outputs and add radar-domain measurements on top, without
touching or re-implementing any of Simulation's own logic:

  1. Perception.process  -> after the normal perception step, compute true
     range/bearing to each detection, add range/bearing noise, and convert
     the noisy range/bearing back into an x/y that OVERWRITES the
     detection's position. Because Simulation reads x/y straight off this
     same dict, the UAV's avoidance/goal-seeking is now driven by the noisy
     radar position, not ground truth.
  2. Simulation._apply_fusion -> after fusion runs, re-snapshot each UAV's
     detection list. This must happen AFTER fusion, not before, because
     fusion can replace an individual UAV's obstacle detection with a
     shared one - logging the pre-fusion version would misreport what the
     UAV actually acted on.
  3. Simulation.step -> lets us know the current step index and capture
     each UAV's position before/after, used to derive velocities (needed
     for Doppler / radial velocity).

Each row reported has: time_step, radar_id, target_id, true_target_x/y,
target_velocity_x/y, true_range/bearing/radial_velocity, measured_range/
bearing/radial_velocity, detected_x/y, confidence_score, detection_status,
false_alarm_flag, missed_detection_flag, clutter_flag, dropout_flag,
radar_pd_miss_flag, false_alarm_source.

Radar probability of detection (Task 5)
----------------------------------------
On top of the existing false_negative_rate perception model (which emulates
upstream perception/comms dropping a detection), this module layers a
genuine radar sensor-model gate: radar_detection_probability (P_D). For
every real (non-phantom) detection that survives Perception.process, an
independent Bernoulli trial with probability P_D decides whether the radar
actually registers it THIS scan. A low P_D increases missed detections
(missed_detection_flag=True) beyond whatever false_negative_rate already
produces, which can increase missed responses, collision risk, and reduce
mission success - because the detection is genuinely removed from what the
UAV acts on, not just logged as missed after the fact.

Radar false alarms / clutter (Task 6)
--------------------------------------
Independently of the existing config-driven false_positive_rate ("phantom")
mechanism, this module also generates radar clutter every step via
_generate_clutter(): a Poisson-distributed number of clutter "candidate
returns" (rate = radar_clutter_density) are drawn inside the radar's
sensing disk, and each candidate is confirmed as a reported false detection
with probability radar_false_alarm_probability (P_FA). Confirmed clutter
points get a random range/bearing (and therefore x/y), a confidence score,
and are flagged false_alarm_flag=True/clutter_flag=True. They are injected
into the same detection stream the UAV steers on (marked is_phantom=True so
Simulation's existing avoidance/steering logic reacts to them exactly like
any other false detection), which can increase unnecessary avoidance, wrong
decisions, and response time. This runs on every step of every scenario -
there is no separate demo scenario for it.

Radar sensing limits (Task 7)
------------------------------
All 13 radar_* keys live in simulation_config.json (top-level "radar"
defaults + per-scenario overrides), all read scenario-first via
scn.get(key, radar_cfg.get(key, default)):
  - radar_max_range / radar_min_range: real detections outside this window
    are dropped before the P_D roll even runs (radar_pd_miss_flag=True,
    same as a P_D miss - the target genuinely wasn't in range this scan).
  - radar_field_of_view: angular sector (degrees) around the UAV's
    heading-to-its-own-goal (no explicit orientation state exists, so this
    is the stand-in); 360 (default) is a no-op. Clutter is generated inside
    the same sector, not omnidirectionally, once FOV is restricted.
  - radar_update_rate: how often the radar actually scans (Hz); between
    scans the most recently delivered scan is held and re-served.
  - radar_latency_steps: extra delay, in steps, before a scan reaches the
    controller, on top of radar_update_rate's own hold.
  - radar_dropout_probability: per-scan chance of a total radar blackout
    (independent of the base Perception model's own dropout_prob).
  - radar_confidence_error: extra Gaussian miscalibration applied to every
    detection's confidence at the radar-reporting stage, on top of
    whatever Perception already applied.
"""

import argparse
import csv
import json
import math
import random

from simple_swarm_sim import Simulation, dist, clamp


def _range_bearing_radial(observer_pos, observer_vel, target_pos, target_vel):
    """Range, bearing, and radial velocity (range-rate) from observer to
    target. Radial velocity is positive when the target is moving away
    (range increasing): relative velocity projected onto the observer->
    target line-of-sight unit vector."""
    dx = target_pos[0] - observer_pos[0]
    dy = target_pos[1] - observer_pos[1]
    rng = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)

    if target_vel is None:
        radial_vel = None
    elif rng < 1e-9:
        radial_vel = 0.0
    else:
        rel_vx = target_vel[0] - observer_vel[0]
        rel_vy = target_vel[1] - observer_vel[1]
        radial_vel = (dx * rel_vx + dy * rel_vy) / rng

    return rng, bearing, radial_vel


def _wrap_angle(angle):
    """Wraps an angle (radians) into (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class RadarLikeModel:
    """Wraps a Simulation instance and produces radar-style detection rows.

    Usage:
        model = RadarLikeModel(config, scenario_name)
        rows = model.run()
    """

    # Independent RNG for radar-domain noise (range, bearing, radial
    # velocity), kept separate from Simulation's own RNG.
    RADAR_NOISE_SEED_OFFSET = 99991

    DEFAULT_RANGE_NOISE_STD = 0.3                   # meters, 1-sigma
    DEFAULT_BEARING_NOISE_STD = math.radians(2.0)   # radians, ~2 deg 1-sigma
    DEFAULT_RADIAL_VELOCITY_NOISE_STD = 0.1         # units/sec, 1-sigma

    # Task 5: radar probability of detection. Independent of
    # false_negative_rate - this is the radar's own per-scan detection gate.
    DEFAULT_RADAR_DETECTION_PROBABILITY = 0.95      # P_D

    # Task 6: radar false alarm / clutter model.
    DEFAULT_RADAR_FALSE_ALARM_PROBABILITY = 0.05    # P_FA: chance a clutter
                                                     # candidate return gets
                                                     # reported as a detection
    DEFAULT_RADAR_CLUTTER_DENSITY = 0.5             # mean clutter candidate
                                                     # returns per scan (Poisson rate)

    # Task 7: radar sensing limits.
    DEFAULT_MIN_RANGE = 0.0                         # no blind zone
    DEFAULT_FIELD_OF_VIEW_DEG = 360.0               # omnidirectional
    DEFAULT_LATENCY_STEPS = 0
    DEFAULT_DROPOUT_PROBABILITY = 0.0
    DEFAULT_CONFIDENCE_ERROR = 0.0

    def __init__(self, config, scenario_name):
        self.cfg = config
        self.scenario_name = scenario_name
        self.sim = Simulation(config, scenario_name)
        self.dt = config["sim"]["dt"]

        base_seed = config["sim"].get("seed", 0)
        self.radar_rng = random.Random(base_seed + self.RADAR_NOISE_SEED_OFFSET)

        # Noise level: scenario override -> top-level "radar" config
        # section -> built-in default. Keys are radar_*-prefixed to match
        # simulation_config.json (they were previously read unprefixed,
        # which silently ignored the config file's radar_range_noise_std /
        # radar_bearing_noise_std / radar_radial_velocity_noise_std values).
        radar_cfg = config.get("radar", {})
        scn = self.sim.scn
        self.range_noise_std = scn.get(
            "radar_range_noise_std", radar_cfg.get("radar_range_noise_std", self.DEFAULT_RANGE_NOISE_STD))
        self.bearing_noise_std = scn.get(
            "radar_bearing_noise_std",
            radar_cfg.get("radar_bearing_noise_std", self.DEFAULT_BEARING_NOISE_STD))
        self.radial_velocity_noise_std = scn.get(
            "radar_radial_velocity_noise_std",
            radar_cfg.get("radar_radial_velocity_noise_std", self.DEFAULT_RADIAL_VELOCITY_NOISE_STD))

        # Task 5: radar probability of detection (P_D). Scenario override ->
        # top-level "radar" config section -> built-in default.
        self.detection_probability = scn.get(
            "radar_detection_probability",
            radar_cfg.get("radar_detection_probability", self.DEFAULT_RADAR_DETECTION_PROBABILITY))

        # Task 6: radar false alarm probability (P_FA) and clutter density.
        self.false_alarm_probability = scn.get(
            "radar_false_alarm_probability",
            radar_cfg.get("radar_false_alarm_probability", self.DEFAULT_RADAR_FALSE_ALARM_PROBABILITY))
        self.clutter_density = scn.get(
            "radar_clutter_density",
            radar_cfg.get("radar_clutter_density", self.DEFAULT_RADAR_CLUTTER_DENSITY))

        # Task 7: radar sensing limits.
        self.radar_max_range = scn.get(
            "radar_max_range", radar_cfg.get("radar_max_range", self.sim.sensor_range))
        self.radar_min_range = scn.get(
            "radar_min_range", radar_cfg.get("radar_min_range", self.DEFAULT_MIN_RANGE))
        self.radar_field_of_view = scn.get(
            "radar_field_of_view", radar_cfg.get("radar_field_of_view", self.DEFAULT_FIELD_OF_VIEW_DEG))

        default_update_rate = (1.0 / self.dt) if self.dt > 0 else 1.0
        self.radar_update_rate = scn.get(
            "radar_update_rate", radar_cfg.get("radar_update_rate", default_update_rate))
        self.radar_update_interval_steps = (
            max(1, round(1.0 / (self.radar_update_rate * self.dt)))
            if self.radar_update_rate > 0 else 1)

        self.radar_latency_steps = scn.get(
            "radar_latency_steps", radar_cfg.get("radar_latency_steps", self.DEFAULT_LATENCY_STEPS))
        self.radar_dropout_probability = scn.get(
            "radar_dropout_probability",
            radar_cfg.get("radar_dropout_probability", self.DEFAULT_DROPOUT_PROBABILITY))
        self.radar_confidence_error = scn.get(
            "radar_confidence_error", radar_cfg.get("radar_confidence_error", self.DEFAULT_CONFIDENCE_ERROR))

        self._clutter_counter = 0  # monotonically increasing id suffix for generated clutter points

        self._capture = {}  # uav_id -> captured true/final-perceived data for the current step
        self.rows = []

        # Per-UAV state for the Task 7 update-rate/latency scan buffer.
        self._scan_buffer = {i: [] for i in range(self.sim.num_uavs)}
        self._held_perceived = {i: [] for i in range(self.sim.num_uavs)}
        self._held_dropout = {i: False for i in range(self.sim.num_uavs)}
        self._held_pd_missed = {i: [] for i in range(self.sim.num_uavs)}
        self._current_t = 0

        self._patch_perception()
        self._patch_fusion()
        self._patch_step()

    # ------------------------------------------------------------------
    # Radar measurement: range/bearing noise, converted back to x/y
    # ------------------------------------------------------------------
    def _apply_radar_noise(self, d, uav_pos, true_by_id):
        """Computes true range/bearing to the target, adds independent
        range/bearing noise, and overwrites the detection's x/y (and
        distance) with the position implied by the noisy range/bearing.
        This is what Simulation's steering and fusion read afterwards."""
        if d.get("is_phantom"):
            base_x, base_y = d["x"], d["y"]
        else:
            true_d = true_by_id.get(d.get("id"))
            base_x, base_y = (true_d["x"], true_d["y"]) if true_d else (d["x"], d["y"])

        dx = base_x - uav_pos[0]
        dy = base_y - uav_pos[1]
        true_range = math.hypot(dx, dy)
        true_bearing = math.atan2(dy, dx)

        noisy_range = max(true_range + self.radar_rng.gauss(0.0, self.range_noise_std), 0.05)
        noisy_bearing = true_bearing + self.radar_rng.gauss(0.0, self.bearing_noise_std)

        d["x"] = uav_pos[0] + noisy_range * math.cos(noisy_bearing)
        d["y"] = uav_pos[1] + noisy_range * math.sin(noisy_bearing)
        d["distance"] = noisy_range
        d["measured_range"] = noisy_range
        d["measured_bearing"] = noisy_bearing

    # ------------------------------------------------------------------
    # Task 7: field of view
    # ------------------------------------------------------------------
    def _heading(self, uav_id, uav_pos):
        """Approximate radar-pointing direction. The simulation has no
        explicit UAV heading/orientation state, so this uses the direction
        from the UAV to its own goal slot as a stand-in. Only matters when
        radar_field_of_view < 360."""
        gx, gy = self.sim.targets[uav_id]
        dx, dy = gx - uav_pos[0], gy - uav_pos[1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return 0.0
        return math.atan2(dy, dx)

    # ------------------------------------------------------------------
    # Task 7: radar-level dropout + confidence miscalibration
    # ------------------------------------------------------------------
    def _radar_dropout_fires(self):
        """Per-scan probability of a total radar blackout, independent of
        the base Perception model's own dropout_prob."""
        return self.radar_rng.random() < self.radar_dropout_probability

    def _apply_confidence_error(self, perceived):
        """Extra Gaussian confidence miscalibration applied at the
        radar-reporting stage, on top of whatever Perception already
        applied - the radar's own self-assessment of detection quality is
        itself imperfect."""
        if self.radar_confidence_error <= 0:
            return
        for d in perceived:
            if d.get("confidence") is not None:
                d["confidence"] = round(clamp(
                    d["confidence"] + self.radar_rng.gauss(0.0, self.radar_confidence_error),
                    0.0, 1.0), 3)

    # ------------------------------------------------------------------
    # Task 7: update-rate hold + latency buffer
    # ------------------------------------------------------------------
    def _get_delayed_scan(self, uav_id, t):
        """Returns the most recent (scan, dropout, pd_missed_ids) that's
        had time to 'arrive' by step t given radar_latency_steps of delay,
        dropping consumed/stale buffer entries so it stays bounded. Returns
        None if nothing has arrived yet (caller keeps whatever it already
        held)."""
        buf = self._scan_buffer[uav_id]
        cutoff = t - self.radar_latency_steps
        used = None
        used_idx = None
        for idx, (gen_t, scan, dropout, pd_missed_ids) in enumerate(buf):
            if gen_t <= cutoff:
                used = (scan, dropout, pd_missed_ids)
                used_idx = idx
            else:
                break
        if used_idx is not None:
            del buf[:used_idx + 1]
        return used

    # ------------------------------------------------------------------
    # Task 6: radar false alarm / clutter generation
    # ------------------------------------------------------------------
    def _poisson_sample(self, lam):
        """Knuth's algorithm, dependency-free. Returns a non-negative int
        drawn from Poisson(lam); 0 if lam <= 0."""
        if lam <= 0:
            return 0
        limit = math.exp(-lam)
        k = 0
        p = 1.0
        while True:
            k += 1
            p *= self.radar_rng.random()
            if p <= limit:
                return k - 1

    def _generate_clutter(self, uav_pos, heading=None, half_fov=None):
        """Generates this scan's confirmed radar clutter detections for one
        UAV, confined to the radar's [radar_min_range, radar_max_range]
        annulus and (Task 7) field-of-view sector if one is set. The number
        of *candidate* clutter returns is drawn from
        Poisson(radar_clutter_density); each candidate is independently
        confirmed as a reported false detection with probability
        radar_false_alarm_probability (P_FA). This runs every step for
        every scenario - it is not a special demo scenario, it is the
        radar's ordinary noise floor."""
        dets = []
        num_candidates = self._poisson_sample(self.clutter_density)
        for _ in range(num_candidates):
            if self.radar_rng.random() >= self.false_alarm_probability:
                continue  # candidate didn't cross the detection threshold this scan

            if half_fov is None:
                bearing = self.radar_rng.uniform(0.0, 2 * math.pi)
            else:
                bearing = heading + self.radar_rng.uniform(-half_fov, half_fov)

            # Uniform-in-annulus-area radius sampling so clutter isn't
            # artificially bunched near the inner or outer edge.
            lo2, hi2 = self.radar_min_range ** 2, self.radar_max_range ** 2
            rng_ = math.sqrt(self.radar_rng.uniform(lo2, hi2)) if hi2 > lo2 else self.radar_max_range

            x = uav_pos[0] + rng_ * math.cos(bearing)
            y = uav_pos[1] + rng_ * math.sin(bearing)
            confidence = round(clamp(self.radar_rng.uniform(0.2, 0.7), 0.0, 1.0), 3)

            self._clutter_counter += 1
            dets.append({
                "kind": "clutter",
                "id": f"clutter_{self._clutter_counter}",
                "x": x, "y": y,
                "distance": rng_,
                # random range/bearing, logged directly (not derived via
                # _apply_radar_noise, since clutter has no ground-truth
                # target underneath to noise a measurement of).
                "measured_range": rng_,
                "measured_bearing": bearing,
                # is_phantom=True routes it through Simulation's existing
                # steering/avoidance logic exactly like any other false
                # detection (see Simulation._steer), so unnecessary
                # avoidance/wrong-decision/response-time effects show up
                # without touching simple_swarm_sim.py at all.
                "is_phantom": True,
                "is_radar_clutter": True,
                "false_alarm_flag": True,
                "clutter_flag": True,
                "confidence": confidence,
                "true_confidence": confidence,
            })
        return dets

    # ------------------------------------------------------------------
    # Non-invasive instrumentation
    # ------------------------------------------------------------------
    def _patch_perception(self):
        """Wrap each UAV's Perception.process: call the original unchanged
        (so its own internal state - dropout timers, stochastic draws -
        stays consistent step to step), then, only on steps due for a radar
        scan update (Task 7's radar_update_rate), run the radar-specific
        pipeline: range/FOV gate, P_D roll (Task 5), clutter/false-alarm
        generation (Task 6), confidence error, and range/bearing noise. The
        result is pushed through a radar_latency_steps delay buffer;
        between scan-update steps, the most recently delivered scan is
        simply held and re-served, modeling a radar refresh rate slower
        than the simulation's own step rate."""
        for uav_id, perc in enumerate(self.sim.perception):
            original_process = perc.process

            def wrapped(true_detections, uav_pos, _uav_id=uav_id,
                        _orig=original_process, _perc=perc):
                true_snapshot = [dict(d) for d in true_detections]
                true_by_id = {d["id"]: d for d in true_snapshot}

                base_perceived = _orig(true_detections, uav_pos)

                t = self._current_t
                if t % self.radar_update_interval_steps == 0:
                    radar_dropout = self._radar_dropout_fires()
                    pd_missed_ids = []

                    if radar_dropout:
                        scan = []
                    else:
                        heading = None
                        half_fov = None
                        if self.radar_field_of_view < 360.0:
                            heading = self._heading(_uav_id, uav_pos)
                            half_fov = math.radians(self.radar_field_of_view) / 2.0

                        # Task 5 (P_D) + Task 7 (min/max range, FOV) gate,
                        # applied only to real (non-phantom) detections that
                        # survived the existing false_negative_rate/dropout
                        # model above - this is an independent, additional
                        # radar-level miss chance, not a replacement for it.
                        surviving = []
                        for d in base_perceived:
                            if d.get("is_phantom"):
                                surviving.append(d)
                                continue

                            true_d = true_by_id.get(d.get("id"))
                            true_range = true_d["distance"] if true_d is not None else None
                            if true_range is not None and (
                                    true_range > self.radar_max_range
                                    or true_range < self.radar_min_range):
                                pd_missed_ids.append(d.get("id"))
                                continue

                            if half_fov is not None and true_d is not None:
                                dx = true_d["x"] - uav_pos[0]
                                dy = true_d["y"] - uav_pos[1]
                                bearing = math.atan2(dy, dx)
                                if abs(_wrap_angle(bearing - heading)) > half_fov:
                                    pd_missed_ids.append(d.get("id"))
                                    continue

                            if self.radar_rng.random() < self.detection_probability:
                                surviving.append(d)
                            else:
                                pd_missed_ids.append(d.get("id"))
                        scan = surviving

                        for d in scan:
                            self._apply_radar_noise(d, uav_pos, true_by_id)

                        self._apply_confidence_error(scan)

                        # Task 6: radar false alarms / clutter - generated
                        # fresh every step, independent of the config-driven
                        # false_positive_rate "phantom" mechanism above.
                        scan = scan + self._generate_clutter(uav_pos, heading, half_fov)

                    self._scan_buffer[_uav_id].append((t, scan, radar_dropout, pd_missed_ids))

                delayed = self._get_delayed_scan(_uav_id, t)
                if delayed is not None:
                    (self._held_perceived[_uav_id], self._held_dropout[_uav_id],
                     self._held_pd_missed[_uav_id]) = delayed

                perceived_out = [dict(d) for d in self._held_perceived[_uav_id]]
                dropout_out = self._held_dropout[_uav_id] or _perc.last_dropout

                self._capture[_uav_id] = {
                    "true_dets": true_snapshot,
                    "perceived": perceived_out,  # overwritten again after fusion
                    "dropout": dropout_out,
                    "observer_pos": tuple(uav_pos),
                    "pd_missed_ids": self._held_pd_missed[_uav_id],
                }
                return perceived_out

            perc.process = wrapped

    def _patch_fusion(self):
        """Wrap Simulation._apply_fusion: call the original unchanged, then
        re-snapshot each UAV's detection list. Fusion can replace an
        individual UAV's obstacle detection with a shared/fused one, so the
        logged detection must be captured AFTER fusion runs - capturing
        before it (as Perception.process alone would) reports what the UAV
        would have seen without fusion, not what it actually acted on."""
        original_fusion = self.sim._apply_fusion

        def wrapped_fusion(raw_percepts, _orig=original_fusion):
            _orig(raw_percepts)
            for uav_id, dets in raw_percepts.items():
                if uav_id in self._capture:
                    self._capture[uav_id]["perceived"] = [dict(d) for d in dets]

        self.sim._apply_fusion = wrapped_fusion

    def _patch_step(self):
        """Wrap Simulation.step: call the original unchanged, but capture
        the current step index and each UAV's position before/after (used
        to derive velocities for radial-velocity/Doppler calculations)."""
        original_step = self.sim.step

        def wrapped_step(t, _orig=original_step):
            self._capture = {}
            self._current_t = t
            pos_before = {i: tuple(self.sim.pos[i]) for i in range(self.sim.num_uavs)}
            _orig(t)
            pos_after = {i: tuple(self.sim.pos[i]) for i in range(self.sim.num_uavs)}
            self._finalize_step(t, pos_before, pos_after)

        self.sim.step = wrapped_step

    # ------------------------------------------------------------------
    # Row construction
    # ------------------------------------------------------------------
    def _target_velocity(self, target_id, uav_vel):
        if target_id == "obstacle_0":
            return (0.0, 0.0)
        if target_id is not None and target_id.startswith("uav_"):
            j = int(target_id.split("_")[1])
            return uav_vel.get(j, (0.0, 0.0))
        return None

    def _make_row(self, t, uav_id, true_det, measured_det, observer_pos,
                  observer_vel, uav_vel, status,
                  false_alarm=False, missed=False, dropout=False,
                  radar_pd_miss=False, clutter=False):
        target_id = true_det["id"] if true_det is not None else (
            measured_det.get("id") if measured_det is not None and measured_det.get("id") != "phantom"
            else None
        )
        if measured_det is not None and measured_det.get("id") == "phantom":
            target_id = f"phantom_t{t}_uav{uav_id}"

        false_alarm_source = None
        if false_alarm:
            false_alarm_source = "radar_clutter" if clutter else "config_false_positive"

        true_x = true_det["x"] if true_det is not None else None
        true_y = true_det["y"] if true_det is not None else None
        target_vel = self._target_velocity(
            true_det["id"] if true_det is not None else None, uav_vel)

        if true_det is not None:
            true_range, true_bearing, true_radial_vel = _range_bearing_radial(
                observer_pos, observer_vel, (true_x, true_y), target_vel)
        else:
            true_range = true_bearing = true_radial_vel = None

        detected_x = detected_y = None
        measured_range = measured_bearing = measured_radial_vel = None
        confidence = None

        if measured_det is not None:
            detected_x = measured_det["x"]
            detected_y = measured_det["y"]
            confidence = measured_det.get("confidence")

            # Reuse the exact range/bearing that was noised in
            # _apply_radar_noise, so logged values match what actually
            # drove detected_x/y and the UAV's decision.
            measured_range = measured_det.get("measured_range")
            measured_bearing = measured_det.get("measured_bearing")

            # Radial velocity is its own measurement channel: projected
            # along the noisy line-of-sight, using true target kinematics,
            # plus its own independent noise. No coherent Doppler for
            # phantoms (no real target underneath).
            if (target_id is not None
                    and not target_id.startswith("phantom_")
                    and not target_id.startswith("clutter_")):
                _, _, base_radial = _range_bearing_radial(
                    observer_pos, observer_vel, (detected_x, detected_y), target_vel)
                if base_radial is not None:
                    measured_radial_vel = base_radial + self.radar_rng.gauss(
                        0.0, self.radial_velocity_noise_std)

        return {
            "time_step": t,
            "radar_id": uav_id,
            "target_id": target_id,
            "true_target_x": round(true_x, 4) if true_x is not None else None,
            "true_target_y": round(true_y, 4) if true_y is not None else None,
            "target_velocity_x": round(target_vel[0], 4) if target_vel is not None else None,
            "target_velocity_y": round(target_vel[1], 4) if target_vel is not None else None,
            "true_range": round(true_range, 4) if true_range is not None else None,
            "true_bearing": round(true_bearing, 5) if true_bearing is not None else None,
            "true_radial_velocity": round(true_radial_vel, 4) if true_radial_vel is not None else None,
            "measured_range": round(measured_range, 4) if measured_range is not None else None,
            "measured_bearing": round(measured_bearing, 5) if measured_bearing is not None else None,
            "measured_radial_velocity": round(measured_radial_vel, 4) if measured_radial_vel is not None else None,
            "detected_x": round(detected_x, 4) if detected_x is not None else None,
            "detected_y": round(detected_y, 4) if detected_y is not None else None,
            "confidence_score": confidence,
            "detection_status": status,
            "false_alarm_flag": bool(false_alarm),
            "missed_detection_flag": bool(missed),
            "clutter_flag": bool(clutter),
            "false_alarm_source": false_alarm_source,
            "dropout_flag": bool(dropout),
            "radar_pd_miss_flag": bool(radar_pd_miss),
        }

    def _finalize_step(self, t, pos_before, pos_after):
        uav_vel = {}
        for i in range(self.sim.num_uavs):
            vx = (pos_after[i][0] - pos_before[i][0]) / self.dt
            vy = (pos_after[i][1] - pos_before[i][1]) / self.dt
            uav_vel[i] = (vx, vy)

        for uav_id in range(self.sim.num_uavs):
            if uav_id not in self._capture:
                # UAV had already reached its goal -> Perception was never
                # called for it this step, so there's nothing to report.
                continue

            cap = self._capture[uav_id]
            observer_pos = cap["observer_pos"]
            observer_vel = uav_vel[uav_id]
            true_dets = cap["true_dets"]
            perceived = cap["perceived"]  # final (post-fusion) detections
            dropout = cap["dropout"]

            if dropout:
                if true_dets:
                    for d in true_dets:
                        self.rows.append(self._make_row(
                            t, uav_id, d, None, observer_pos, observer_vel,
                            uav_vel, status="dropout", missed=True, dropout=True))
                else:
                    self.rows.append(self._make_row(
                        t, uav_id, None, None, observer_pos, observer_vel,
                        uav_vel, status="dropout", missed=True, dropout=True))
                continue

            # A detection is a false alarm if it's the legacy config-driven
            # "phantom" (id == "phantom") or a Task 6 radar clutter point
            # (is_radar_clutter). Both are excluded from perceived_by_id so
            # they never get mistaken for a real target's detection below.
            false_alarm_dets = [d for d in perceived
                                 if d.get("id") == "phantom" or d.get("is_radar_clutter")]
            false_alarm_ids = {d["id"] for d in false_alarm_dets}
            perceived_by_id = {d["id"]: d for d in perceived if d["id"] not in false_alarm_ids}

            pd_missed_ids = set(cap.get("pd_missed_ids", []))

            # A target is "detected" if it ended up in the final (post-
            # fusion) detection set for this UAV - whether that's because
            # the UAV's own sensor caught it, or because fusion supplied it
            # after an individual miss. Otherwise it's "missed" - which
            # includes Task 5's radar P_D gate removing it upstream.
            for d in true_dets:
                meas = perceived_by_id.get(d["id"])
                if meas is not None:
                    self.rows.append(self._make_row(
                        t, uav_id, d, meas, observer_pos, observer_vel,
                        uav_vel, status="detected"))
                else:
                    self.rows.append(self._make_row(
                        t, uav_id, d, None, observer_pos, observer_vel,
                        uav_vel, status="missed", missed=True,
                        radar_pd_miss=(d["id"] in pd_missed_ids)))

            for fd in false_alarm_dets:
                self.rows.append(self._make_row(
                    t, uav_id, None, fd, observer_pos, observer_vel,
                    uav_vel, status="false_alarm", false_alarm=True,
                    clutter=bool(fd.get("is_radar_clutter"))))

    # ------------------------------------------------------------------
    # Driver
    # ------------------------------------------------------------------
    def run(self):
        t = 0
        for t in range(self.sim.max_steps):
            self.sim.step(t)
            if all(self.sim.reached_goal):
                break
        return self.rows


def main():
    parser = argparse.ArgumentParser(
        description="Generate radar-like detection logs from the UAV swarm simulation")
    parser.add_argument("--config", default="simulation_config.json")
    parser.add_argument("--scenario", default=None, help="Run just one scenario instead of all")
    parser.add_argument("--log", default="logs/radar_log.csv")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    scenario_names = [args.scenario] if args.scenario else list(config["scenarios"].keys())

    all_rows = []
    for name in scenario_names:
        model = RadarLikeModel(config, name)
        rows = model.run()
        for row in rows:
            row_with_scenario = {"scenario": name}
            row_with_scenario.update(row)
            all_rows.append(row_with_scenario)
        print(f"{name}: {len(rows)} radar rows generated")

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