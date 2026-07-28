"""
radar_model_validation.py

Validates the core radar-domain equations and behaviors in
radar_like_model.py against controlled, hand-computable cases and known
statistical properties - not the full swarm simulation (which
RadarLikeModel wraps), just the radar-domain math and the radar-level
scan/dropout/latency machinery itself.

Covers:
  - exact range / exact bearing / exact radial velocity
  - positive radial velocity / negative radial velocity / stationary target
  - noisy range / noisy bearing / noisy radial velocity
  - Cartesian (x/y) reconstruction
  - covariance dimensions / covariance positivity
  - P_D (probability of detection) behavior
  - P_FA (probability of false alarm) behavior
  - Poisson clutter behavior
  - range-dependent SNR
  - latency
  - dropout
  - timestamp behavior
  - maximum sensing range

Two testing strategies are used:
  1. "Bare model" checks (make_bare_model): construct a RadarLikeModel via
     object.__new__ (skipping __init__, which needs a full Simulation) and
     set only the attributes the specific method under test reads. This
     directly exercises the real range/bearing/noise/uncertainty/clutter
     methods with controlled inputs.
  2. "Full model" integration checks (make_full_model / _FakeSim): a small
     stand-in for simple_swarm_sim.Simulation providing only the surface
     RadarLikeModel.__init__ and its three patched hooks
     (Perception.process, Simulation._apply_fusion, Simulation.step)
     actually touch. This lets max-range/min-range gating, radar dropout,
     and the scan-generation-timestamp/latency buffer be exercised through
     the REAL RadarLikeModel._patch_perception closure, with fully
     controlled UAV/target geometry instead of full swarm dynamics.

Every check prints PASS/FAIL as it runs. Results (test name, configuration,
expected result, actual result, tolerance, PASS/FAIL, correction required)
are written to results/radar_model_validation_results.md.

Note: simple_swarm_sim.py (and therefore radar_like_model.py, which imports
Simulation/dist/clamp from it) imports
`dependability.perception_quality_monitor` unconditionally at module load
time. That module was not part of this validation task's inputs, so a
minimal stub (dependability/perception_quality_monitor.py) is provided
alongside this file purely to satisfy the import chain. No test here
exercises PerceptionQualityMonitor: bare-model checks never construct a
Simulation at all, and the _FakeSim used for integration checks never
constructs the real Simulation class either.

Usage:
    python radar_model_validation.py
"""

import math
import os
import random
import statistics
import sys

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from models.radar_like_model import (
    RadarLikeModel,
    _range_bearing_radial,
    _wrap_angle,
    _apply_doppler_aliasing,
)
from simple_swarm_sim import Perception
import models.radar_like_model as _rlm


# ======================================================================
# Lightweight self-contained result tracker (validation_common.py was not
# supplied with this task's inputs, so this is implemented inline).
# ======================================================================
class Checker:
    def __init__(self):
        self.records = []
        self.n_pass = 0
        self.n_fail = 0

    def close(self, a, b, tol=1e-6):
        if a is None or b is None:
            return a == b
        return abs(a - b) <= tol

    def record(self, task, description, passed, config="", expected="",
               actual="", tolerance="", correction=None):
        status = "PASS" if passed else "FAIL"
        if correction is None:
            correction = "" if passed else (
                "Actual result fell outside the expected value/tolerance "
                "above - inspect the corresponding radar_like_model.py "
                "method and re-run this check after any fix."
            )
        self.records.append(dict(
            task=task, description=description, status=status,
            config=config, expected=expected, actual=actual,
            tolerance=tolerance, correction=correction,
        ))
        if passed:
            self.n_pass += 1
        else:
            self.n_fail += 1
        line = f"[{status}] {task}: {description}"
        if not passed:
            line += f"  (expected={expected!r} actual={actual!r} tol={tolerance!r})"
        print(line)
        return passed

    def print_summary(self):
        total = self.n_pass + self.n_fail
        print("=" * 78)
        print(f"TOTAL CHECKS: {total}   PASS: {self.n_pass}   FAIL: {self.n_fail}")
        print("=" * 78)
        return self.n_fail

    def write_markdown(self, path, title, intro=""):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        def esc(s):
            return str(s).replace("|", "\\|").replace("\n", " ")

        lines = [f"# {title}", ""]
        if intro:
            lines += [intro, ""]
        lines += [
            f"**Total checks:** {self.n_pass + self.n_fail}  |  "
            f"**PASS:** {self.n_pass}  |  **FAIL:** {self.n_fail}",
            "",
            "| # | Test Name | Configuration | Expected Result | Actual Result | Tolerance | Result | Correction Required |",
            "|---|---|---|---|---|---|---|---|",
        ]
        for i, r in enumerate(self.records, 1):
            name = f"{r['task']}: {r['description']}"
            lines.append(
                f"| {i} | {esc(name)} | {esc(r['config'])} | {esc(r['expected'])} | "
                f"{esc(r['actual'])} | {esc(r['tolerance'])} | {r['status']} | "
                f"{esc(r['correction']) if r['correction'] else '-'} |"
            )
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")


_checker = Checker()


def check(task, description, condition, config="", expected="", actual="",
          tolerance="", correction=None):
    return _checker.record(task, description, condition, config=config,
                            expected=expected, actual=actual,
                            tolerance=tolerance, correction=correction)


def close(a, b, tol=1e-6):
    return _checker.close(a, b, tol)


# ======================================================================
# Strategy 1: bare RadarLikeModel (no Simulation involved at all)
# ======================================================================
def make_bare_model(**overrides):
    """Builds a RadarLikeModel instance without running __init__ (which
    requires a full Simulation), setting only the attributes the methods
    under test actually read."""
    m = object.__new__(RadarLikeModel)
    defaults = dict(
        range_noise_std=RadarLikeModel.DEFAULT_RANGE_NOISE_STD,
        bearing_noise_std=RadarLikeModel.DEFAULT_BEARING_NOISE_STD,
        radial_velocity_noise_std=RadarLikeModel.DEFAULT_RADIAL_VELOCITY_NOISE_STD,
        detection_probability=RadarLikeModel.DEFAULT_RADAR_DETECTION_PROBABILITY,
        false_alarm_probability=RadarLikeModel.DEFAULT_RADAR_FALSE_ALARM_PROBABILITY,
        clutter_lambda=RadarLikeModel.DEFAULT_RADAR_CLUTTER_DENSITY,
        clutter_distribution=RadarLikeModel.DEFAULT_CLUTTER_DISTRIBUTION,
        clutter_range_min=0.0,
        clutter_range_max=50.0,
        reference_snr_db=RadarLikeModel.DEFAULT_REFERENCE_SNR_DB,
        snr_exponent=RadarLikeModel.DEFAULT_SNR_EXPONENT,
        reference_range=50.0,
        environmental_condition=RadarLikeModel.DEFAULT_ENVIRONMENTAL_CONDITION,
        radar_reliability_state=RadarLikeModel.DEFAULT_RELIABILITY_STATE,
        radar_mode=RadarLikeModel.DEFAULT_RADAR_MODE,
        radar_clutter_confidence_bias=0.0,
        radar_confidence_bias=0.0,
        radar_confidence_error=0.0,
        doppler_aliasing_enabled=RadarLikeModel.DEFAULT_DOPPLER_ALIASING_ENABLED,
        max_unambiguous_radial_velocity=RadarLikeModel.DEFAULT_MAX_UNAMBIGUOUS_RV,
        radar_latency_steps=RadarLikeModel.DEFAULT_LATENCY_STEPS,
        radar_dropout_probability=RadarLikeModel.DEFAULT_DROPOUT_PROBABILITY,
        radar_max_range=15.0,
        radar_min_range=0.0,
        dt=0.2,
    )
    defaults.update(overrides)
    for k, v in defaults.items():
        setattr(m, k, v)
    m._env_factors = RadarLikeModel.ENV_FACTORS[m.environmental_condition]
    m._reliability_factors = RadarLikeModel.RELIABILITY_FACTORS[m.radar_reliability_state]
    m._mode_factors = RadarLikeModel.RADAR_MODES[m.radar_mode]
    m.radar_rng = random.Random(12345)
    m._clutter_counter = 0
    m._scan_buffer = {0: []}
    return m


# ======================================================================
# Strategy 2: full RadarLikeModel wired to a minimal fake Simulation, so
# the real _patch_perception closure (max/min-range gate, radar dropout,
# scan-timestamp/latency buffer) runs unmodified against controlled
# geometry.
# ======================================================================
class _FakeSim:
    """Minimal stand-in for simple_swarm_sim.Simulation exposing only what
    RadarLikeModel.__init__ and its three patched hooks touch: .scn,
    .num_uavs, .sensor_range, .targets, .pos, .perception (real Perception
    instances), .step(t), ._apply_fusion(raw_percepts). Test code sets
    .true_dets_by_uav before each .step(t) call to control exactly what
    each UAV's radar sees this scan."""

    def __init__(self, scn, num_uavs=1, sensor_range=15.0, seed=7):
        self.scn = scn
        self.num_uavs = num_uavs
        self.sensor_range = sensor_range
        self.targets = [(0.0, 0.0)] * num_uavs
        self.pos = [[0.0, 0.0] for _ in range(num_uavs)]
        self.perception = [Perception({}, random.Random(seed + i), sensor_range)
                            for i in range(num_uavs)]
        self.true_dets_by_uav = {i: [] for i in range(num_uavs)}

    def step(self, t):
        for i in range(self.num_uavs):
            self.perception[i].process(self.true_dets_by_uav.get(i, []), tuple(self.pos[i]))
        self._apply_fusion({})

    def _apply_fusion(self, raw_percepts):
        pass


def make_full_model(scn_overrides, num_uavs=1, sensor_range=15.0, dt=0.2, seed=42):
    """Builds a real RadarLikeModel with its self.sim replaced by a
    _FakeSim, so __init__'s config resolution and the three patched hooks
    run as actual radar_like_model.py code against controlled geometry."""
    scn = dict(scn_overrides)
    config = {"sim": {"dt": dt, "seed": seed}, "radar": {},
              "sensing": {"sensor_range": sensor_range}}
    fake_sim = _FakeSim(scn, num_uavs=num_uavs, sensor_range=sensor_range)
    original_simulation_cls = _rlm.Simulation
    _rlm.Simulation = lambda cfg, name: fake_sim
    try:
        model = RadarLikeModel(config, "validation_test_scenario")
    finally:
        _rlm.Simulation = original_simulation_cls
    return model, fake_sim


CLEAN_SCN = dict(
    radar_detection_probability=1.0, radar_false_alarm_probability=0.0,
    clutter_lambda=0.0, radar_dropout_probability=0.0,
    radar_range_noise_std=0.0, radar_bearing_noise_std=0.0,
    false_negative_rate=0.0, position_noise_std=0.0, dropout_prob=0.0,
    confidence_error_level=0.0,
)


# ---------------------------------------------------------------------
# 1. Exact range
# ---------------------------------------------------------------------
def test_exact_range():
    task = "exact_range"
    cfg = "_range_bearing_radial(observer=(0,0), target=(3,4))"
    rng, _, _ = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (3.0, 4.0), None)
    check(task, "3-4-5 right triangle gives range=5.0", close(rng, 5.0),
          config=cfg, expected="5.0", actual=f"{rng}", tolerance="1e-6")

    cfg2 = "_range_bearing_radial(observer=(10,10), target=(13,14))"
    rng2, _, _ = _range_bearing_radial((10.0, 10.0), (0.0, 0.0), (13.0, 14.0), None)
    check(task, "range is translation-invariant (offset observer, same displacement)",
          close(rng2, 5.0), config=cfg2, expected="5.0", actual=f"{rng2}", tolerance="1e-6")

    cfg3 = "_range_bearing_radial(observer=(5,5), target=(5,5))"
    rng3, _, _ = _range_bearing_radial((5.0, 5.0), (0.0, 0.0), (5.0, 5.0), None)
    check(task, "coincident observer/target gives range=0.0", close(rng3, 0.0),
          config=cfg3, expected="0.0", actual=f"{rng3}", tolerance="1e-6")

    cfg4 = "_range_bearing_radial(observer=(0,0), target=(15,0)) at radar_max_range=15"
    rng4, _, _ = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (15.0, 0.0), None)
    check(task, "range exactly at a hypothetical boundary value computes exactly (15.0)",
          close(rng4, 15.0), config=cfg4, expected="15.0", actual=f"{rng4}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 2. Exact bearing
# ---------------------------------------------------------------------
def test_exact_bearing():
    task = "exact_bearing"
    cases = [
        ((1.0, 0.0), 0.0, "due east -> 0 rad"),
        ((0.0, 1.0), math.pi / 2, "due north -> +pi/2 rad"),
        ((-1.0, 0.0), math.pi, "due west -> +/-pi rad"),
        ((0.0, -1.0), -math.pi / 2, "due south -> -pi/2 rad"),
        ((1.0, 1.0), math.pi / 4, "northeast diagonal -> +pi/4 rad"),
    ]
    for (dx, dy), expected, label in cases:
        _, bearing, _ = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (dx, dy), None)
        ok = close(bearing, expected) or close(abs(bearing), abs(expected))
        check(task, label, ok, config=f"target offset=({dx},{dy})",
              expected=f"{expected:.4f} rad", actual=f"{bearing:.4f} rad", tolerance="1e-6")

    wrapped = _wrap_angle(3 * math.pi)
    check(task, "_wrap_angle folds 3*pi into (-pi, pi]", close(abs(wrapped), math.pi, tol=1e-6),
          config="_wrap_angle(3*pi)", expected="+/-pi", actual=f"{wrapped:.4f}", tolerance="1e-6")

    wrapped2 = _wrap_angle(-3 * math.pi)
    check(task, "_wrap_angle folds -3*pi into (-pi, pi]", close(abs(wrapped2), math.pi, tol=1e-6),
          config="_wrap_angle(-3*pi)", expected="+/-pi", actual=f"{wrapped2:.4f}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 3. Exact radial velocity
# ---------------------------------------------------------------------
def test_exact_radial_velocity():
    task = "exact_radial_velocity"
    # Non-axis-aligned target/velocity: only the component of velocity
    # along the observer->target line-of-sight should register.
    # Target at (3,4) (range=5, unit LOS=(0.6,0.8)); velocity (5,0) ->
    # projected radial velocity = 5*0.6 = 3.0
    _, _, rv = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (3.0, 4.0), (5.0, 0.0))
    check(task, "oblique velocity projects onto line-of-sight correctly (3.0)",
          close(rv, 3.0), config="target=(3,4), target_vel=(5,0)",
          expected="3.0", actual=f"{rv}", tolerance="1e-6")

    # Both observer and target moving: relative velocity is what matters.
    _, _, rv2 = _range_bearing_radial((0.0, 0.0), (1.0, 0.0), (10.0, 0.0), (4.0, 0.0))
    check(task, "relative (target-observer) velocity used, not absolute (3.0)",
          close(rv2, 3.0), config="observer_vel=(1,0), target=(10,0), target_vel=(4,0)",
          expected="3.0", actual=f"{rv2}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 4. Positive radial velocity
# ---------------------------------------------------------------------
def test_positive_radial_velocity():
    task = "positive_radial_velocity"
    m = make_bare_model(radial_velocity_noise_std=0.0, range_noise_std=0.0, bearing_noise_std=0.0)
    true_det = {"id": "uav_1", "x": 10.0, "y": 0.0}
    measured_det = {"id": "uav_1", "x": 10.0, "y": 0.0, "confidence": 0.9}
    uav_vel = {0: (0.0, 0.0), 1: (3.0, 0.0)}  # target moving directly away
    row = m._make_row(0, 0, true_det, measured_det, (0.0, 0.0), (0.0, 0.0), uav_vel, status="detected")
    check(task, "target moving directly away from observer -> true_radial_velocity = +3.0",
          close(row["true_radial_velocity"], 3.0),
          config="observer=(0,0) static, target=(10,0) moving vel=(3,0)",
          expected="+3.0", actual=f"{row['true_radial_velocity']}", tolerance="1e-6")
    check(task, "...and measured_radial_velocity matches (zero noise)",
          close(row["measured_radial_velocity"], 3.0, tol=1e-6),
          config="radial_velocity_noise_std=0.0",
          expected="+3.0", actual=f"{row['measured_radial_velocity']}", tolerance="1e-6")

    # Oblique receding case.
    true_det2 = {"id": "uav_1", "x": 8.0, "y": 6.0}  # range 10
    measured_det2 = {"id": "uav_1", "x": 8.0, "y": 6.0, "confidence": 0.9}
    uav_vel2 = {0: (0.0, 0.0), 1: (8.0, 6.0)}  # moving straight along its own bearing -> speed 10 fully radial
    row2 = m._make_row(0, 0, true_det2, measured_det2, (0.0, 0.0), (0.0, 0.0), uav_vel2, status="detected")
    check(task, "target receding directly along its own bearing -> full speed is radial (+10.0)",
          close(row2["true_radial_velocity"], 10.0),
          config="target=(8,6) range=10, vel=(8,6) i.e. along LOS",
          expected="+10.0", actual=f"{row2['true_radial_velocity']}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 5. Negative radial velocity
# ---------------------------------------------------------------------
def test_negative_radial_velocity():
    task = "negative_radial_velocity"
    m = make_bare_model(radial_velocity_noise_std=0.0, range_noise_std=0.0, bearing_noise_std=0.0)
    true_det = {"id": "uav_1", "x": 10.0, "y": 0.0}
    measured_det = {"id": "uav_1", "x": 10.0, "y": 0.0, "confidence": 0.9}
    uav_vel = {0: (0.0, 0.0), 1: (-3.0, 0.0)}  # target moving directly toward observer
    row = m._make_row(0, 0, true_det, measured_det, (0.0, 0.0), (0.0, 0.0), uav_vel, status="detected")
    check(task, "target moving directly toward observer -> true_radial_velocity = -3.0",
          close(row["true_radial_velocity"], -3.0),
          config="observer=(0,0) static, target=(10,0) moving vel=(-3,0)",
          expected="-3.0", actual=f"{row['true_radial_velocity']}", tolerance="1e-6")
    check(task, "...and measured_radial_velocity matches (zero noise)",
          close(row["measured_radial_velocity"], -3.0, tol=1e-6),
          config="radial_velocity_noise_std=0.0",
          expected="-3.0", actual=f"{row['measured_radial_velocity']}", tolerance="1e-6")

    # Purely tangential motion should contribute zero radial velocity even
    # though the target itself is moving (not stationary).
    true_det2 = {"id": "uav_1", "x": 10.0, "y": 0.0}
    measured_det2 = {"id": "uav_1", "x": 10.0, "y": 0.0, "confidence": 0.9}
    uav_vel2 = {0: (0.0, 0.0), 1: (0.0, 5.0)}
    row2 = m._make_row(0, 0, true_det2, measured_det2, (0.0, 0.0), (0.0, 0.0), uav_vel2, status="detected")
    check(task, "purely tangential target motion gives radial_velocity = 0.0 (not negative)",
          close(row2["true_radial_velocity"], 0.0),
          config="target=(10,0), vel=(0,5) i.e. perpendicular to LOS",
          expected="0.0", actual=f"{row2['true_radial_velocity']}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 6. Stationary target
# ---------------------------------------------------------------------
def test_stationary_target():
    task = "stationary_target"
    m = make_bare_model(radial_velocity_noise_std=0.0, range_noise_std=0.0, bearing_noise_std=0.0)

    # Genuinely zero-velocity target (not None) -> radial velocity exactly 0.
    true_det = {"id": "uav_1", "x": 7.0, "y": 3.0}
    measured_det = {"id": "uav_1", "x": 7.0, "y": 3.0, "confidence": 0.9}
    uav_vel = {0: (0.0, 0.0), 1: (0.0, 0.0)}
    row = m._make_row(0, 0, true_det, measured_det, (0.0, 0.0), (0.0, 0.0), uav_vel, status="detected")
    check(task, "zero-velocity target gives true_radial_velocity = 0.0",
          close(row["true_radial_velocity"], 0.0),
          config="target=(7,3) vel=(0,0), observer static",
          expected="0.0", actual=f"{row['true_radial_velocity']}", tolerance="1e-6")
    check(task, "...and measured_radial_velocity = 0.0 (zero noise)",
          close(row["measured_radial_velocity"], 0.0),
          config="radial_velocity_noise_std=0.0",
          expected="0.0", actual=f"{row['measured_radial_velocity']}", tolerance="1e-6")

    # A stationary target with a moving observer: relative motion is
    # nonzero even though the target itself never moves.
    uav_vel2 = {0: (2.0, 0.0), 1: (0.0, 0.0)}
    row2 = m._make_row(0, 0, true_det, measured_det, (0.0, 0.0), (2.0, 0.0), uav_vel2, status="detected")
    check(task, "stationary target + moving observer gives nonzero relative radial velocity",
          row2["true_radial_velocity"] is not None and not close(row2["true_radial_velocity"], 0.0, tol=1e-9),
          config="target=(7,3) vel=(0,0), observer moving vel=(2,0)",
          expected="nonzero", actual=f"{row2['true_radial_velocity']}", tolerance="n/a (nonzero check)")

    # Static obstacle (id="obstacle_0"): the model has a built-in
    # always-stationary kinematic model for obstacles, regardless of
    # whatever uav_vel dict is passed in.
    true_det3 = {"id": "obstacle_0", "x": 5.0, "y": 0.0}
    measured_det3 = {"id": "obstacle_0", "x": 5.0, "y": 0.0, "confidence": 0.9}
    row3 = m._make_row(0, 0, true_det3, measured_det3, (0.0, 0.0), (0.0, 0.0), {}, status="detected")
    check(task, "id='obstacle_0' is always treated as stationary (radial_velocity=0.0)",
          close(row3["true_radial_velocity"], 0.0),
          config="target_id='obstacle_0', at (5,0)",
          expected="0.0", actual=f"{row3['true_radial_velocity']}", tolerance="1e-6")

    # Unrecognized target id: no kinematic model exists for it, so
    # velocity (and therefore radial velocity) is documented as None
    # rather than silently defaulting to some guessed value.
    true_det4 = {"id": "clutter_1", "x": 5.0, "y": 0.0}
    row4 = m._make_row(0, 0, true_det4, None, (0.0, 0.0), (0.0, 0.0), {}, status="missed")
    check(task, "unrecognized target id has no kinematic model -> true_radial_velocity is None (documented, not a bug)",
          row4["true_radial_velocity"] is None,
          config="target_id='clutter_1' (not 'obstacle_0' or 'uav_N')",
          expected="None", actual=f"{row4['true_radial_velocity']}", tolerance="n/a (None check)")


# ---------------------------------------------------------------------
# 7/8. Noisy range and noisy bearing
# ---------------------------------------------------------------------
def test_noisy_range_and_bearing():
    m = make_bare_model(range_noise_std=0.0, bearing_noise_std=0.0, radial_velocity_noise_std=0.0)
    uav_pos = (0.0, 0.0)
    true_by_id = {"t1": {"x": 6.0, "y": 8.0}}
    d = {"id": "t1", "x": 6.0, "y": 8.0}
    m._apply_radar_noise(d, uav_pos, true_by_id)
    check("noisy_range", "zero-noise measured_range equals true range (6-8-10 triangle)",
          close(d["measured_range"], 10.0, tol=1e-6),
          config="range_noise_std=0.0, target=(6,8)", expected="10.0",
          actual=f"{d['measured_range']}", tolerance="1e-6")
    expected_bearing = math.atan2(8.0, 6.0)
    check("noisy_bearing", "zero-noise measured_bearing equals true bearing",
          close(d["measured_bearing"], expected_bearing, tol=1e-6),
          config="bearing_noise_std=0.0, target=(6,8)", expected=f"{expected_bearing:.4f}",
          actual=f"{d['measured_bearing']:.4f}", tolerance="1e-6")

    m2 = make_bare_model(range_noise_std=2.0, bearing_noise_std=math.radians(5.0),
                          radial_velocity_noise_std=0.0)
    range_samples, bearing_samples = [], []
    for _ in range(4000):
        d2 = {"id": "t1", "x": 6.0, "y": 8.0}
        m2._apply_radar_noise(d2, uav_pos, true_by_id)
        range_samples.append(d2["measured_range"])
        bearing_samples.append(d2["measured_bearing"])
    range_std = statistics.pstdev(range_samples)
    bearing_std = statistics.pstdev(bearing_samples)
    check("noisy_range", "sampled range-noise std matches configured range_noise_std (2.0)",
          abs(range_std - 2.0) < 0.15,
          config="range_noise_std=2.0, n=4000 trials", expected="~2.0",
          actual=f"{range_std:.3f}", tolerance="0.15")
    check("noisy_bearing", "sampled bearing-noise std matches configured bearing_noise_std (0.0873 rad)",
          abs(bearing_std - math.radians(5.0)) < 0.01,
          config="bearing_noise_std=5deg, n=4000 trials", expected="~0.0873 rad",
          actual=f"{bearing_std:.4f} rad", tolerance="0.01 rad")
    check("noisy_range", "noisy range is floored at 0.05 (never non-positive)",
          all(r >= 0.05 for r in range_samples),
          config="range_noise_std=2.0, n=4000 trials", expected=">= 0.05",
          actual=f"min={min(range_samples):.4f}", tolerance="n/a (hard floor)")

    # Range/bearing noise should scale with the model's own reported
    # variance under degraded conditions (storm), not stay fixed.
    m3 = make_bare_model(range_noise_std=1.0, bearing_noise_std=math.radians(2.0),
                          radial_velocity_noise_std=0.0, environmental_condition="storm")
    range_samples3 = []
    for _ in range(3000):
        d3 = {"id": "t1", "x": 6.0, "y": 8.0}
        m3._apply_radar_noise(d3, uav_pos, true_by_id)
        range_samples3.append(d3["measured_range"])
    storm_std = statistics.pstdev(range_samples3)
    expected_storm_std = math.sqrt(m3._measurement_uncertainty(10.0)["range_variance"])
    check("noisy_range", "range-noise std under 'storm' matches the model's own reported range_variance",
          abs(storm_std - expected_storm_std) < 0.1,
          config="range_noise_std=1.0, environmental_condition='storm', n=3000",
          expected=f"~{expected_storm_std:.3f}", actual=f"{storm_std:.3f}", tolerance="0.1")


# ---------------------------------------------------------------------
# 9. Noisy radial velocity
# ---------------------------------------------------------------------
def test_noisy_radial_velocity():
    task = "noisy_radial_velocity"
    m = make_bare_model(radial_velocity_noise_std=0.4, range_noise_std=0.0, bearing_noise_std=0.0)
    true_det = {"id": "uav_1", "x": 10.0, "y": 0.0}
    measured_det = {"id": "uav_1", "x": 10.0, "y": 0.0, "confidence": 0.9}
    uav_vel = {0: (0.0, 0.0), 1: (5.0, 0.0)}
    samples = []
    for _ in range(4000):
        row = m._make_row(0, 0, true_det, dict(measured_det), (0.0, 0.0), (0.0, 0.0), uav_vel, status="detected")
        samples.append(row["measured_radial_velocity"])
    sample_mean = statistics.mean(samples)
    sample_std = statistics.pstdev(samples)
    expected_var = m._measurement_uncertainty(10.0)["radial_velocity_variance"]
    expected_std = math.sqrt(expected_var)
    check(task, "sampled measured_radial_velocity mean matches true radial velocity (+5.0)",
          abs(sample_mean - 5.0) < 0.05,
          config="radial_velocity_noise_std=0.4, target vel=(5,0), n=4000",
          expected="~5.0", actual=f"{sample_mean:.4f}", tolerance="0.05")
    check(task, "sampled measured_radial_velocity std matches the model's own reported radial_velocity_variance",
          abs(sample_std - expected_std) < 0.03,
          config="radial_velocity_noise_std=0.4, n=4000",
          expected=f"~{expected_std:.4f}", actual=f"{sample_std:.4f}", tolerance="0.03")


# ---------------------------------------------------------------------
# 10. Cartesian (x/y) reconstruction
# ---------------------------------------------------------------------
def test_cartesian_reconstruction():
    task = "cartesian_reconstruction"
    m = make_bare_model(range_noise_std=0.0, bearing_noise_std=0.0, radial_velocity_noise_std=0.0)
    uav_pos = (10.0, -5.0)
    true_by_id = {"t1": {"x": 13.0, "y": -1.0}}  # dx=3, dy=4 -> range 5
    d = {"id": "t1", "x": 13.0, "y": -1.0}
    m._apply_radar_noise(d, uav_pos, true_by_id)
    check(task, "zero-noise reconstructed x/y matches true target position exactly",
          close(d["x"], 13.0, tol=1e-6) and close(d["y"], -1.0, tol=1e-6),
          config="observer=(10,-5), target=(13,-1), zero noise",
          expected="(13.0, -1.0)", actual=f"({d['x']:.4f}, {d['y']:.4f})", tolerance="1e-6")

    m2 = make_bare_model(range_noise_std=1.5, bearing_noise_std=math.radians(3.0),
                          radial_velocity_noise_std=0.0)
    d2 = {"id": "t1", "x": 13.0, "y": -1.0}
    m2._apply_radar_noise(d2, uav_pos, true_by_id)
    expected_x = uav_pos[0] + d2["measured_range"] * math.cos(d2["measured_bearing"])
    expected_y = uav_pos[1] + d2["measured_range"] * math.sin(d2["measured_bearing"])
    check(task, "noisy x/y = uav_pos + measured_range*(cos,sin(measured_bearing)) (self-consistent)",
          close(d2["x"], expected_x, tol=1e-9) and close(d2["y"], expected_y, tol=1e-9),
          config="range_noise_std=1.5, bearing_noise_std=3deg",
          expected=f"({expected_x:.4f}, {expected_y:.4f})",
          actual=f"({d2['x']:.4f}, {d2['y']:.4f})", tolerance="1e-9")

    # Round-trip: reconstructed range/bearing back out of x/y should equal
    # the measured_range/measured_bearing that produced them.
    rt_range = math.hypot(d2["x"] - uav_pos[0], d2["y"] - uav_pos[1])
    rt_bearing = math.atan2(d2["y"] - uav_pos[1], d2["x"] - uav_pos[0])
    check(task, "round-trip range/bearing recovered from x/y matches measured_range/measured_bearing",
          close(rt_range, d2["measured_range"], tol=1e-9) and close(rt_bearing, d2["measured_bearing"], tol=1e-9),
          config="derived from previous noisy case",
          expected=f"range={d2['measured_range']:.4f}, bearing={d2['measured_bearing']:.4f}",
          actual=f"range={rt_range:.4f}, bearing={rt_bearing:.4f}", tolerance="1e-9")


# ---------------------------------------------------------------------
# 11. Covariance dimensions
# ---------------------------------------------------------------------
def test_covariance_dimensions():
    task = "covariance_dimensions"
    m = make_bare_model()
    unc = m._measurement_uncertainty(25.0)
    cov = unc["covariance"]
    check(task, "measurement covariance is 3x3 (range, bearing, radial-velocity)",
          len(cov) == 3 and all(len(row) == 3 for row in cov),
          config="range=25.0, defaults", expected="3x3",
          actual=f"{len(cov)}x{len(cov[0]) if cov else 0}", tolerance="exact")
    check(task, "measurement covariance is diagonal (no modeled cross-channel correlation)",
          all(cov[i][j] == 0.0 for i in range(3) for j in range(3) if i != j),
          config="range=25.0, defaults", expected="off-diagonals == 0.0",
          actual=f"{[cov[i][j] for i in range(3) for j in range(3) if i != j]}", tolerance="exact")
    check(task, "diagonal entries equal the individually reported per-channel variances",
          close(cov[0][0], unc["range_variance"]) and close(cov[1][1], unc["bearing_variance"])
          and close(cov[2][2], unc["radial_velocity_variance"]),
          config="range=25.0, defaults",
          expected=f"{[unc['range_variance'], unc['bearing_variance'], unc['radial_velocity_variance']]}",
          actual=f"{[cov[0][0], cov[1][1], cov[2][2]]}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 12. Covariance positivity
# ---------------------------------------------------------------------
def test_covariance_positivity():
    task = "covariance_positivity"
    m = make_bare_model()
    for rng_ in (0.5, 5.0, 25.0, 100.0, 1000.0):
        cov = m._measurement_uncertainty(rng_)["covariance"]
        ok = cov[0][0] >= 0 and cov[1][1] >= 0 and cov[2][2] >= 0
        check(task, f"all variances are non-negative at range={rng_}", ok,
              config=f"range={rng_}, defaults", expected=">= 0",
              actual=f"diag={[cov[0][0], cov[1][1], cov[2][2]]}", tolerance="n/a")

    # Strict positivity: with nonzero configured noise stds, variance
    # should never collapse to exactly 0 for a finite positive range.
    m2 = make_bare_model(range_noise_std=0.3, bearing_noise_std=0.02, radial_velocity_noise_std=0.1)
    for rng_, cond in [(1.0, "clear"), (500.0, "clear"), (25.0, "storm")]:
        m2.environmental_condition = cond
        m2._env_factors = RadarLikeModel.ENV_FACTORS[cond]
        cov = m2._measurement_uncertainty(rng_)["covariance"]
        ok = cov[0][0] > 0 and cov[1][1] > 0 and cov[2][2] > 0
        check(task, f"variances are strictly positive at range={rng_}, condition={cond} (nonzero noise stds configured)",
              ok, config=f"range={rng_}, condition={cond}, nonzero base noise stds",
              expected="> 0", actual=f"diag={[cov[0][0], cov[1][1], cov[2][2]]}", tolerance="n/a")

    # Degraded conditions should raise variance relative to nominal/clear,
    # never lower it.
    m_clear = make_bare_model(environmental_condition="clear", radar_reliability_state="nominal")
    m_bad = make_bare_model(environmental_condition="storm", radar_reliability_state="critical")
    v_clear = m_clear._measurement_uncertainty(25.0)["range_variance"]
    v_bad = m_bad._measurement_uncertainty(25.0)["range_variance"]
    check(task, "range variance under storm+critical-reliability is >= variance under clear+nominal",
          v_bad >= v_clear, config="range=25.0; clear/nominal vs storm/critical",
          expected=f">= {v_clear:.5f}", actual=f"{v_bad:.5f}", tolerance="n/a")


# ---------------------------------------------------------------------
# 13. P_D behavior
# ---------------------------------------------------------------------
def test_pd_behavior():
    task = "pd_behavior"
    m = make_bare_model()
    pd_near = m._measurement_uncertainty(1.0)["pd_effective"]
    pd_far = m._measurement_uncertainty(500.0)["pd_effective"]
    check(task, "P_D decreases as range grows (low SNR -> harder to detect)",
          pd_far < pd_near, config="range=1.0 vs range=500.0",
          expected="pd(500) < pd(1)", actual=f"pd_near={pd_near:.4f} pd_far={pd_far:.4f}", tolerance="n/a")

    m_clear = make_bare_model(environmental_condition="clear")
    m_storm = make_bare_model(environmental_condition="storm")
    pd_clear = m_clear._measurement_uncertainty(25.0)["pd_effective"]
    pd_storm = m_storm._measurement_uncertainty(25.0)["pd_effective"]
    check(task, "P_D is lower in a storm than in clear weather at the same range",
          pd_storm < pd_clear, config="range=25.0; clear vs storm",
          expected="pd(storm) < pd(clear)", actual=f"pd_clear={pd_clear:.4f} pd_storm={pd_storm:.4f}", tolerance="n/a")

    m_nom = make_bare_model(radar_reliability_state="nominal")
    m_crit = make_bare_model(radar_reliability_state="critical")
    pd_nom = m_nom._measurement_uncertainty(25.0)["pd_effective"]
    pd_crit = m_crit._measurement_uncertainty(25.0)["pd_effective"]
    check(task, "P_D is lower for a critical-reliability radar than a nominal one",
          pd_crit < pd_nom, config="range=25.0; nominal vs critical reliability",
          expected="pd(critical) < pd(nominal)", actual=f"pd_nom={pd_nom:.4f} pd_crit={pd_crit:.4f}", tolerance="n/a")

    for rng_ in (0.5, 10.0, 100.0, 1000.0):
        pd = m._measurement_uncertainty(rng_)["pd_effective"]
        check(task, f"P_D stays within [0, 1] at range={rng_}", 0.0 <= pd <= 1.0,
              config=f"range={rng_}, defaults", expected="[0,1]", actual=f"{pd:.4f}", tolerance="n/a")


# ---------------------------------------------------------------------
# 14. P_FA behavior
# ---------------------------------------------------------------------
def test_pfa_behavior():
    task = "pfa_behavior"
    m_low = make_bare_model(clutter_lambda=0.1)
    m_high = make_bare_model(clutter_lambda=5.0)
    pfa_low = m_low._measurement_uncertainty(25.0)["pfa_effective"]
    pfa_high = m_high._measurement_uncertainty(25.0)["pfa_effective"]
    check(task, "P_FA rises with clutter density (clutter_lambda)", pfa_high > pfa_low,
          config="range=25.0; clutter_lambda=0.1 vs 5.0",
          expected="pfa(5.0) > pfa(0.1)", actual=f"pfa_low={pfa_low:.4f} pfa_high={pfa_high:.4f}", tolerance="n/a")

    m_clear = make_bare_model(environmental_condition="clear")
    m_storm = make_bare_model(environmental_condition="storm")
    pfa_clear = m_clear._measurement_uncertainty(25.0)["pfa_effective"]
    pfa_storm = m_storm._measurement_uncertainty(25.0)["pfa_effective"]
    check(task, "P_FA rises in a storm relative to clear weather", pfa_storm > pfa_clear,
          config="range=25.0; clear vs storm",
          expected="pfa(storm) > pfa(clear)", actual=f"pfa_clear={pfa_clear:.4f} pfa_storm={pfa_storm:.4f}", tolerance="n/a")

    for lam in (0.0, 1.0, 10.0, 100.0):
        pfa = make_bare_model(clutter_lambda=lam)._measurement_uncertainty(25.0)["pfa_effective"]
        check(task, f"P_FA stays within [0, 1] at clutter_lambda={lam}", 0.0 <= pfa <= 1.0,
              config=f"range=25.0, clutter_lambda={lam}", expected="[0,1]", actual=f"{pfa:.4f}", tolerance="n/a")


# ---------------------------------------------------------------------
# 15. Poisson clutter
# ---------------------------------------------------------------------
def test_poisson_clutter():
    task = "poisson_clutter"
    m = make_bare_model()
    lam = 4.0
    n_trials = 20000
    samples = [m._poisson_sample(lam) for _ in range(n_trials)]
    sample_mean = statistics.mean(samples)
    sample_var = statistics.pvariance(samples)
    check(task, f"Poisson(lambda={lam}) sample mean matches lambda", abs(sample_mean - lam) < 0.1,
          config=f"lambda={lam}, n={n_trials}", expected=f"~{lam}", actual=f"{sample_mean:.3f}", tolerance="0.1")
    check(task, f"Poisson(lambda={lam}) sample variance matches lambda (mean==variance)",
          abs(sample_var - lam) < 0.3, config=f"lambda={lam}, n={n_trials}",
          expected=f"~{lam}", actual=f"{sample_var:.3f}", tolerance="0.3")

    zero_samples = [m._poisson_sample(0.0) for _ in range(50)]
    check(task, "Poisson(lambda=0) always returns 0", all(s == 0 for s in zero_samples),
          config="lambda=0.0, n=50", expected="{0}", actual=f"{set(zero_samples)}", tolerance="exact")

    m_fixed = make_bare_model(clutter_lambda=3.0)
    m_fixed.clutter_distribution = "fixed"
    m_fixed.false_alarm_probability = 1.0
    counts = [len(m_fixed._generate_clutter((0.0, 0.0))) for _ in range(30)]
    check(task, "clutter_distribution='fixed' always generates round(clutter_lambda) candidates (PFA forced to 1.0)",
          all(c == 3 for c in counts), config="clutter_lambda=3.0, distribution='fixed', PFA=1.0, n=30",
          expected="{3}", actual=f"{sorted(set(counts))}", tolerance="exact")

    m_poisson = make_bare_model(clutter_lambda=3.0)
    m_poisson.clutter_distribution = "poisson"
    m_poisson.false_alarm_probability = 1.0
    poisson_counts = [len(m_poisson._generate_clutter((0.0, 0.0))) for _ in range(2000)]
    check(task, "clutter_distribution='poisson' produces a varying candidate count (not constant)",
          len(set(poisson_counts)) > 1, config="clutter_lambda=3.0, distribution='poisson', PFA=1.0, n=2000",
          expected="> 1 distinct value", actual=f"{len(set(poisson_counts))} distinct values", tolerance="n/a")
    check(task, "clutter_distribution='poisson' mean confirmed count matches clutter_lambda",
          abs(statistics.mean(poisson_counts) - 3.0) < 0.2,
          config="clutter_lambda=3.0, distribution='poisson', PFA=1.0, n=2000",
          expected="~3.0", actual=f"{statistics.mean(poisson_counts):.3f}", tolerance="0.2")


# ---------------------------------------------------------------------
# 16. Range-dependent SNR
# ---------------------------------------------------------------------
def test_range_dependent_snr():
    task = "range_dependent_snr"
    m = make_bare_model(reference_snr_db=30.0, snr_exponent=4.0, reference_range=50.0,
                         environmental_condition="clear")
    snr_at_ref = m._snr_db_for_range(50.0)
    check(task, "SNR equals reference_snr_db at reference_range (clear weather)",
          close(snr_at_ref, 30.0, tol=1e-6), config="range=reference_range=50.0, clear",
          expected="30.0", actual=f"{snr_at_ref}", tolerance="1e-6")

    snr_double = m._snr_db_for_range(100.0)
    expected_drop = 4.0 * 10.0 * math.log10(2.0)
    check(task, "doubling range drops SNR by ~12.04 dB (4th-power falloff)",
          close(snr_at_ref - snr_double, expected_drop, tol=1e-6),
          config="range=100.0 (2x reference), snr_exponent=4.0",
          expected=f"{expected_drop:.4f} dB drop", actual=f"{snr_at_ref - snr_double:.4f} dB drop", tolerance="1e-6")

    snrs = [m._snr_db_for_range(r) for r in (1.0, 10.0, 50.0, 100.0, 500.0)]
    check(task, "SNR decreases monotonically as range increases",
          all(snrs[i] > snrs[i + 1] for i in range(len(snrs) - 1)),
          config="ranges=[1,10,50,100,500]", expected="strictly decreasing",
          actual=f"{[round(s,2) for s in snrs]}", tolerance="n/a")

    m_storm = make_bare_model(reference_snr_db=30.0, snr_exponent=4.0, reference_range=50.0,
                               environmental_condition="storm")
    snr_clear = m._snr_db_for_range(50.0)
    snr_storm = m_storm._snr_db_for_range(50.0)
    check(task, "storm attenuation (6 dB) subtracts directly from SNR at the same range",
          close(snr_clear - snr_storm, 6.0, tol=1e-6),
          config="range=50.0; clear vs storm", expected="6.0 dB drop",
          actual=f"{snr_clear - snr_storm:.4f} dB drop", tolerance="1e-6")

    snr_extreme = m._snr_db_for_range(1e9)
    check(task, "SNR is floored at SNR_DB_MIN for extremely long range",
          close(snr_extreme, RadarLikeModel.SNR_DB_MIN, tol=1e-6),
          config="range=1e9", expected=f"{RadarLikeModel.SNR_DB_MIN}", actual=f"{snr_extreme}", tolerance="1e-6")
    snr_tiny = m._snr_db_for_range(1e-9)
    check(task, "SNR is capped at SNR_DB_MAX for extremely short range",
          close(snr_tiny, RadarLikeModel.SNR_DB_MAX, tol=1e-6),
          config="range=1e-9", expected=f"{RadarLikeModel.SNR_DB_MAX}", actual=f"{snr_tiny}", tolerance="1e-6")

    none_ok = (m._snr_db_for_range(None) is None and m._snr_db_for_range(0.0) is None
               and m._snr_db_for_range(-5.0) is None)
    check(task, "SNR is None for non-positive/unknown range", none_ok,
          config="range in {None, 0.0, -5.0}", expected="None", actual=f"{none_ok}", tolerance="exact")

    qualities = [m._quality_from_snr(s) for s in (-20.0, -5.0, 0.0, 5.0, 20.0, 60.0)]
    check(task, "measurement quality rises monotonically with SNR and stays in [0, 1]",
          all(0.0 <= q <= 1.0 for q in qualities)
          and all(qualities[i] <= qualities[i + 1] for i in range(len(qualities) - 1)),
          config="SNR=[-20,-5,0,5,20,60] dB", expected="monotonic, in [0,1]",
          actual=f"{[round(q,4) for q in qualities]}", tolerance="n/a")
    check(task, "quality at SNR=0 dB is exactly 0.5 (SNR/(SNR+1) with linear SNR=1)",
          close(m._quality_from_snr(0.0), 0.5, tol=1e-6),
          config="SNR=0 dB", expected="0.5", actual=f"{m._quality_from_snr(0.0)}", tolerance="1e-6")


# ---------------------------------------------------------------------
# 17. Latency
# ---------------------------------------------------------------------
def test_latency():
    task = "latency"

    # Unit-level: _get_delayed_scan's own gen_t/cutoff bookkeeping.
    m = make_bare_model(radar_latency_steps=3)
    m._scan_buffer = {0: [(0, ["scanA"], False, []), (2, ["scanB"], False, [])]}
    result_t1 = m._get_delayed_scan(0, 1)
    check(task, "nothing has arrived yet at t=1 given latency=3 and earliest scan gen_t=0 (needs t>=3)",
          result_t1 is None, config="radar_latency_steps=3, scan buffer gen_t=[0,2], query t=1",
          expected="None", actual=f"{result_t1}", tolerance="exact")
    result_t3 = m._get_delayed_scan(0, 3)
    check(task, "scan generated at t=0 has arrived by t=3 (3-step latency elapsed)",
          result_t3 is not None and result_t3[0] == ["scanA"],
          config="radar_latency_steps=3, scan buffer gen_t=[0,2], query t=3",
          expected="scanA", actual=f"{result_t3[0] if result_t3 else None}", tolerance="exact")
    result_t5 = m._get_delayed_scan(0, 5)
    check(task, "by t=5, the most recent arrived scan is gen_t=2's (t=5-3=2 cutoff)",
          result_t5 is not None and result_t5[0] == ["scanB"],
          config="radar_latency_steps=3, scan buffer gen_t=[0,2], query t=5",
          expected="scanB", actual=f"{result_t5[0] if result_t5 else None}", tolerance="exact")

    # Zero latency: a scan generated this step should be immediately usable.
    m0 = make_bare_model(radar_latency_steps=0)
    m0._scan_buffer = {0: [(4, ["scanC"], False, [])]}
    result_zero = m0._get_delayed_scan(0, 4)
    check(task, "with radar_latency_steps=0, a scan is available the same step it was generated",
          result_zero is not None and result_zero[0] == ["scanC"],
          config="radar_latency_steps=0, scan gen_t=4, query t=4",
          expected="scanC", actual=f"{result_zero[0] if result_zero else None}", tolerance="exact")

    # Integration-level: full RadarLikeModel + _FakeSim, real closure.
    scn = dict(CLEAN_SCN, radar_max_range=15.0, radar_latency_steps=3)
    model, fs = make_full_model(scn)
    seen_ids_by_t = {}
    for t in range(6):
        fs.true_dets_by_uav[0] = [{"id": "obstacle_0", "x": 5.0, "y": 0.0, "distance": 5.0}]
        model.sim.step(t)
        seen_ids_by_t[t] = [d["id"] for d in model._capture[0]["perceived"]]
    check(task, "with radar_latency_steps=3, a static in-range target is NOT perceived at t=0,1,2",
          all(seen_ids_by_t[t] == [] for t in (0, 1, 2)),
          config="radar_latency_steps=3, static target at range=5.0 within radar_max_range=15.0",
          expected="[] at t=0,1,2", actual=f"{[seen_ids_by_t[t] for t in (0,1,2)]}", tolerance="exact")
    check(task, "...and IS perceived starting at t=3 (once the 3-step delay has elapsed)",
          all(seen_ids_by_t[t] == ["obstacle_0"] for t in (3, 4, 5)),
          config="radar_latency_steps=3, static target at range=5.0",
          expected="['obstacle_0'] at t=3,4,5", actual=f"{[seen_ids_by_t[t] for t in (3,4,5)]}", tolerance="exact")


# ---------------------------------------------------------------------
# 18. Dropout
# ---------------------------------------------------------------------
def test_dropout():
    task = "dropout"

    # Unit-level: _radar_dropout_fires empirical rate.
    for p in (0.0, 0.3, 1.0):
        m = make_bare_model(radar_dropout_probability=p)
        n = 5000
        fires = sum(1 for _ in range(n) if m._radar_dropout_fires())
        rate = fires / n
        check(task, f"empirical radar dropout rate matches configured radar_dropout_probability={p}",
              abs(rate - p) < 0.03, config=f"radar_dropout_probability={p}, n={n}",
              expected=f"~{p}", actual=f"{rate:.4f}", tolerance="0.03")

    # Integration-level: full RadarLikeModel + _FakeSim, real closure.
    scn_always = dict(CLEAN_SCN, radar_max_range=15.0, radar_dropout_probability=1.0)
    model, fs = make_full_model(scn_always)
    fs.true_dets_by_uav[0] = [{"id": "obstacle_0", "x": 5.0, "y": 0.0, "distance": 5.0}]
    model.sim.step(0)
    cap = model._capture[0]
    check(task, "radar_dropout_probability=1.0 -> this scan is a total blackout (perceived=[], dropout=True)",
          cap["perceived"] == [] and cap["dropout"] is True,
          config="radar_dropout_probability=1.0, target in range",
          expected="perceived=[], dropout=True",
          actual=f"perceived={cap['perceived']}, dropout={cap['dropout']}", tolerance="exact")

    scn_never = dict(CLEAN_SCN, radar_max_range=15.0, radar_dropout_probability=0.0)
    model2, fs2 = make_full_model(scn_never)
    fs2.true_dets_by_uav[0] = [{"id": "obstacle_0", "x": 5.0, "y": 0.0, "distance": 5.0}]
    model2.sim.step(0)
    cap2 = model2._capture[0]
    check(task, "radar_dropout_probability=0.0 -> no blackout, in-range target is perceived",
          [d["id"] for d in cap2["perceived"]] == ["obstacle_0"] and cap2["dropout"] is False,
          config="radar_dropout_probability=0.0, target in range",
          expected="perceived=['obstacle_0'], dropout=False",
          actual=f"perceived={[d['id'] for d in cap2['perceived']]}, dropout={cap2['dropout']}", tolerance="exact")


# ---------------------------------------------------------------------
# 19. Timestamp behavior
# ---------------------------------------------------------------------
def test_timestamp_behavior():
    task = "timestamp_behavior"
    m = make_bare_model(radar_latency_steps=0, radial_velocity_noise_std=0.0,
                         range_noise_std=0.0, bearing_noise_std=0.0)

    # _make_row's time_step field must echo exactly the step index passed in.
    true_det = {"id": "obstacle_0", "x": 5.0, "y": 0.0}
    measured_det = {"id": "obstacle_0", "x": 5.0, "y": 0.0, "confidence": 0.9}
    for t in (0, 1, 7, 42):
        row = m._make_row(t, 0, true_det, measured_det, (0.0, 0.0), (0.0, 0.0), {}, status="detected")
        check(task, f"row's time_step field equals the step index passed in (t={t})",
              row["time_step"] == t, config=f"t={t}",
              expected=f"{t}", actual=f"{row['time_step']}", tolerance="exact")

    # Scan-buffer bookkeeping: only entries whose generation timestamp
    # gen_t satisfies gen_t <= query_t - radar_latency_steps are eligible,
    # and consumed/stale entries are pruned rather than re-served forever.
    m2 = make_bare_model(radar_latency_steps=2)
    m2._scan_buffer = {0: [(0, ["s0"], False, []), (1, ["s1"], False, []), (2, ["s2"], False, [])]}
    r = m2._get_delayed_scan(0, 2)  # cutoff=0 -> only gen_t=0 qualifies
    check(task, "at query t=2 with latency=2, only the gen_t=0 scan has arrived (cutoff=t-latency=0)",
          r is not None and r[0] == ["s0"],
          config="radar_latency_steps=2, buffer gen_t=[0,1,2], query t=2",
          expected="s0", actual=f"{r[0] if r else None}", tolerance="exact")
    check(task, "consumed/older buffer entries are pruned after being served (buffer shrinks)",
          m2._scan_buffer[0] == [(1, ["s1"], False, []), (2, ["s2"], False, [])],
          config="after _get_delayed_scan(0, 2) call",
          expected="[(1,...),(2,...)] remaining", actual=f"{m2._scan_buffer[0]}", tolerance="exact")

    # Querying a timestamp before anything could possibly have arrived
    # returns None rather than an empty/garbage scan.
    m3 = make_bare_model(radar_latency_steps=5)
    m3._scan_buffer = {0: []}
    r3 = m3._get_delayed_scan(0, 0)
    check(task, "querying with an empty scan buffer returns None (nothing has arrived)",
          r3 is None, config="radar_latency_steps=5, empty buffer, query t=0",
          expected="None", actual=f"{r3}", tolerance="exact")


# ---------------------------------------------------------------------
# 20. Maximum sensing range
# ---------------------------------------------------------------------
def test_maximum_sensing_range():
    task = "maximum_sensing_range"

    # Config resolution: radar_max_range falls back to sim.sensor_range
    # when not explicitly set in the scenario/radar config.
    scn_default = dict(CLEAN_SCN)
    model_default, fs_default = make_full_model(scn_default, sensor_range=12.5)
    check(task, "radar_max_range defaults to sim.sensor_range when not overridden",
          close(model_default.radar_max_range, 12.5),
          config="no radar_max_range override, sim.sensor_range=12.5",
          expected="12.5", actual=f"{model_default.radar_max_range}", tolerance="1e-6")

    # Integration: targets beyond radar_max_range are gated out (radar_pd_miss),
    # targets within range are perceived, and a target exactly at the boundary
    # (range == radar_max_range) is NOT gated (condition is strictly '>').
    scn = dict(CLEAN_SCN, radar_max_range=15.0, radar_min_range=0.0)
    model, fs = make_full_model(scn, sensor_range=15.0)
    near = {"id": "obstacle_0", "x": 5.0, "y": 0.0, "distance": 5.0}
    far = {"id": "uav_1", "x": 25.0, "y": 0.0, "distance": 25.0}
    boundary = {"id": "uav_2", "x": 0.0, "y": 15.0, "distance": 15.0}
    fs.true_dets_by_uav[0] = [near, far, boundary]
    model.sim.step(0)
    cap = model._capture[0]
    perceived_ids = {d["id"] for d in cap["perceived"]}
    check(task, "target within radar_max_range is perceived (range=5.0 < max=15.0)",
          "obstacle_0" in perceived_ids, config="radar_max_range=15.0, target range=5.0",
          expected="perceived", actual=f"perceived_ids={perceived_ids}", tolerance="n/a")
    check(task, "target beyond radar_max_range is gated out with radar_pd_miss_flag equivalent (pd_missed_ids)",
          "uav_1" in cap["pd_missed_ids"] and "uav_1" not in perceived_ids,
          config="radar_max_range=15.0, target range=25.0",
          expected="in pd_missed_ids, not perceived",
          actual=f"pd_missed_ids={cap['pd_missed_ids']}, perceived_ids={perceived_ids}", tolerance="n/a")
    check(task, "target exactly at radar_max_range boundary (range==max) is NOT gated (condition is strict '>')",
          "uav_2" in perceived_ids, config="radar_max_range=15.0, target range=15.0 (exact boundary)",
          expected="perceived", actual=f"perceived_ids={perceived_ids}", tolerance="n/a")

    # radar_min_range: a near-field blind zone gates out targets that are
    # too close, independent of the max-range gate.
    scn_min = dict(CLEAN_SCN, radar_max_range=15.0, radar_min_range=2.0)
    model_min, fs_min = make_full_model(scn_min, sensor_range=15.0)
    too_close = {"id": "obstacle_0", "x": 1.0, "y": 0.0, "distance": 1.0}
    ok_range = {"id": "uav_1", "x": 5.0, "y": 0.0, "distance": 5.0}
    fs_min.true_dets_by_uav[0] = [too_close, ok_range]
    model_min.sim.step(0)
    cap_min = model_min._capture[0]
    perceived_min_ids = {d["id"] for d in cap_min["perceived"]}
    check(task, "target inside the radar_min_range blind zone is gated out",
          "obstacle_0" in cap_min["pd_missed_ids"] and "obstacle_0" not in perceived_min_ids,
          config="radar_min_range=2.0, target range=1.0",
          expected="in pd_missed_ids, not perceived",
          actual=f"pd_missed_ids={cap_min['pd_missed_ids']}, perceived_ids={perceived_min_ids}", tolerance="n/a")
    check(task, "target beyond the blind zone but within max range is perceived normally",
          "uav_1" in perceived_min_ids, config="radar_min_range=2.0, radar_max_range=15.0, target range=5.0",
          expected="perceived", actual=f"perceived_ids={perceived_min_ids}", tolerance="n/a")


def main():
    test_exact_range()
    test_exact_bearing()
    test_exact_radial_velocity()
    test_positive_radial_velocity()
    test_negative_radial_velocity()
    test_stationary_target()
    test_noisy_range_and_bearing()
    test_noisy_radial_velocity()
    test_cartesian_reconstruction()
    test_covariance_dimensions()
    test_covariance_positivity()
    test_pd_behavior()
    test_pfa_behavior()
    test_poisson_clutter()
    test_range_dependent_snr()
    test_latency()
    test_dropout()
    test_timestamp_behavior()
    test_maximum_sensing_range()

    failed = _checker.print_summary()

    out_dir = os.path.join(_ROOT_DIR, "results")
    out_path = os.path.join(out_dir, "radar_model_validation_results.md")
    _checker.write_markdown(
        out_path, "Radar Model Validation Results",
        intro=(
            "Controlled, hand-computable checks of the core radar-domain "
            "equations and behaviors in `radar_like_model.py` (range/"
            "bearing/radial-velocity, noise, Cartesian reconstruction, "
            "covariance, P_D/P_FA, Poisson clutter, range-dependent SNR, "
            "latency, dropout, scan-timestamp bookkeeping, and sensing-"
            "range limits) - not the full swarm simulation, just the "
            "radar-domain math and the radar-level scan machinery.\n\n"
            "Two strategies are used: (1) a *bare* `RadarLikeModel` "
            "(constructed via `object.__new__`, bypassing `__init__`, "
            "which needs a full `Simulation`) for the pure-math checks; "
            "and (2) a *full* `RadarLikeModel` wired to a minimal fake "
            "`Simulation` stand-in (`_FakeSim`) for checks that need the "
            "real `_patch_perception` closure (max/min-range gating, "
            "radar dropout, and the scan-generation-timestamp/latency "
            "buffer) exercised against controlled UAV/target geometry.\n\n"
            "`dependability.perception_quality_monitor` (imported "
            "unconditionally by `simple_swarm_sim.py`) was not part of "
            "this task's inputs; a minimal stub is used purely to satisfy "
            "the import chain and is not exercised by any check here."
        ))
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
