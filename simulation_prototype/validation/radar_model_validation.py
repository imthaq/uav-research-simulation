"""
radar_model_validation.py

Task 3: validates the core radar equations in radar_like_model.py against
controlled, hand-computable cases - not against the full swarm simulation
(which radar_like_model wraps), just the radar-domain math itself:

  - range calculation
  - bearing calculation
  - radial-velocity calculation
  - noisy range conversion
  - noisy bearing conversion
  - detected x/y reconstruction
  - covariance dimensions
  - P_D (probability of detection) behavior
  - P_FA (probability of false alarm) behavior
  - Poisson clutter behavior
  - range-dependent SNR

Each check is a controlled case with a known expected answer (or a known
statistical property), asserted with a small numerical tolerance. Results
are printed and written to results/radar_model_validation_results.md.

Usage:
    python radar_model_validation.py
"""

import math
import os
import statistics
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_MODELS_DIR = os.path.join(_ROOT_DIR, "models")
for _p in (_ROOT_DIR, _MODELS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from models.radar_like_model import (
    RadarLikeModel,
    _range_bearing_radial,
    _wrap_angle,
)
from validation_common import Checker

_checker = Checker()


def check(task, description, condition, detail=""):
    return _checker.check(task, description, condition, detail)


def close(a, b, tol=1e-6):
    return _checker.close(a, b, tol)


def make_bare_model(**overrides):
    """Builds a RadarLikeModel instance without running __init__ (which
    would require a real Simulation), setting only the attributes the
    methods under test actually read."""
    m = object.__new__(RadarLikeModel)
    defaults = dict(
        range_noise_std=RadarLikeModel.DEFAULT_RANGE_NOISE_STD,
        bearing_noise_std=RadarLikeModel.DEFAULT_BEARING_NOISE_STD,
        radial_velocity_noise_std=RadarLikeModel.DEFAULT_RADIAL_VELOCITY_NOISE_STD,
        detection_probability=RadarLikeModel.DEFAULT_RADAR_DETECTION_PROBABILITY,
        false_alarm_probability=RadarLikeModel.DEFAULT_RADAR_FALSE_ALARM_PROBABILITY,
        clutter_lambda=RadarLikeModel.DEFAULT_RADAR_CLUTTER_DENSITY,
        reference_snr_db=RadarLikeModel.DEFAULT_REFERENCE_SNR_DB,
        snr_exponent=RadarLikeModel.DEFAULT_SNR_EXPONENT,
        reference_range=50.0,
        environmental_condition="clear",
        radar_reliability_state="nominal",
        radar_mode="normal",
        radar_clutter_confidence_bias=0.0,
    )
    defaults.update(overrides)
    for k, v in defaults.items():
        setattr(m, k, v)
    m._env_factors = RadarLikeModel.ENV_FACTORS[m.environmental_condition]
    m._reliability_factors = RadarLikeModel.RELIABILITY_FACTORS[m.radar_reliability_state]
    m._mode_factors = RadarLikeModel.RADAR_MODES[m.radar_mode]
    import random
    m.radar_rng = random.Random(12345)
    m._clutter_counter = 0
    return m


# ---------------------------------------------------------------------
# 1. Range calculation
# ---------------------------------------------------------------------
def test_range_calculation():
    # 3-4-5 triangle: dx=3, dy=4 -> range=5
    rng, _, _ = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (3.0, 4.0), None)
    check("range_calculation", "3-4-5 triangle gives range=5.0", close(rng, 5.0), f"got {rng}")

    # Observer offset from origin, same relative displacement
    rng2, _, _ = _range_bearing_radial((10.0, 10.0), (0.0, 0.0), (13.0, 14.0), None)
    check("range_calculation", "range is translation-invariant (offset observer)",
          close(rng2, 5.0), f"got {rng2}")

    # Coincident observer/target -> range 0
    rng3, _, _ = _range_bearing_radial((5.0, 5.0), (0.0, 0.0), (5.0, 5.0), None)
    check("range_calculation", "coincident observer/target gives range=0.0",
          close(rng3, 0.0), f"got {rng3}")


# ---------------------------------------------------------------------
# 2. Bearing calculation
# ---------------------------------------------------------------------
def test_bearing_calculation():
    cases = [
        ((1.0, 0.0), 0.0, "due east -> 0 rad"),
        ((0.0, 1.0), math.pi / 2, "due north -> +pi/2 rad"),
        ((-1.0, 0.0), math.pi, "due west -> +/-pi rad"),
        ((0.0, -1.0), -math.pi / 2, "due south -> -pi/2 rad"),
    ]
    for (dx, dy), expected, label in cases:
        _, bearing, _ = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (dx, dy), None)
        ok = close(bearing, expected) or close(abs(bearing), abs(expected))
        check("bearing_calculation", label, ok, f"got {bearing:.4f} rad")

    # Angle wrapping helper: 3*pi should wrap into (-pi, pi] as pi (or -pi)
    wrapped = _wrap_angle(3 * math.pi)
    check("bearing_calculation", "_wrap_angle folds 3*pi into (-pi, pi]",
          close(abs(wrapped), math.pi, tol=1e-6), f"got {wrapped:.4f}")


# ---------------------------------------------------------------------
# 3. Radial-velocity calculation
# ---------------------------------------------------------------------
def test_radial_velocity_calculation():
    # Target directly east of observer, moving further east (away):
    # radial velocity should equal its full east-ward speed.
    _, _, rv = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (3.0, 0.0))
    check("radial_velocity_calculation", "target moving directly away gives radial_vel=+speed",
          close(rv, 3.0), f"got {rv}")

    # Same target moving toward the observer (west): negative full speed.
    _, _, rv2 = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (-3.0, 0.0))
    check("radial_velocity_calculation", "target moving directly toward observer gives radial_vel=-speed",
          close(rv2, -3.0), f"got {rv2}")

    # Target moving perpendicular to the line of sight contributes 0 range-rate.
    _, _, rv3 = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (0.0, 5.0))
    check("radial_velocity_calculation", "purely tangential motion gives radial_vel=0",
          close(rv3, 0.0), f"got {rv3}")

    # Observer's own motion is subtracted (relative velocity, not absolute).
    _, _, rv4 = _range_bearing_radial((0.0, 0.0), (3.0, 0.0), (10.0, 0.0), (3.0, 0.0))
    check("radial_velocity_calculation", "matching observer/target velocity gives radial_vel=0 (relative motion)",
          close(rv4, 0.0), f"got {rv4}")

    # target_vel=None (e.g. static clutter) -> radial velocity undefined
    _, _, rv5 = _range_bearing_radial((0.0, 0.0), (0.0, 0.0), (10.0, 0.0), None)
    check("radial_velocity_calculation", "target_vel=None gives radial_vel=None",
          rv5 is None, f"got {rv5}")


# ---------------------------------------------------------------------
# 4/5. Noisy range and bearing conversion
# ---------------------------------------------------------------------
def test_noisy_range_bearing_conversion():
    # Zero noise std: measured range/bearing must equal the true values.
    m = make_bare_model(range_noise_std=0.0, bearing_noise_std=0.0,
                         radial_velocity_noise_std=0.0)
    uav_pos = (0.0, 0.0)
    true_by_id = {"t1": {"x": 6.0, "y": 8.0}}
    d = {"id": "t1", "x": 6.0, "y": 8.0}
    m._apply_radar_noise(d, uav_pos, true_by_id)
    check("noisy_range_conversion", "zero-noise range equals true range (6-8-10 triangle)",
          close(d["measured_range"], 10.0, tol=1e-6), f"got {d['measured_range']}")
    expected_bearing = math.atan2(8.0, 6.0)
    check("noisy_bearing_conversion", "zero-noise bearing equals true bearing",
          close(d["measured_bearing"], expected_bearing, tol=1e-6), f"got {d['measured_bearing']}")

    # Nonzero noise std: over many trials, the sample std of the measured
    # range/bearing should land close to the configured 1-sigma value.
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
    check("noisy_range_conversion", "sampled range noise std matches configured range_noise_std (~2.0)",
          abs(range_std - 2.0) < 0.15, f"sample std={range_std:.3f}")
    check("noisy_bearing_conversion", "sampled bearing noise std matches configured bearing_noise_std (~0.0873 rad)",
          abs(bearing_std - math.radians(5.0)) < 0.01, f"sample std={bearing_std:.4f} rad")
    check("noisy_range_conversion", "noisy range is floored at 0.05 (never goes non-positive)",
          all(r >= 0.05 for r in range_samples), f"min={min(range_samples):.4f}")


# ---------------------------------------------------------------------
# 6. Detected x/y reconstruction
# ---------------------------------------------------------------------
def test_detected_xy_reconstruction():
    m = make_bare_model(range_noise_std=0.0, bearing_noise_std=0.0,
                         radial_velocity_noise_std=0.0)
    uav_pos = (10.0, -5.0)
    true_by_id = {"t1": {"x": 13.0, "y": -1.0}}  # dx=3, dy=4 -> range 5
    d = {"id": "t1", "x": 13.0, "y": -1.0}
    m._apply_radar_noise(d, uav_pos, true_by_id)
    check("detected_xy_reconstruction", "zero-noise reconstructed x/y matches true target position",
          close(d["x"], 13.0, tol=1e-6) and close(d["y"], -1.0, tol=1e-6),
          f"got ({d['x']:.4f}, {d['y']:.4f})")

    # With noise: reconstructed x/y must equal uav_pos + range*(cos,sin(bearing))
    # for the SAME measured_range/measured_bearing that got logged (i.e. the
    # reconstruction is self-consistent, not derived from a different pair).
    m2 = make_bare_model(range_noise_std=1.5, bearing_noise_std=math.radians(3.0),
                          radial_velocity_noise_std=0.0)
    d2 = {"id": "t1", "x": 13.0, "y": -1.0}
    m2._apply_radar_noise(d2, uav_pos, true_by_id)
    expected_x = uav_pos[0] + d2["measured_range"] * math.cos(d2["measured_bearing"])
    expected_y = uav_pos[1] + d2["measured_range"] * math.sin(d2["measured_bearing"])
    check("detected_xy_reconstruction", "noisy x/y = uav_pos + measured_range*(cos,sin(measured_bearing))",
          close(d2["x"], expected_x, tol=1e-9) and close(d2["y"], expected_y, tol=1e-9),
          f"got ({d2['x']:.4f},{d2['y']:.4f}) vs expected ({expected_x:.4f},{expected_y:.4f})")


# ---------------------------------------------------------------------
# 7. Covariance dimensions
# ---------------------------------------------------------------------
def test_covariance_dimensions():
    m = make_bare_model()
    unc = m._measurement_uncertainty(25.0)
    cov = unc["covariance"]
    check("covariance_dimensions", "measurement covariance is 3x3 (range, bearing, radial-velocity)",
          len(cov) == 3 and all(len(row) == 3 for row in cov), f"shape={len(cov)}x{len(cov[0]) if cov else 0}")
    check("covariance_dimensions", "measurement covariance is diagonal (no modeled cross-channel correlation)",
          all(cov[i][j] == 0.0 for i in range(3) for j in range(3) if i != j),
          f"off-diagonals={[cov[i][j] for i in range(3) for j in range(3) if i != j]}")
    check("covariance_dimensions", "diagonal entries equal the reported per-channel variances",
          close(cov[0][0], unc["range_variance"]) and close(cov[1][1], unc["bearing_variance"])
          and close(cov[2][2], unc["radial_velocity_variance"]),
          f"diag={[cov[0][0], cov[1][1], cov[2][2]]}")
    check("covariance_dimensions", "all variances are non-negative",
          cov[0][0] >= 0 and cov[1][1] >= 0 and cov[2][2] >= 0)


# ---------------------------------------------------------------------
# 8. P_D behavior
# ---------------------------------------------------------------------
def test_pd_behavior():
    m = make_bare_model()
    pd_near = m._measurement_uncertainty(1.0)["pd_effective"]
    pd_far = m._measurement_uncertainty(500.0)["pd_effective"]
    check("pd_behavior", "P_D decreases as range grows (low SNR -> harder to detect)",
          pd_far < pd_near, f"pd_near={pd_near:.4f} pd_far={pd_far:.4f}")

    m_clear = make_bare_model(environmental_condition="clear")
    m_storm = make_bare_model(environmental_condition="storm")
    pd_clear = m_clear._measurement_uncertainty(25.0)["pd_effective"]
    pd_storm = m_storm._measurement_uncertainty(25.0)["pd_effective"]
    check("pd_behavior", "P_D is lower in a storm than in clear weather at the same range",
          pd_storm < pd_clear, f"pd_clear={pd_clear:.4f} pd_storm={pd_storm:.4f}")

    m_nom = make_bare_model(radar_reliability_state="nominal")
    m_crit = make_bare_model(radar_reliability_state="critical")
    pd_nom = m_nom._measurement_uncertainty(25.0)["pd_effective"]
    pd_crit = m_crit._measurement_uncertainty(25.0)["pd_effective"]
    check("pd_behavior", "P_D is lower for a critical-reliability radar than a nominal one",
          pd_crit < pd_nom, f"pd_nominal={pd_nom:.4f} pd_critical={pd_crit:.4f}")

    for rng_ in (0.5, 10.0, 100.0, 1000.0):
        pd = m._measurement_uncertainty(rng_)["pd_effective"]
        check("pd_behavior", f"P_D stays within [0, 1] at range={rng_}", 0.0 <= pd <= 1.0, f"pd={pd:.4f}")


# ---------------------------------------------------------------------
# 9. P_FA behavior
# ---------------------------------------------------------------------
def test_pfa_behavior():
    m_low = make_bare_model(clutter_lambda=0.1)
    m_high = make_bare_model(clutter_lambda=5.0)
    pfa_low = m_low._measurement_uncertainty(25.0)["pfa_effective"]
    pfa_high = m_high._measurement_uncertainty(25.0)["pfa_effective"]
    check("pfa_behavior", "P_FA rises with clutter density (clutter_lambda)",
          pfa_high > pfa_low, f"pfa_low={pfa_low:.4f} pfa_high={pfa_high:.4f}")

    m_clear = make_bare_model(environmental_condition="clear")
    m_storm = make_bare_model(environmental_condition="storm")
    pfa_clear = m_clear._measurement_uncertainty(25.0)["pfa_effective"]
    pfa_storm = m_storm._measurement_uncertainty(25.0)["pfa_effective"]
    check("pfa_behavior", "P_FA rises in a storm relative to clear weather",
          pfa_storm > pfa_clear, f"pfa_clear={pfa_clear:.4f} pfa_storm={pfa_storm:.4f}")

    for lam in (0.0, 1.0, 10.0, 100.0):
        pfa = make_bare_model(clutter_lambda=lam)._measurement_uncertainty(25.0)["pfa_effective"]
        check("pfa_behavior", f"P_FA stays within [0, 1] at clutter_lambda={lam}", 0.0 <= pfa <= 1.0, f"pfa={pfa:.4f}")


# ---------------------------------------------------------------------
# 10. Poisson clutter behavior
# ---------------------------------------------------------------------
def test_poisson_clutter_behavior():
    m = make_bare_model()
    lam = 4.0
    n_trials = 20000
    samples = [m._poisson_sample(lam) for _ in range(n_trials)]
    sample_mean = statistics.mean(samples)
    sample_var = statistics.pvariance(samples)
    check("poisson_clutter_behavior", f"Poisson(lambda={lam}) sample mean matches lambda",
          abs(sample_mean - lam) < 0.1, f"sample_mean={sample_mean:.3f}")
    check("poisson_clutter_behavior", f"Poisson(lambda={lam}) sample variance matches lambda (mean==variance)",
          abs(sample_var - lam) < 0.3, f"sample_var={sample_var:.3f}")

    zero_samples = [m._poisson_sample(0.0) for _ in range(50)]
    check("poisson_clutter_behavior", "Poisson(lambda=0) always returns 0",
          all(s == 0 for s in zero_samples), f"got {set(zero_samples)}")

    # _generate_clutter: candidate count should follow clutter_lambda via
    # Poisson when clutter_distribution="poisson", and be exactly
    # round(clutter_lambda) every call when "fixed".
    m_fixed = make_bare_model(clutter_lambda=3.0)
    m_fixed.clutter_distribution = "fixed"
    m_fixed.clutter_range_min = 0.0
    m_fixed.clutter_range_max = 50.0
    m_fixed.false_alarm_probability = 1.0  # force every candidate to be confirmed
    counts = []
    for _ in range(30):
        m_fixed.false_alarm_probability = 1.0
        dets = m_fixed._generate_clutter((0.0, 0.0))
        # PFA is itself range/condition-scaled and clamped to 1.0 at most,
        # so with false_alarm_probability=1.0 and clear/nominal factors the
        # effective PFA is exactly 1.0 -> every candidate is confirmed.
        counts.append(len(dets))
    check("poisson_clutter_behavior", "clutter_distribution='fixed' always generates round(clutter_lambda) candidates (PFA forced to 1.0)",
          all(c == 3 for c in counts), f"counts={sorted(set(counts))}")

    m_poisson = make_bare_model(clutter_lambda=3.0)
    m_poisson.clutter_distribution = "poisson"
    m_poisson.clutter_range_min = 0.0
    m_poisson.clutter_range_max = 50.0
    m_poisson.false_alarm_probability = 1.0
    poisson_counts = [len(m_poisson._generate_clutter((0.0, 0.0))) for _ in range(2000)]
    check("poisson_clutter_behavior", "clutter_distribution='poisson' produces a varying candidate count (not constant)",
          len(set(poisson_counts)) > 1, f"distinct counts seen={len(set(poisson_counts))}")
    check("poisson_clutter_behavior", "clutter_distribution='poisson' mean confirmed count matches clutter_lambda",
          abs(statistics.mean(poisson_counts) - 3.0) < 0.2, f"mean={statistics.mean(poisson_counts):.3f}")


# ---------------------------------------------------------------------
# 11. Range-dependent SNR
# ---------------------------------------------------------------------
def test_range_dependent_snr():
    m = make_bare_model(reference_snr_db=30.0, snr_exponent=4.0, reference_range=50.0,
                         environmental_condition="clear")
    # At range == reference_range, SNR should equal reference_snr_db exactly
    # (clear weather has 0 dB attenuation).
    snr_at_ref = m._snr_db_for_range(50.0)
    check("range_dependent_snr", "SNR equals reference_snr_db at reference_range (clear weather)",
          close(snr_at_ref, 30.0, tol=1e-6), f"got {snr_at_ref}")

    # Doubling range with a 4th-power exponent should drop SNR by
    # 4 * 10*log10(2) ~= 12.04 dB.
    snr_double = m._snr_db_for_range(100.0)
    expected_drop = 4.0 * 10.0 * math.log10(2.0)
    check("range_dependent_snr", "doubling range drops SNR by ~12.04 dB (4th-power falloff)",
          close(snr_at_ref - snr_double, expected_drop, tol=1e-6),
          f"drop={snr_at_ref - snr_double:.4f} dB, expected {expected_drop:.4f} dB")

    # Monotonic decrease with range.
    snrs = [m._snr_db_for_range(r) for r in (1.0, 10.0, 50.0, 100.0, 500.0)]
    check("range_dependent_snr", "SNR decreases monotonically as range increases",
          all(snrs[i] > snrs[i + 1] for i in range(len(snrs) - 1)), f"snrs={[round(s,2) for s in snrs]}")

    # Environmental attenuation subtracts directly from SNR.
    m_storm = make_bare_model(reference_snr_db=30.0, snr_exponent=4.0, reference_range=50.0,
                               environmental_condition="storm")
    snr_clear = m._snr_db_for_range(50.0)
    snr_storm = m_storm._snr_db_for_range(50.0)
    check("range_dependent_snr", "storm attenuation (6 dB) subtracts directly from SNR at the same range",
          close(snr_clear - snr_storm, 6.0, tol=1e-6), f"drop={snr_clear - snr_storm:.4f} dB")

    # SNR is clamped to [SNR_DB_MIN, SNR_DB_MAX] even at extreme range.
    snr_extreme = m._snr_db_for_range(1e9)
    check("range_dependent_snr", "SNR is floored at SNR_DB_MIN for extremely long range",
          close(snr_extreme, RadarLikeModel.SNR_DB_MIN, tol=1e-6), f"got {snr_extreme}")
    snr_tiny = m._snr_db_for_range(1e-9)
    check("range_dependent_snr", "SNR is capped at SNR_DB_MAX for extremely short range",
          close(snr_tiny, RadarLikeModel.SNR_DB_MAX, tol=1e-6), f"got {snr_tiny}")

    # None/non-positive range is undefined.
    check("range_dependent_snr", "SNR is None for non-positive/unknown range",
          m._snr_db_for_range(None) is None and m._snr_db_for_range(0.0) is None
          and m._snr_db_for_range(-5.0) is None)

    # quality factor: rises monotonically with SNR, bounded in [0, 1].
    qualities = [m._quality_from_snr(s) for s in (-20.0, -5.0, 0.0, 5.0, 20.0, 60.0)]
    check("range_dependent_snr", "measurement quality rises monotonically with SNR and stays in [0, 1]",
          all(0.0 <= q <= 1.0 for q in qualities)
          and all(qualities[i] <= qualities[i + 1] for i in range(len(qualities) - 1)),
          f"qualities={[round(q,4) for q in qualities]}")
    check("range_dependent_snr", "quality at SNR=0 dB is exactly 0.5 (SNR/(SNR+1) with linear SNR=1)",
          close(m._quality_from_snr(0.0), 0.5, tol=1e-6), f"got {m._quality_from_snr(0.0)}")


def main():
    test_range_calculation()
    test_bearing_calculation()
    test_radial_velocity_calculation()
    test_noisy_range_bearing_conversion()
    test_detected_xy_reconstruction()
    test_covariance_dimensions()
    test_pd_behavior()
    test_pfa_behavior()
    test_poisson_clutter_behavior()
    test_range_dependent_snr()

    failed = _checker.print_summary()

    out_dir = os.path.join(_ROOT_DIR, "results")
    out_path = os.path.join(out_dir, "radar_model_validation_results.md")
    _checker.write_markdown(
        out_path, "Radar Model Validation Results (Task 3)",
        intro="Controlled, hand-computable checks of the core radar equations "
              "in `models/radar_like_model.py` (range/bearing/radial-velocity "
              "conversion, noise, P_D/P_FA, clutter, range-dependent SNR) - "
              "not the full swarm simulation, just the radar-domain math.")
    print(f"Detailed results written to: {out_path}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())