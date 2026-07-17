# Radar Model Validation Results (Task 3)

Controlled, hand-computable checks of the core radar equations in `models/radar_like_model.py` (range/bearing/radial-velocity conversion, noise, P_D/P_FA, clutter, range-dependent SNR) - not the full swarm simulation, just the radar-domain math.

**Result: 52/52 checks passed** — all green

## Summary by task

| Task | Passed |
|---|---|
| range_calculation | 3/3 |
| bearing_calculation | 5/5 |
| radial_velocity_calculation | 5/5 |
| noisy_range_conversion | 3/3 |
| noisy_bearing_conversion | 2/2 |
| detected_xy_reconstruction | 2/2 |
| covariance_dimensions | 4/4 |
| pd_behavior | 7/7 |
| pfa_behavior | 6/6 |
| poisson_clutter_behavior | 6/6 |
| range_dependent_snr | 9/9 |

## Detailed results

| Status | Task | Description | Detail |
|---|---|---|---|
| PASS | range_calculation | 3-4-5 triangle gives range=5.0 | got 5.0 |
| PASS | range_calculation | range is translation-invariant (offset observer) | got 5.0 |
| PASS | range_calculation | coincident observer/target gives range=0.0 | got 0.0 |
| PASS | bearing_calculation | due east -> 0 rad | got 0.0000 rad |
| PASS | bearing_calculation | due north -> +pi/2 rad | got 1.5708 rad |
| PASS | bearing_calculation | due west -> +/-pi rad | got 3.1416 rad |
| PASS | bearing_calculation | due south -> -pi/2 rad | got -1.5708 rad |
| PASS | bearing_calculation | _wrap_angle folds 3*pi into (-pi, pi] | got -3.1416 |
| PASS | radial_velocity_calculation | target moving directly away gives radial_vel=+speed | got 3.0 |
| PASS | radial_velocity_calculation | target moving directly toward observer gives radial_vel=-speed | got -3.0 |
| PASS | radial_velocity_calculation | purely tangential motion gives radial_vel=0 | got 0.0 |
| PASS | radial_velocity_calculation | matching observer/target velocity gives radial_vel=0 (relative motion) | got 0.0 |
| PASS | radial_velocity_calculation | target_vel=None gives radial_vel=None | got None |
| PASS | noisy_range_conversion | zero-noise range equals true range (6-8-10 triangle) | got 10.0 |
| PASS | noisy_bearing_conversion | zero-noise bearing equals true bearing | got 0.9272952180016122 |
| PASS | noisy_range_conversion | sampled range noise std matches configured range_noise_std (~2.0) | sample std=1.981 |
| PASS | noisy_bearing_conversion | sampled bearing noise std matches configured bearing_noise_std (~0.0873 rad) | sample std=0.0862 rad |
| PASS | noisy_range_conversion | noisy range is floored at 0.05 (never goes non-positive) | min=3.0129 |
| PASS | detected_xy_reconstruction | zero-noise reconstructed x/y matches true target position | got (13.0000, -1.0000) |
| PASS | detected_xy_reconstruction | noisy x/y = uav_pos + measured_range*(cos,sin(measured_bearing)) | got (12.8741,-1.1378) vs expected (12.8741,-1.1378) |
| PASS | covariance_dimensions | measurement covariance is 3x3 (range, bearing, radial-velocity) | shape=3x3 |
| PASS | covariance_dimensions | measurement covariance is diagonal (no modeled cross-channel correlation) | off-diagonals=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] |
| PASS | covariance_dimensions | diagonal entries equal the reported per-channel variances | diag=[0.0900112503515625, 0.001218621992616375, 0.010001250039062503] |
| PASS | covariance_dimensions | all variances are non-negative |  |
| PASS | pd_behavior | P_D decreases as range grows (low SNR -> harder to detect) | pd_near=0.9500 pd_far=0.3455 |
| PASS | pd_behavior | P_D is lower in a storm than in clear weather at the same range | pd_clear=0.9500 pd_storm=0.7124 |
| PASS | pd_behavior | P_D is lower for a critical-reliability radar than a nominal one | pd_nominal=0.9500 pd_critical=0.5700 |
| PASS | pd_behavior | P_D stays within [0, 1] at range=0.5 | pd=0.9500 |
| PASS | pd_behavior | P_D stays within [0, 1] at range=10.0 | pd=0.9500 |
| PASS | pd_behavior | P_D stays within [0, 1] at range=100.0 | pd=0.9395 |
| PASS | pd_behavior | P_D stays within [0, 1] at range=1000.0 | pd=0.2916 |
| PASS | pfa_behavior | P_FA rises with clutter density (clutter_lambda) | pfa_low=0.0525 pfa_high=0.1750 |
| PASS | pfa_behavior | P_FA rises in a storm relative to clear weather | pfa_clear=0.0625 pfa_storm=0.1350 |
| PASS | pfa_behavior | P_FA stays within [0, 1] at clutter_lambda=0.0 | pfa=0.0500 |
| PASS | pfa_behavior | P_FA stays within [0, 1] at clutter_lambda=1.0 | pfa=0.0750 |
| PASS | pfa_behavior | P_FA stays within [0, 1] at clutter_lambda=10.0 | pfa=0.3000 |
| PASS | pfa_behavior | P_FA stays within [0, 1] at clutter_lambda=100.0 | pfa=1.0000 |
| PASS | poisson_clutter_behavior | Poisson(lambda=4.0) sample mean matches lambda | sample_mean=3.986 |
| PASS | poisson_clutter_behavior | Poisson(lambda=4.0) sample variance matches lambda (mean==variance) | sample_var=4.014 |
| PASS | poisson_clutter_behavior | Poisson(lambda=0) always returns 0 | got {0} |
| PASS | poisson_clutter_behavior | clutter_distribution='fixed' always generates round(clutter_lambda) candidates (PFA forced to 1.0) | counts=[3] |
| PASS | poisson_clutter_behavior | clutter_distribution='poisson' produces a varying candidate count (not constant) | distinct counts seen=12 |
| PASS | poisson_clutter_behavior | clutter_distribution='poisson' mean confirmed count matches clutter_lambda | mean=2.974 |
| PASS | range_dependent_snr | SNR equals reference_snr_db at reference_range (clear weather) | got 30.0 |
| PASS | range_dependent_snr | doubling range drops SNR by ~12.04 dB (4th-power falloff) | drop=12.0412 dB, expected 12.0412 dB |
| PASS | range_dependent_snr | SNR decreases monotonically as range increases | snrs=[60.0, 57.96, 30.0, 17.96, -10.0] |
| PASS | range_dependent_snr | storm attenuation (6 dB) subtracts directly from SNR at the same range | drop=6.0000 dB |
| PASS | range_dependent_snr | SNR is floored at SNR_DB_MIN for extremely long range | got -20.0 |
| PASS | range_dependent_snr | SNR is capped at SNR_DB_MAX for extremely short range | got 60.0 |
| PASS | range_dependent_snr | SNR is None for non-positive/unknown range |  |
| PASS | range_dependent_snr | measurement quality rises monotonically with SNR and stays in [0, 1] | qualities=[0.0099, 0.2403, 0.5, 0.7597, 0.9901, 1.0] |
| PASS | range_dependent_snr | quality at SNR=0 dB is exactly 0.5 (SNR/(SNR+1) with linear SNR=1) | got 0.5 |
