# Radar Model Validation Results

Ran `radar_model_validation.py` against `radar_like_model.py`: 52/52 controlled-case checks passed. Full machine-readable output in `results/radar_model_validation_results.json`.

## Range calculation
Verified `_range_bearing_radial` against a 3-4-5 triangle (range=5), confirmed it's translation-invariant under an offset observer, and confirmed a coincident observer/target gives range=0. All 3/3 passed.

## Bearing calculation
Checked all four cardinal directions (east=0, north=+π/2, west=±π, south=−π/2) plus `_wrap_angle` folding 3π back into (−π, π]. All 5/5 passed.

## Radial-velocity calculation
Confirmed a target receding gives +speed, approaching gives −speed, purely tangential motion gives 0, matching observer/target velocities cancel to 0 (true relative motion), and `target_vel=None` correctly yields `None`. All 5/5 passed.

## Noisy range conversion
With noise std=0, measured range exactly matched true range; with std=2.0 over 4000 trials, sampled std came out to 1.981; confirmed the 0.05 floor is respected. All 3/3 passed.

## Noisy bearing conversion
With noise std=0, measured bearing exactly matched true bearing; with std≈0.0873 rad over 4000 trials, sampled std came out to 0.0862 rad. Both 2/2 passed.

## Detected x/y reconstruction
Zero-noise reconstruction reproduced the true target position exactly; with noise, reconstructed x/y matched `uav_pos + measured_range*(cos,sin(measured_bearing))` to floating-point precision, confirming self-consistency with the logged range/bearing. Both 2/2 passed.

## Covariance dimensions
`measurement_covariance` is a 3x3 matrix (range, bearing, radial-velocity), strictly diagonal (no cross-channel correlation is modeled), with diagonal entries matching the reported per-channel variances and all non-negative. All 4/4 passed.

## P_D behavior
Effective P_D dropped from 0.95 (range=1) to 0.35 (range=500) as SNR fell; storm weather and critical hardware-reliability state each independently lowered P_D relative to clear/nominal baselines; P_D stayed within [0,1] across a wide range sweep. All 7/7 passed.

## P_FA behavior
Effective P_FA rose with clutter density (0.05→0.18 as `clutter_lambda` went 0.1→5.0) and with storm weather (0.06→0.14 vs. clear); stayed within [0,1] even at extreme `clutter_lambda`. All 6/6 passed.

## Poisson clutter behavior
20,000-sample draw at λ=4 gave mean≈3.99 and variance≈4.01 (Poisson's mean=variance property holds); λ=0 always returned 0; `clutter_distribution="fixed"` deterministically produced exactly 3 candidates every call, while `"poisson"` produced a varying count averaging ≈2.97. All 6/6 passed.

## Range-dependent SNR
SNR exactly equaled `reference_snr_db` at `reference_range`; doubling range dropped SNR by 12.04 dB, matching the expected `4*10*log10(2)` for the 4th-power falloff; SNR decreased monotonically with range, was clamped to `[SNR_DB_MIN, SNR_DB_MAX]` at extremes, and storm attenuation subtracted exactly 6 dB. Quality-from-SNR was monotonic, bounded to [0,1], and equaled 0.5 exactly at 0 dB. All 9/9 passed.
