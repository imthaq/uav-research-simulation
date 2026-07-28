# Radar Model Validation Results

Controlled, hand-computable checks of the core radar-domain equations and behaviors in `radar_like_model.py` (range/bearing/radial-velocity, noise, Cartesian reconstruction, covariance, P_D/P_FA, Poisson clutter, range-dependent SNR, latency, dropout, scan-timestamp bookkeeping, and sensing-range limits) - not the full swarm simulation, just the radar-domain math and the radar-level scan machinery.

Two strategies are used: (1) a *bare* `RadarLikeModel` (constructed via `object.__new__`, bypassing `__init__`, which needs a full `Simulation`) for the pure-math checks; and (2) a *full* `RadarLikeModel` wired to a minimal fake `Simulation` stand-in (`_FakeSim`) for checks that need the real `_patch_perception` closure (max/min-range gating, radar dropout, and the scan-generation-timestamp/latency buffer) exercised against controlled UAV/target geometry.

`dependability.perception_quality_monitor` (imported unconditionally by `simple_swarm_sim.py`) was not part of this task's inputs; a minimal stub is used purely to satisfy the import chain and is not exercised by any check here.

**Total checks:** 99  |  **PASS:** 99  |  **FAIL:** 0

| # | Test Name | Configuration | Expected Result | Actual Result | Tolerance | Result | Correction Required |
|---|---|---|---|---|---|---|---|
| 1 | exact_range: 3-4-5 right triangle gives range=5.0 | _range_bearing_radial(observer=(0,0), target=(3,4)) | 5.0 | 5.0 | 1e-6 | PASS | - |
| 2 | exact_range: range is translation-invariant (offset observer, same displacement) | _range_bearing_radial(observer=(10,10), target=(13,14)) | 5.0 | 5.0 | 1e-6 | PASS | - |
| 3 | exact_range: coincident observer/target gives range=0.0 | _range_bearing_radial(observer=(5,5), target=(5,5)) | 0.0 | 0.0 | 1e-6 | PASS | - |
| 4 | exact_range: range exactly at a hypothetical boundary value computes exactly (15.0) | _range_bearing_radial(observer=(0,0), target=(15,0)) at radar_max_range=15 | 15.0 | 15.0 | 1e-6 | PASS | - |
| 5 | exact_bearing: due east -> 0 rad | target offset=(1.0,0.0) | 0.0000 rad | 0.0000 rad | 1e-6 | PASS | - |
| 6 | exact_bearing: due north -> +pi/2 rad | target offset=(0.0,1.0) | 1.5708 rad | 1.5708 rad | 1e-6 | PASS | - |
| 7 | exact_bearing: due west -> +/-pi rad | target offset=(-1.0,0.0) | 3.1416 rad | 3.1416 rad | 1e-6 | PASS | - |
| 8 | exact_bearing: due south -> -pi/2 rad | target offset=(0.0,-1.0) | -1.5708 rad | -1.5708 rad | 1e-6 | PASS | - |
| 9 | exact_bearing: northeast diagonal -> +pi/4 rad | target offset=(1.0,1.0) | 0.7854 rad | 0.7854 rad | 1e-6 | PASS | - |
| 10 | exact_bearing: _wrap_angle folds 3*pi into (-pi, pi] | _wrap_angle(3*pi) | +/-pi | -3.1416 | 1e-6 | PASS | - |
| 11 | exact_bearing: _wrap_angle folds -3*pi into (-pi, pi] | _wrap_angle(-3*pi) | +/-pi | -3.1416 | 1e-6 | PASS | - |
| 12 | exact_radial_velocity: oblique velocity projects onto line-of-sight correctly (3.0) | target=(3,4), target_vel=(5,0) | 3.0 | 3.0 | 1e-6 | PASS | - |
| 13 | exact_radial_velocity: relative (target-observer) velocity used, not absolute (3.0) | observer_vel=(1,0), target=(10,0), target_vel=(4,0) | 3.0 | 3.0 | 1e-6 | PASS | - |
| 14 | positive_radial_velocity: target moving directly away from observer -> true_radial_velocity = +3.0 | observer=(0,0) static, target=(10,0) moving vel=(3,0) | +3.0 | 3.0 | 1e-6 | PASS | - |
| 15 | positive_radial_velocity: ...and measured_radial_velocity matches (zero noise) | radial_velocity_noise_std=0.0 | +3.0 | 3.0 | 1e-6 | PASS | - |
| 16 | positive_radial_velocity: target receding directly along its own bearing -> full speed is radial (+10.0) | target=(8,6) range=10, vel=(8,6) i.e. along LOS | +10.0 | 10.0 | 1e-6 | PASS | - |
| 17 | negative_radial_velocity: target moving directly toward observer -> true_radial_velocity = -3.0 | observer=(0,0) static, target=(10,0) moving vel=(-3,0) | -3.0 | -3.0 | 1e-6 | PASS | - |
| 18 | negative_radial_velocity: ...and measured_radial_velocity matches (zero noise) | radial_velocity_noise_std=0.0 | -3.0 | -3.0 | 1e-6 | PASS | - |
| 19 | negative_radial_velocity: purely tangential target motion gives radial_velocity = 0.0 (not negative) | target=(10,0), vel=(0,5) i.e. perpendicular to LOS | 0.0 | 0.0 | 1e-6 | PASS | - |
| 20 | stationary_target: zero-velocity target gives true_radial_velocity = 0.0 | target=(7,3) vel=(0,0), observer static | 0.0 | 0.0 | 1e-6 | PASS | - |
| 21 | stationary_target: ...and measured_radial_velocity = 0.0 (zero noise) | radial_velocity_noise_std=0.0 | 0.0 | 0.0 | 1e-6 | PASS | - |
| 22 | stationary_target: stationary target + moving observer gives nonzero relative radial velocity | target=(7,3) vel=(0,0), observer moving vel=(2,0) | nonzero | -1.8383 | n/a (nonzero check) | PASS | - |
| 23 | stationary_target: id='obstacle_0' is always treated as stationary (radial_velocity=0.0) | target_id='obstacle_0', at (5,0) | 0.0 | 0.0 | 1e-6 | PASS | - |
| 24 | stationary_target: unrecognized target id has no kinematic model -> true_radial_velocity is None (documented, not a bug) | target_id='clutter_1' (not 'obstacle_0' or 'uav_N') | None | None | n/a (None check) | PASS | - |
| 25 | noisy_range: zero-noise measured_range equals true range (6-8-10 triangle) | range_noise_std=0.0, target=(6,8) | 10.0 | 10.0 | 1e-6 | PASS | - |
| 26 | noisy_bearing: zero-noise measured_bearing equals true bearing | bearing_noise_std=0.0, target=(6,8) | 0.9273 | 0.9273 | 1e-6 | PASS | - |
| 27 | noisy_range: sampled range-noise std matches configured range_noise_std (2.0) | range_noise_std=2.0, n=4000 trials | ~2.0 | 1.981 | 0.15 | PASS | - |
| 28 | noisy_bearing: sampled bearing-noise std matches configured bearing_noise_std (0.0873 rad) | bearing_noise_std=5deg, n=4000 trials | ~0.0873 rad | 0.0862 rad | 0.01 rad | PASS | - |
| 29 | noisy_range: noisy range is floored at 0.05 (never non-positive) | range_noise_std=2.0, n=4000 trials | >= 0.05 | min=3.0129 | n/a (hard floor) | PASS | - |
| 30 | noisy_range: range-noise std under 'storm' matches the model's own reported range_variance | range_noise_std=1.0, environmental_condition='storm', n=3000 | ~1.600 | 1.579 | 0.1 | PASS | - |
| 31 | noisy_radial_velocity: sampled measured_radial_velocity mean matches true radial velocity (+5.0) | radial_velocity_noise_std=0.4, target vel=(5,0), n=4000 | ~5.0 | 4.9888 | 0.05 | PASS | - |
| 32 | noisy_radial_velocity: sampled measured_radial_velocity std matches the model's own reported radial_velocity_variance | radial_velocity_noise_std=0.4, n=4000 | ~0.4000 | 0.3911 | 0.03 | PASS | - |
| 33 | cartesian_reconstruction: zero-noise reconstructed x/y matches true target position exactly | observer=(10,-5), target=(13,-1), zero noise | (13.0, -1.0) | (13.0000, -1.0000) | 1e-6 | PASS | - |
| 34 | cartesian_reconstruction: noisy x/y = uav_pos + measured_range*(cos,sin(measured_bearing)) (self-consistent) | range_noise_std=1.5, bearing_noise_std=3deg | (12.8741, -1.1378) | (12.8741, -1.1378) | 1e-9 | PASS | - |
| 35 | cartesian_reconstruction: round-trip range/bearing recovered from x/y matches measured_range/measured_bearing | derived from previous noisy case | range=4.8143, bearing=0.9310 | range=4.8143, bearing=0.9310 | 1e-9 | PASS | - |
| 36 | covariance_dimensions: measurement covariance is 3x3 (range, bearing, radial-velocity) | range=25.0, defaults | 3x3 | 3x3 | exact | PASS | - |
| 37 | covariance_dimensions: measurement covariance is diagonal (no modeled cross-channel correlation) | range=25.0, defaults | off-diagonals == 0.0 | [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] | exact | PASS | - |
| 38 | covariance_dimensions: diagonal entries equal the individually reported per-channel variances | range=25.0, defaults | [0.0900112503515625, 0.001218621992616375, 0.010001250039062503] | [0.0900112503515625, 0.001218621992616375, 0.010001250039062503] | 1e-6 | PASS | - |
| 39 | covariance_positivity: all variances are non-negative at range=0.5 | range=0.5, defaults | >= 0 | diag=[0.09000018000009004, 0.0012184721160874116, 0.010000020000010004] | n/a | PASS | - |
| 40 | covariance_positivity: all variances are non-negative at range=5.0 | range=5.0, defaults | >= 0 | diag=[0.09000018000009004, 0.0012184721160874116, 0.010000020000010004] | n/a | PASS | - |
| 41 | covariance_positivity: all variances are non-negative at range=25.0 | range=25.0, defaults | >= 0 | diag=[0.0900112503515625, 0.001218621992616375, 0.010001250039062503] | n/a | PASS | - |
| 42 | covariance_positivity: all variances are non-negative at range=100.0 | range=100.0, defaults | >= 0 | diag=[0.09290304, 0.0012577726371173947, 0.010322560000000001] | n/a | PASS | - |
| 43 | covariance_positivity: all variances are non-negative at range=1000.0 | range=1000.0, defaults | >= 0 | diag=[36.0, 0.4873878716587337, 4.0] | n/a | PASS | - |
| 44 | covariance_positivity: variances are strictly positive at range=1.0, condition=clear (nonzero noise stds configured) | range=1.0, condition=clear, nonzero base noise stds | > 0 | diag=[0.09000018000009004, 0.00040000080000040015, 0.010000020000010004] | n/a | PASS | - |
| 45 | covariance_positivity: variances are strictly positive at range=500.0, condition=clear (nonzero noise stds configured) | range=500.0, condition=clear, nonzero base noise stds | > 0 | diag=[10.889999999999999, 0.0484, 1.2100000000000002] | n/a | PASS | - |
| 46 | covariance_positivity: variances are strictly positive at range=25.0, condition=storm (nonzero noise stds configured) | range=25.0, condition=storm, nonzero base noise stds | > 0 | diag=[0.23051466912915813, 0.0010245096405740361, 0.025612741014350907] | n/a | PASS | - |
| 47 | covariance_positivity: range variance under storm+critical-reliability is >= variance under clear+nominal | range=25.0; clear/nominal vs storm/critical | >= 0.09001 | 1.44072 | n/a | PASS | - |
| 48 | pd_behavior: P_D decreases as range grows (low SNR -> harder to detect) | range=1.0 vs range=500.0 | pd(500) < pd(1) | pd_near=0.9500 pd_far=0.3455 | n/a | PASS | - |
| 49 | pd_behavior: P_D is lower in a storm than in clear weather at the same range | range=25.0; clear vs storm | pd(storm) < pd(clear) | pd_clear=0.9500 pd_storm=0.7124 | n/a | PASS | - |
| 50 | pd_behavior: P_D is lower for a critical-reliability radar than a nominal one | range=25.0; nominal vs critical reliability | pd(critical) < pd(nominal) | pd_nom=0.9500 pd_crit=0.5700 | n/a | PASS | - |
| 51 | pd_behavior: P_D stays within [0, 1] at range=0.5 | range=0.5, defaults | [0,1] | 0.9500 | n/a | PASS | - |
| 52 | pd_behavior: P_D stays within [0, 1] at range=10.0 | range=10.0, defaults | [0,1] | 0.9500 | n/a | PASS | - |
| 53 | pd_behavior: P_D stays within [0, 1] at range=100.0 | range=100.0, defaults | [0,1] | 0.9395 | n/a | PASS | - |
| 54 | pd_behavior: P_D stays within [0, 1] at range=1000.0 | range=1000.0, defaults | [0,1] | 0.2916 | n/a | PASS | - |
| 55 | pfa_behavior: P_FA rises with clutter density (clutter_lambda) | range=25.0; clutter_lambda=0.1 vs 5.0 | pfa(5.0) > pfa(0.1) | pfa_low=0.0525 pfa_high=0.1750 | n/a | PASS | - |
| 56 | pfa_behavior: P_FA rises in a storm relative to clear weather | range=25.0; clear vs storm | pfa(storm) > pfa(clear) | pfa_clear=0.0625 pfa_storm=0.1350 | n/a | PASS | - |
| 57 | pfa_behavior: P_FA stays within [0, 1] at clutter_lambda=0.0 | range=25.0, clutter_lambda=0.0 | [0,1] | 0.0500 | n/a | PASS | - |
| 58 | pfa_behavior: P_FA stays within [0, 1] at clutter_lambda=1.0 | range=25.0, clutter_lambda=1.0 | [0,1] | 0.0750 | n/a | PASS | - |
| 59 | pfa_behavior: P_FA stays within [0, 1] at clutter_lambda=10.0 | range=25.0, clutter_lambda=10.0 | [0,1] | 0.3000 | n/a | PASS | - |
| 60 | pfa_behavior: P_FA stays within [0, 1] at clutter_lambda=100.0 | range=25.0, clutter_lambda=100.0 | [0,1] | 1.0000 | n/a | PASS | - |
| 61 | poisson_clutter: Poisson(lambda=4.0) sample mean matches lambda | lambda=4.0, n=20000 | ~4.0 | 3.986 | 0.1 | PASS | - |
| 62 | poisson_clutter: Poisson(lambda=4.0) sample variance matches lambda (mean==variance) | lambda=4.0, n=20000 | ~4.0 | 4.014 | 0.3 | PASS | - |
| 63 | poisson_clutter: Poisson(lambda=0) always returns 0 | lambda=0.0, n=50 | {0} | {0} | exact | PASS | - |
| 64 | poisson_clutter: clutter_distribution='fixed' always generates round(clutter_lambda) candidates (PFA forced to 1.0) | clutter_lambda=3.0, distribution='fixed', PFA=1.0, n=30 | {3} | [3] | exact | PASS | - |
| 65 | poisson_clutter: clutter_distribution='poisson' produces a varying candidate count (not constant) | clutter_lambda=3.0, distribution='poisson', PFA=1.0, n=2000 | > 1 distinct value | 12 distinct values | n/a | PASS | - |
| 66 | poisson_clutter: clutter_distribution='poisson' mean confirmed count matches clutter_lambda | clutter_lambda=3.0, distribution='poisson', PFA=1.0, n=2000 | ~3.0 | 2.974 | 0.2 | PASS | - |
| 67 | range_dependent_snr: SNR equals reference_snr_db at reference_range (clear weather) | range=reference_range=50.0, clear | 30.0 | 30.0 | 1e-6 | PASS | - |
| 68 | range_dependent_snr: doubling range drops SNR by ~12.04 dB (4th-power falloff) | range=100.0 (2x reference), snr_exponent=4.0 | 12.0412 dB drop | 12.0412 dB drop | 1e-6 | PASS | - |
| 69 | range_dependent_snr: SNR decreases monotonically as range increases | ranges=[1,10,50,100,500] | strictly decreasing | [60.0, 57.96, 30.0, 17.96, -10.0] | n/a | PASS | - |
| 70 | range_dependent_snr: storm attenuation (6 dB) subtracts directly from SNR at the same range | range=50.0; clear vs storm | 6.0 dB drop | 6.0000 dB drop | 1e-6 | PASS | - |
| 71 | range_dependent_snr: SNR is floored at SNR_DB_MIN for extremely long range | range=1e9 | -20.0 | -20.0 | 1e-6 | PASS | - |
| 72 | range_dependent_snr: SNR is capped at SNR_DB_MAX for extremely short range | range=1e-9 | 60.0 | 60.0 | 1e-6 | PASS | - |
| 73 | range_dependent_snr: SNR is None for non-positive/unknown range | range in {None, 0.0, -5.0} | None | True | exact | PASS | - |
| 74 | range_dependent_snr: measurement quality rises monotonically with SNR and stays in [0, 1] | SNR=[-20,-5,0,5,20,60] dB | monotonic, in [0,1] | [0.0099, 0.2403, 0.5, 0.7597, 0.9901, 1.0] | n/a | PASS | - |
| 75 | range_dependent_snr: quality at SNR=0 dB is exactly 0.5 (SNR/(SNR+1) with linear SNR=1) | SNR=0 dB | 0.5 | 0.5 | 1e-6 | PASS | - |
| 76 | latency: nothing has arrived yet at t=1 given latency=3 and earliest scan gen_t=0 (needs t>=3) | radar_latency_steps=3, scan buffer gen_t=[0,2], query t=1 | None | None | exact | PASS | - |
| 77 | latency: scan generated at t=0 has arrived by t=3 (3-step latency elapsed) | radar_latency_steps=3, scan buffer gen_t=[0,2], query t=3 | scanA | ['scanA'] | exact | PASS | - |
| 78 | latency: by t=5, the most recent arrived scan is gen_t=2's (t=5-3=2 cutoff) | radar_latency_steps=3, scan buffer gen_t=[0,2], query t=5 | scanB | ['scanB'] | exact | PASS | - |
| 79 | latency: with radar_latency_steps=0, a scan is available the same step it was generated | radar_latency_steps=0, scan gen_t=4, query t=4 | scanC | ['scanC'] | exact | PASS | - |
| 80 | latency: with radar_latency_steps=3, a static in-range target is NOT perceived at t=0,1,2 | radar_latency_steps=3, static target at range=5.0 within radar_max_range=15.0 | [] at t=0,1,2 | [[], [], []] | exact | PASS | - |
| 81 | latency: ...and IS perceived starting at t=3 (once the 3-step delay has elapsed) | radar_latency_steps=3, static target at range=5.0 | ['obstacle_0'] at t=3,4,5 | [['obstacle_0'], ['obstacle_0'], ['obstacle_0']] | exact | PASS | - |
| 82 | dropout: empirical radar dropout rate matches configured radar_dropout_probability=0.0 | radar_dropout_probability=0.0, n=5000 | ~0.0 | 0.0000 | 0.03 | PASS | - |
| 83 | dropout: empirical radar dropout rate matches configured radar_dropout_probability=0.3 | radar_dropout_probability=0.3, n=5000 | ~0.3 | 0.3000 | 0.03 | PASS | - |
| 84 | dropout: empirical radar dropout rate matches configured radar_dropout_probability=1.0 | radar_dropout_probability=1.0, n=5000 | ~1.0 | 1.0000 | 0.03 | PASS | - |
| 85 | dropout: radar_dropout_probability=1.0 -> this scan is a total blackout (perceived=[], dropout=True) | radar_dropout_probability=1.0, target in range | perceived=[], dropout=True | perceived=[], dropout=True | exact | PASS | - |
| 86 | dropout: radar_dropout_probability=0.0 -> no blackout, in-range target is perceived | radar_dropout_probability=0.0, target in range | perceived=['obstacle_0'], dropout=False | perceived=['obstacle_0'], dropout=False | exact | PASS | - |
| 87 | timestamp_behavior: row's time_step field equals the step index passed in (t=0) | t=0 | 0 | 0 | exact | PASS | - |
| 88 | timestamp_behavior: row's time_step field equals the step index passed in (t=1) | t=1 | 1 | 1 | exact | PASS | - |
| 89 | timestamp_behavior: row's time_step field equals the step index passed in (t=7) | t=7 | 7 | 7 | exact | PASS | - |
| 90 | timestamp_behavior: row's time_step field equals the step index passed in (t=42) | t=42 | 42 | 42 | exact | PASS | - |
| 91 | timestamp_behavior: at query t=2 with latency=2, only the gen_t=0 scan has arrived (cutoff=t-latency=0) | radar_latency_steps=2, buffer gen_t=[0,1,2], query t=2 | s0 | ['s0'] | exact | PASS | - |
| 92 | timestamp_behavior: consumed/older buffer entries are pruned after being served (buffer shrinks) | after _get_delayed_scan(0, 2) call | [(1,...),(2,...)] remaining | [(1, ['s1'], False, []), (2, ['s2'], False, [])] | exact | PASS | - |
| 93 | timestamp_behavior: querying with an empty scan buffer returns None (nothing has arrived) | radar_latency_steps=5, empty buffer, query t=0 | None | None | exact | PASS | - |
| 94 | maximum_sensing_range: radar_max_range defaults to sim.sensor_range when not overridden | no radar_max_range override, sim.sensor_range=12.5 | 12.5 | 12.5 | 1e-6 | PASS | - |
| 95 | maximum_sensing_range: target within radar_max_range is perceived (range=5.0 < max=15.0) | radar_max_range=15.0, target range=5.0 | perceived | perceived_ids={'obstacle_0', 'uav_2'} | n/a | PASS | - |
| 96 | maximum_sensing_range: target beyond radar_max_range is gated out with radar_pd_miss_flag equivalent (pd_missed_ids) | radar_max_range=15.0, target range=25.0 | in pd_missed_ids, not perceived | pd_missed_ids=['uav_1'], perceived_ids={'obstacle_0', 'uav_2'} | n/a | PASS | - |
| 97 | maximum_sensing_range: target exactly at radar_max_range boundary (range==max) is NOT gated (condition is strict '>') | radar_max_range=15.0, target range=15.0 (exact boundary) | perceived | perceived_ids={'obstacle_0', 'uav_2'} | n/a | PASS | - |
| 98 | maximum_sensing_range: target inside the radar_min_range blind zone is gated out | radar_min_range=2.0, target range=1.0 | in pd_missed_ids, not perceived | pd_missed_ids=['obstacle_0'], perceived_ids={'uav_1'} | n/a | PASS | - |
| 99 | maximum_sensing_range: target beyond the blind zone but within max range is perceived normally | radar_min_range=2.0, radar_max_range=15.0, target range=5.0 | perceived | perceived_ids={'uav_1'} | n/a | PASS | - |
