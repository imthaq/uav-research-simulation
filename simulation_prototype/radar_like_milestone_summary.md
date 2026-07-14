# Radar-Like Multimodal Simulation Milestone — v2 Freeze

**Date frozen:** 2026-07-10  
**Status:** Complete — radar-like sensor model integrated and validated

---

## Radar Model Features Completed

- Range/bearing/Doppler-like measurements per detection
- True target range and bearing calculation from Cartesian positions
- Radial velocity (Doppler proxy) derivation from frame-to-frame UAV motion
- Range/bearing noise injection via Gaussian error at radar reporting stage
- Coordinate transformation: noisy range/bearing back to x/y for UAV steering
- Radar measurement logging in 36-column CSV schema (time_step, radar_id, target_id, positions, velocities, true/measured ranges/bearings/radial_velocity, confidence, detection flags)

---

## Radar Parameters Implemented

**Sensing Limits & Dynamics**
- `radar_max_range` / `radar_min_range` — enforces range gating; out-of-range detections flagged and excluded
- `radar_field_of_view` — angular sector (degrees) around UAV's goal-heading direction
- `radar_update_rate` — scan frequency (Hz); between scans, previous scan held and re-served
- `radar_latency_steps` — additional delay (in steps) beyond update rate before scan reaches controller
- `radar_dropout_probability` — per-scan chance of total radar blackout

**Measurement Quality**
- `radar_confidence_error` — extra Gaussian miscalibration applied to detection confidence scores
- `radar_range_noise_m` / `radar_bearing_noise_deg` — noise injected into range and bearing measurements
- `radar_detection_probability` (P_D) — Bernoulli gate; real detections independently pass with this probability, increasing genuine missed detections
- `radar_clutter_density` — Poisson rate for clutter candidate returns per scan
- `radar_false_alarm_probability` (P_FA) — probability clutter candidate becomes a reported detection

---

## Tracking Method Currently Used

**Frame-to-Frame Detection Snapshots**
- Per-step detection snapshots captured after perception and fusion stages
- Each snapshot holds: true target position/velocity, measured range/bearing/Doppler, detected position, confidence, and detection flags (is_phantom, missed, false_alarm, clutter, dropout, radar_pd_miss)
- No persistent track history or multi-frame data association
- Detection-to-detection steering: UAV steers on current-scan measurements, not filtered history
- Velocity estimated from consecutive step positions (dx/dt, dy/dt)

---

## Fusion Modes Implemented

1. **no_fusion** — each UAV acts independently on its own detections
2. **naive_fusion** — unweighted average of positions from all UAVs currently perceiving an obstacle
3. **trust_weighted_fusion** — confidence-weighted average; high-confidence reports pull the fused estimate toward them
4. **Fusion recovery** — when fusion fills a gap, contributes to `fusion_recovery_count` per run

---

## Scenarios Completed

**9 total scenarios**, each testing a specific perception-error axis:

1. **baseline** — no errors; ground truth sensing
2. **false_positive** — 20% phantom detection rate
3. **false_negative** — 20% missed detection rate
4. **sensor_noise** — Gaussian position error (±0.5 m)
5. **latency** — 2-step detection delay
6. **sensor_dropout** — 10% per-step total blackout
7. **confidence_error** — Gaussian miscalibration of confidence scores (±0.3)
8. **no_fusion_matched** — false_positive + false_negative, no fusion (baseline for comparison)
9. **naive_fusion** — false_positive + false_negative, naive fusion
10. **trust_weighted_fusion** — false_positive + false_negative, trust-weighted fusion

---

## Number of Experiment Runs

- **84 total CSV logs** in `logs/` directory
- Run across 9 scenarios with multiple seeded trials
- Covers all combinations of perception errors, fusion modes, and random seeds
- Each run logs full step-by-step state: UAV positions, velocities, detections, fused obstacles, mission status

---

## Plots Generated

**6 PNG charts** in `plots/` directory:

1. `baseline_vs_error_scenarios.png` — mission success rate and collision risk across all 9 scenarios
2. `dropout_vs_mission_success.png` — impact of sensor dropout on success/failure
3. `false_negative_vs_collision_risk.png` — missed detections and collision outcomes
4. `false_positive_vs_unnecessary_avoidance.png` — phantom detections and wasted maneuvers
5. `fusion_mode_vs_safety_metrics.png` — no_fusion vs naive_fusion vs trust_weighted_fusion (safety and latency)
6. `latency_vs_response_time.png` — delay effects on detection-to-avoidance reaction time

---

## Videos Generated

**11 MP4/GIF files** in `videos/` directory:

- Per-scenario replay animations (4 UAVs, obstacle, target, detection markers, fusion state)
- Fusion-mode comparison montages
- Live-simulation recordings captured during experimental runs
- Format: 30 fps, ~5–10 seconds per scenario, includes real-time telemetry overlay

---

## Current Limitations

- **No multi-frame track association:** each detection is independent; no history linking detections across steps to a persistent track ID
- **No Kalman filter or state estimator:** detection stream is raw; UAV steers directly on noisy measurements without smoothing
- **Radar updates synchronous to main loop:** radar_update_rate is emulated via hold/re-serve, not asynchronous interrupts
- **No radar cross-section (RCS) model:** all real detections have equal probability of being reported; no target-size or aspect-angle dependence
- **No multi-hypothesis tracking (MHT):** ambiguous detections (e.g., two targets close together) are resolved by position alone, no track probability gates
- **Clutter is Poisson point process:** spatially random, no range-dependent or weather-dependent clutter model
- **No explicit gating or pre-association filtering:** all detections fed to fusion without spatial-kinematic plausibility gates

---

## Planned Upgrades

1. **Track history and kinematic filtering**
   - Maintain per-detection history across N steps
   - Kalman filter or other state estimator to smooth noisy measurements
   - Multi-step association for improved collision-avoidance decisions

2. **Persistent track IDs**
   - Frame-to-frame detection data association (nearest-neighbor, Hungarian algorithm, MHT)
   - Track birth/death logic (initiation threshold, termination rules)
   - Reduced false-alarm reactivity via track-level confidence gates

3. **Radar cross-section (RCS) model**
   - Detection probability as function of target size, aspect angle, range
   - Range-dependent SNR threshold
   - More realistic P_D behavior

4. **Advanced tracking modes**
   - Extended Kalman Filter (EKF) with constant-velocity or coordinated-turn model
   - Unscented Kalman Filter (UKF) for nonlinear range/bearing measurements
   - Particle filter for multi-modal uncertainty

5. **Clutter and false alarm refinement**
   - Range-dependent clutter density (weather model)
   - Clutter spatial clustering (ground reflection patches)
   - Statistical false alarm rate control (CFAR)

6. **Asynchronous sensor simulation**
   - Event-driven radar updates independent of main simulation loop
   - Realistic scan-line timing and inter-scan hold behavior
   - Multi-beam or phased-array patterns

7. **Validation suite**
   - Synthetic target insertion for P_D/P_FA verification
   - Comparison with reference tracking benchmarks (e.g., AAAI multi-object tracking data)
   - Sensitivity analysis across radar parameter sweep ranges
