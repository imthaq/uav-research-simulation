# Result Sanity Checks

## 1. Lower P_D generally increases missed detections
**P_D** (probability of detection) is the chance the radar registers a real target that's actually there each time it scans; a "missed detection" is a real target the radar failed to report that step.
**Confirmed.** `baseline` (P_D=1.0) had a 0.23% missed-detection rate; `very_low_P_D` (P_D=0.1) had 87.75%  trend holds strongly in the expected direction.

## 2. Higher P_FA generally increases false tracks
**P_FA** (probability of false alarm) is the per-candidate chance a clutter/noise blip gets confirmed and reported as if it were a real detection; a "false track" is a track formed from one of these non-existent targets.
**Unexpected — investigated.** `very_high_P_FA` (P_FA=0.4) produced a 0.0% false-alarm rate, identical to baseline, because that scenario left `radar_clutter_density` at 0.0  with zero clutter *candidates* generated, there's nothing for the 0.4 P_FA to ever confirm. Manually setting both `radar_clutter_density>0` and P_FA>0 together does produce false alarms (~7.3%), confirming this is a scenario-config gap (two dependent parameters, only one set), not a broken P_FA mechanism.

## 3. Greater clutter generally increases association difficulty
**Clutter** is the density of spurious radar returns (environmental noise, not real objects) that compete with real targets for the tracker's nearest-neighbor association; "association difficulty" shows up as more mismatches/fragmented tracks.
**Unexpected — investigated.** `high_clutter` (clutter_density=0.5) also showed a 0.0% clutter/false-alarm rate, for the mirror-image reason: it left `radar_false_alarm_probability` at 0.0, so every generated clutter candidate had zero chance of being confirmed. Same root cause as #2  the two scenarios each set only one half of a two-part mechanism.

## 4. Higher latency generally increases response time
**Latency** here is `radar_latency_steps`/`latency_steps`, the delay (in simulation steps) between a detection happening and it reaching the decision loop; "response time" is how long a UAV takes to react to a hazard once it's detectable.
**Confirmed.** `baseline` (latency=0) averaged 0.0s response time; `high_latency` (latency=20 steps, ~2s) averaged 4.0s  a large, expected increase.

## 5. Higher packet loss generally damages distributed fusion
**Packet loss** is the per-message chance an inter-UAV track broadcast never arrives (modeled by `CommunicationChannel`); "distributed fusion" is combining multiple UAVs' track reports into one shared estimate over that unreliable link.
**Unexpected — investigated.** `perfect_communication` and `high_packet_loss` (40% loss) produced byte-identical metrics (mission_success_rate=1.0, avg_formation_error=3.7777, collision_risk=0 for both). Root cause: the live simulation's decision loop (`run_radar_track_fusion_pipeline`) calls `fuse_step()` without ever constructing or passing a `CommunicationChannel`, so the `"communication"` config block scenarios like `high_packet_loss` set is never actually applied to the live run  it's wired only into the separate offline `build_fused_log` evaluation path. This is a real implementation gap, not a robustness result, and should be fixed before packet-loss scenarios are trusted.

## 6. Better fusion should not systematically perform worse because of an implementation error
"Better fusion" means a weighting scheme (confidence/trust/covariance) that's supposed to down-weight a known-bad sensor more effectively than plain averaging; here tested against `faulty_sensor_*` (one UAV reports a biased position at forced-high confidence).
**Mostly as designed, one point flagged.** `trust_weighted_fusion_fixed` (0.6) outscoring `trust_weighted_fusion_dynamic` (0.467). This gap is within plausible noise (9 vs 7 successes), so it's flagged for a larger-sample rerun rather than called a confirmed regression.
