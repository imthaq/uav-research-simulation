#Final Dependability Experiments

Run at 2026-07-28T06:39:54.289547+00:00, base_seed=42, wall_clock=61.8s.

## calibrated_vs_uncalibrated_confidence

Expected Calibration Error (lower = better calibrated). mildly_overconfident performed better on this metric (0.35685 vs 0.20925)

## fixed_vs_dynamic_trust

nearest-true-object fused position error, meters (lower = better). fixed performed better on this metric (1.95215 vs 1.9571999999999998)

## fixed_vs_adaptive_safety_margin

mission_success_rate (higher = better) plus collision_risk_count (lower = better). see aggregated_results.json

## no_abstention_vs_abstention

fraction of degraded/critical-perception steps left unmitigated (no abstention) vs given an active fallback decision (abstention). abstention_mitigated_trigger_rate performed better on this metric (1.0 vs 0.5408)

## no_handoff_vs_handoff

Not implemented: no "handoff" concept exists in this codebase to compare. See no_handoff_vs_handoff/handoff_stub.json for a proposed definition.

## centralized_vs_distributed_fusion

nearest-true-object fused position error, meters, under 40% packet loss. distributed performed better on this metric (0.27085000000000004 vs 0.2704)

## covariance_fusion_vs_covariance_intersection

nearest-true-object fused position error, meters. covariance_weighted performed better on this metric (1.88395 vs 2.4255)

## single_fault_vs_combined_fault

mission_success_rate (higher = better) plus collision_risk_count (lower = better). see aggregated_results.json

## normal_vs_ghost_aliasing_radar

nearest-true-object fused position error, meters, plus fused-track count inflation. normal performed better on this metric (0.4779 vs 0.48605)

