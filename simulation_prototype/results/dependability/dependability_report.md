# Task 23: Final Dependability Experiments

Run at 2026-07-23T06:19:46.886116+00:00, base_seed=42, wall_clock=82.8s.

## calibrated_vs_uncalibrated_confidence

Expected Calibration Error (lower = better calibrated). mildly_overconfident performed better on this metric (0.35654 vs 0.20876666666666666)

## fixed_vs_dynamic_trust

nearest-true-object fused position error, meters (lower = better). fixed performed better on this metric (1.9474 vs 1.95193)

## fixed_vs_adaptive_safety_margin

mission_success_rate (higher = better) plus collision_risk_count (lower = better). see aggregated_results.json

## no_abstention_vs_abstention

fraction of degraded/critical-perception steps left unmitigated (no abstention) vs given an active fallback decision (abstention). abstention_mitigated_trigger_rate performed better on this metric (1.0 vs 0.57238)

## no_handoff_vs_handoff

Not implemented: no "handoff" concept exists in this codebase to compare. See no_handoff_vs_handoff/handoff_stub.json for a proposed definition.

## centralized_vs_distributed_fusion

nearest-true-object fused position error, meters, under 40% packet loss. distributed performed better on this metric (0.27088 vs 0.26696)

## covariance_fusion_vs_covariance_intersection

nearest-true-object fused position error, meters. covariance_weighted performed better on this metric (1.80715 vs 2.42913)

## single_fault_vs_combined_fault

mission_success_rate (higher = better) plus collision_risk_count (lower = better). see aggregated_results.json

## normal_vs_ghost_aliasing_radar

nearest-true-object fused position error, meters, plus fused-track count inflation. ghost_aliasing performed better on this metric (0.49746999999999997 vs 0.46283)

