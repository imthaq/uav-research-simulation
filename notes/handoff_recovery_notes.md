# Handoff Recovery Notes (Task 16)

Design notes for recovery / return-to-normal behavior after a Task 14/15 abstention or handoff episode ends. Each check below gates the next — none is skipped just because an earlier one passed.

## 1. Verify sensor quality improves
Re-run `PerceptionQualityMonitor.evaluate()` each step and require the composite score to clear `resume_threshold` (GOOD), not merely exit CRITICAL.
A single good reading isn't proof — it only starts the stability count in step 2.

## 2. Wait for a configurable number of stable updates
Require `stable_updates_required` (config knob, default e.g. 5) consecutive GOOD-quality steps before declaring recovery, matching `max_hold_steps`'s "how many is enough" convention.
Any single DEGRADED/CRITICAL reading during the window resets the counter to zero rather than just pausing it.

## 3. Restore normal speed gradually
Once stable, ramp the reduced-speed multiplier back to 1.0 linearly over `speed_recovery_steps`, not in one jump.
Ramp rate is capped (e.g. +0.1/step) so control inputs stay smooth even if quality flickers mid-ramp.

## 4. Reduce enlarged safety margin gradually
Shrink the widened formation/collision margin back to nominal over `margin_recovery_steps`, symmetric to how it was enlarged on entry into DEGRADED/CRITICAL.
Margin never shrinks faster than one step's worth of true position uncertainty reduction, so it can't outrun the sensors it's trusting.

## 5. Return from fallback fusion to normal fusion
Switch the fusion source back (radar-only/LiDAR-only/peer/centralized → nominal multi-sensor fusion) only after speed and margin ramps both complete, not the instant quality clears.
The old fallback source stays live in parallel for one overlap step so the switch itself introduces no measurement gap.

## 6. Avoid rapid switching between modes
Enforce a minimum dwell time (`min_mode_dwell_steps`) in any mode, fallback or normal, before another transition is allowed.
A quality dip during dwell is logged as a near-miss recovery, not acted on, unless it re-crosses the original CRITICAL trigger threshold.

## Hysteresis (prevents oscillation)
Use separate enter/exit thresholds — enter fallback at `CRITICAL_THRESHOLD` (0.4) but only exit back to normal at a higher `resume_threshold` (e.g. 0.7, GOOD) — so borderline scores can't flip the mode every step.
Combined with the dwell timer in step 6, a mode change requires both a threshold crossing *and* enough elapsed time, which is what actually stops the oscillation either check alone would miss.