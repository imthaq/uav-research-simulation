# Fusion Consistency Analysis

Sensor-correlation and fusion-consistency tests for `fusion_model.py`'s two
covariance-aware fusion modes: `covariance_weighted_fusion` (an
inverse-covariance / information-filter combine, statistically optimal
only when sources' errors are independent) and
`covariance_intersection_fusion` (CI, designed to stay *consistent* even
when that independence assumption doesn't hold). All numbers below come
from `fusion_consistency_validation.py`, which calls the real
`_information_fusion_xy` / `_covariance_intersection_xy` functions in
`fusion/fusion_model.py` directly - this is the production fusion math
under test, not a reimplementation of it.

**Reproduce:** `python fusion_consistency_validation.py` (seed 42, 5,000
Monte Carlo trials per cell; writes `results/fusion_consistency_results.json`
and `results/fusion_consistency_checks.md`).

## Methodology

Four synthetic UAV radars (`sigma` = 0.6, 0.8, 0.7, 0.9 world units,
matching this project's default swarm size) each report a noisy position
estimate of the same true point, with covariance = `diag(sigma^2, sigma^2)`
- i.e. every sensor reports *only its own independent-noise variance*,
regardless of what's actually driving its error. Two noise-generation
conditions are compared:

- **independent** - each sensor's error is drawn independently; nothing
  shared.
- **positively_correlated** - every sensor's error additionally carries
  the *same* per-trial random bias (std 1.0/axis, drawn once and added to
  all four sensors identically) - e.g. a common atmospheric/multipath
  effect or a shared reference-frame error that pushes every UAV's radar
  the same way at once. Crucially, no sensor's reported covariance
  reflects this shared component - it has no way to measure a bias common
  to everyone, so "sensors incorrectly assuming independence" is baked
  into the input data itself, not an extra flag anywhere in the fusion
  code.

Each of the 5,000 trials per cell is fused both ways (`covariance_weighted_fusion`
and `covariance_intersection_fusion`), giving four cells that cover all
five comparison points from the task:

| Comparison point (task wording)     | Where it appears below |
|---|---|
| independent-sensor assumption       | the two "independent" rows |
| positively correlated sensor errors | the two "positively_correlated" rows |
| incorrectly assumed independence    | positively_correlated x covariance_weighted_fusion specifically - the mismatch case, since that's the mode that actually assumes independence when combining |
| Covariance Intersection             | both covariance_intersection_fusion rows |
| standard covariance-weighted fusion | both covariance_weighted_fusion rows |

**Metrics.** Normalized estimation error (NEES) is implemented directly:
`NEES = e^T P_fused^-1 e` where `e` is the fused-position error against
the known true point and `P_fused` is the fused 2x2 covariance. For a
consistent 2-D estimator, `E[NEES] = 2`; `consistency_ratio = mean_NEES / 2`
summarizes covariance consistency in one number (1.0 = consistent, >1 =
overconfident/too-tight covariance, <1 = conservative). "Overconfident
covariance count" = trials whose NEES exceeds the chi-square(2) 95%
upper critical value (5.991) - the reported covariance claims more
precision than the estimate actually has.

**Collision risk / mission success are proxies**, not a rerun of the
full multi-step `simple_swarm_sim.py` avoidance loop - that loop has no
way to inject a shared cross-UAV correlated bias without changing
production code, which is out of scope for this task (only this file was
requested). Instead, both proxies reuse this project's own decision
thresholds from `simulation_config.json["sensing"]`:
- **collision risk** = fraction of trials whose fused-position error
  exceeds `near_miss_distance - collision_distance` = 3.5 - 1.5 = **2.0**
  world units - large enough to plausibly flip whether a UAV judges a
  given separation as inside or outside the near-miss band.
- **mission success** = fraction of trials whose fused-position error
  stays within `goal_tolerance` = **2.0** world units - small enough that
  navigating on the fused estimate would still reach the target within
  tolerance.

## Results

| Condition | Fusion mode | Fused-pos. RMSE | Mean NEES | Consistency ratio | Overconfident count (rate) | Collision risk rate | Mission success rate |
|---|---|---|---|---|---|---|---|
| independent | covariance_weighted_fusion | 0.5166 | 2.0324 | 1.0162 | 249 (4.98%) | 0.0% | 100.0% |
| independent | covariance_intersection_fusion | 0.8453 | 1.9823 | 0.9912 | 243 (4.86%) | 0.4% | 99.6% |
| positively_correlated | covariance_weighted_fusion | 1.4809 | 16.7016 | 8.3508 | 3481 (69.62%) | 15.64% | 84.36% |
| positively_correlated | covariance_intersection_fusion | 1.6715 | 7.7510 | 3.8755 | 2321 (46.42%) | 24.08% | 75.92% |

(n = 5,000 trials/cell; full machine-readable numbers in
`results/fusion_consistency_results.json`.)

## Findings

**Independent-sensor assumption holds up as designed.** Under truly
independent errors, `covariance_weighted_fusion`'s mean NEES (2.03) sits
right at the theoretical value of 2.0 - consistency ratio 1.02 - and its
overconfident-covariance rate (4.98%) is almost exactly the ~5% a
correctly-calibrated estimator should show against a 95% threshold by
chance alone. CI is very slightly more conservative here (ratio 0.99,
essentially the same), at the cost of a higher RMSE (0.845 vs 0.517) -
CI's covariance-agnostic-to-correlation weighting is intentionally less
aggressive than the information filter's precision-maximizing weights,
so it gives up a little accuracy even when it didn't need to.

**Incorrectly assumed independence produces real overconfidence, not
just a theoretical worry.** With a shared bias present but invisible to
every sensor's own reported covariance, `covariance_weighted_fusion`'s
mean NEES jumps more than 8x (2.03 -> 16.70, consistency ratio 8.35), and
its overconfident-covariance rate goes from ~5% to nearly 70% of all
trials. The fused covariance is telling a UAV it knows the target's
position far better than it actually does.

**Covariance Intersection meaningfully reduces, but does not eliminate,
that overconfidence.** Under the same correlated-bias trials, CI's mean
NEES (7.75) is well under half of the information filter's (16.70), and
its overconfident rate (46.4%) is markedly lower (33% relative
reduction). CI is not magic, though: with the bias completely absent
from every input covariance, no fusion algorithm can fully correct for
it from the covariances alone, so CI's own consistency ratio (3.88) is
still above 1.0 - it narrows the miscalibration, it doesn't cure it.

**The shared bias itself moves the fused *position*, for both modes -
this is a real effect, not an artifact of the covariance being wrong.**
Averaging (or any convex-weighted combination) cannot cancel a bias
common to every source: fused-position RMSE rises from 0.517 to 1.481 for
covariance_weighted_fusion (2.9x) and from 0.845 to 1.671 for CI (2.0x)
under correlation. This is the part of the story that's about accuracy,
not just calibration - correlated sensor errors genuinely degrade the
estimate, they don't just make the fusion algorithm overconfident about
an otherwise-fine estimate.

**Collision risk and mission success both move in the expected
direction under correlation**, and the RMSE difference between the two
fusion modes shows up here too: covariance_weighted_fusion's slightly
lower RMSE under correlation gives it a lower collision-risk rate
(15.6% vs 24.1%) and higher mission-success rate (84.4% vs 75.9%) than
CI in this specific proxy setup - a reminder that CI's calibration
advantage doesn't automatically translate into better downstream
decisions when the underlying position error is what a decision
threshold actually reacts to; if the reported covariance itself feeds a
downstream decision (e.g. a stale-data or trust-based accept/reject
rule), CI's better-calibrated covariance would be the more relevant
number.

## Consistency checks

All 7 pass/fail checks in `fusion_consistency_validation.py` pass (see
`results/fusion_consistency_checks.md` for the full table):

| Task | Passed |
|---|---|
| independent_sensor_assumption | 2/2 |
| incorrectly_assumed_independence | 2/2 |
| covariance_intersection | 2/2 |
| standard_covariance_weighted_fusion | 1/1 |

## Limitations

- Collision risk / mission success are single-trial proxies against this
  project's own distance thresholds, not outcomes of the actual
  multi-step swarm-avoidance loop in `simple_swarm_sim.py` - that loop
  has no mechanism to inject a shared cross-UAV correlated bias without
  a code change.
- The correlated-bias model here is one specific case (a single shared
  Gaussian bias, identical across all four sensors, std 1.0/axis) - real
  correlated-error sources (e.g. a common atmospheric effect that varies
  with range, or partial rather than full correlation across UAVs) would
  shift the exact numbers, though the qualitative conclusion (information
  filter overconfident under correlation, CI better but not perfect)
  should hold generally - that's the entire reason CI exists.
- This analysis only covers `covariance_weighted_fusion` and
  `covariance_intersection_fusion` - the other fusion modes
  (`naive_fusion`, `confidence_weighted_fusion`, `trust_weighted_fusion`)
  don't claim covariance-based statistical optimality in the first place,
  so a NEES/consistency test isn't the right lens for them.
