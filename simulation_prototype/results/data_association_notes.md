# Data Association Notes

## Why gating is used

Every simulation step produces a mix of real detections and Poisson-distributed
clutter. Without gating, the tracker would be free to associate a track with
*any* detection in the scan, including clutter points that happen to be
closest by raw distance even when they are nowhere near where the track's own
motion model predicts it should be. Gating restricts candidate matches to a
statistically plausible neighborhood around each track's predicted position,
so clutter and unrelated detections are rejected before association is even
attempted. This keeps the tracker's Kalman update from being corrupted by
implausible measurements and keeps track continuity stable in cluttered or
noisy scans.

## How false alarms affect tracking

False alarms (clutter or spurious detections) that fall inside a track's gate
are the main failure mode — they can still be picked up if no real detection
is closer, causing the Kalman filter to update on a phantom measurement and
briefly pull the track off its true state. False alarms that fall outside
every existing track's gate are handled differently: they spawn new
*tentative* tracks. Most of these tentative tracks die quickly, since a
single clutter point rarely repeats in roughly the same place on consecutive
scans, so it fails to accumulate the consecutive hits needed to reach
`CONFIRMED` status and instead ages out through missed counts. Higher clutter
density increases both the rate of tentative-track churn and, less often, the
chance of a real track briefly latching onto a false alarm.

## How nearest-neighbor association works

At each step, every existing track predicts its next position from its own
Kalman motion model. For every (track, detection) pair, the Mahalanobis
distance-squared between the predicted track position and the detection is
computed using the track's innovation covariance. Any pair whose distance
exceeds `GATE_CHI2 = 9.21` (a chi-squared gate at roughly a 99% confidence
region for 2 degrees of freedom) is rejected outright. Among the remaining
in-gate candidates, association is resolved greedily: pairs are sorted by
distance and claimed nearest-first, so each track matches at most one
detection and each detection matches at most one track per step. Matched
tracks run a full Kalman update, reset `missed_count` to zero, and increase
`existence_probability`. A track only flips from `tentative` to `confirmed`
after `CONFIRM_HITS = 3` consecutive matched hits. Unmatched tracks are left
at their predicted (coasted) state, `missed_count` increments, and
`existence_probability` decays; a track is marked `lost` once `missed_count`
reaches the configured ceiling or `existence_probability` collapses below the
deletion floor, and is removed the following step. Unassociated detections
that survive gating against all existing tracks spawn new `tentative` tracks.

## Current limitations

The association method is a greedy nearest-neighbor scheme, not a globally
optimal assignment — in dense or closely-spaced-target scenarios it can make
a locally reasonable but globally suboptimal match, occasionally causing two
nearby tracks to compete for the same detection in a way that isn't resolved
optimally. It also only ever considers a single hypothesis per detection per
step, so it cannot represent or carry forward ambiguity when two tracks are
both plausible matches for the same detection. Clutter is treated purely as
noise to be gated out or spawn short-lived tracks; there is no explicit
clutter-density term feeding back into the gate size itself.

## Possible future JPDA/RFS upgrade

A Joint Probabilistic Data Association (JPDA) filter would let a detection
update multiple in-gate tracks in proportion to their association
probabilities instead of forcing a single greedy winner-take-all match, which
should help specifically in the crossing-target and closely-spaced-target
scenarios where nearest-neighbor is weakest. A Random Finite Set (RFS)
approach (e.g., PHD or CPHD filter) would go further by modeling the entire
multi-target state as a single set-valued distribution, natively handling
clutter, missed detections, and an unknown, time-varying number of targets
without needing explicit track initiation/deletion heuristics. Either would
be a heavier computational and implementation lift than the current gated
nearest-neighbor tracker and is flagged as a planned upgrade rather than
something attempted in this milestone.
