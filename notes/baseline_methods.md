# Baseline Methods Comparison

This document compares the baseline perception/tracking/fusion methods used
in the UAV swarm simulation and explains why each is included as a
reference point.

## 1. Perfect Ground-Truth Perception
Uses the true obstacle/target position directly, with no sensor noise,
dropout, or false alarms. Represents the theoretical upper bound on
performance. Included to isolate perception error from planning/control
error any gap between this and other methods is attributable purely to
sensing imperfection.

## 2. Radar-Only Detection
Uses raw radar detections each step with no temporal filtering a
detection either exists this step or it doesn't. Captures the effect of
range/FOV limits, missed detections, and false alarms/clutter in
isolation. Included as the simplest real-sensor baseline, before any
smoothing or estimation is applied.

## 3. Radar-Only Tracking
Adds a Kalman filter on top of radar detections, giving predicted
("coasting") vs. filtered position estimates, track status, and
covariance. Included to show the value of temporal filtering alone
the improvement over Detection-only isolates what tracking (not fusion)
buys you.

## 4. Naive Multimodal Fusion
Combines radar, vision, and LiDAR estimates with simple averaging,
ignoring per-sensor confidence or reliability. Included as the simplest
multi-sensor baseline, showing the raw benefit of adding sensors before
any weighting scheme is applied  and as a foil for smarter fusion
methods below.

## 5. Trust-Weighted Fusion
Weights each sensor's contribution by a scalar "trust"/reliability score
(e.g. lower trust for a known-noisy or degraded sensor). Included to
show the benefit of accounting for sensor quality, and to test
robustness when one sensor is unreliable or faulty (e.g. the
overconfident-faulty-sensor scenario).

## 6. Covariance-Weighted Fusion
Weights each sensor's contribution by its full covariance matrix rather
than a single scalar, giving anisotropic (direction-dependent)
confidence. Included to test whether directional uncertainty
information improves fusion accuracy beyond a single trust scalar,
especially for sensors like LiDAR with non-isotropic error.

## 7. Dynamic Trust Fusion
Trust/reliability scores are updated online based on observed sensor
performance (e.g. residual error, staleness) rather than fixed a priori.
Included to test adaptability — how well fusion recovers when a sensor's
reliability changes mid-run, such as degrading or recovering from
dropout.

## 8. Centralized Fusion
All UAVs' sensor data is combined into one shared fused estimate,
computed at a single point. Included as the accuracy upper bound for
multi-UAV fusion no communication loss, no per-UAV divergence — and as
the comparison target for the distributed architecture below.

## 9. Distributed Fusion
Each UAV computes its own local fused estimate from whatever peer
broadcasts it actually receives, subject to communication dropout.
Included to measure the real-world cost of decentralization how much
accuracy is lost when communication is imperfect, and how gracefully the
swarm degrades under packet loss (e.g. the communication-outage
scenario).
