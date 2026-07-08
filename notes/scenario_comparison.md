# Scenario Comparison

## Baseline vs False positive
* what changed:  Phantom (ghost) detections trigger the swarm into unnecessary avoidance maneuvers against obstacles that aren't there.
* which metric increased/decreased:  `unnecessary_avoidance_count` increased sharply (0 → ~114/run); collisions stayed at 0.
* why it happened:  Because of false positives the sensor reports a phantom obstacle, and the UAV steers to avoid it just like a real one.
* what this means for swarm safety: A) Not a safety risk the swarm still reaches its goals with zero collisions, just with wasted evasive maneuvers.

## Baseline vs False negative
* what changed: Missed real detections leave a UAV blind to a genuine threat right in front of it.
* which metric increased/decreased: `missed_response_count` increased (0 → 9/run) and `collision_count` increased (0 → 1.5/run).
* why it happened: 25% of real detections are dropped, so the UAV has nothing to react to even though the obstacle is really there.
* what this means for swarm safety: A real safety risk  unlike false positives, this scenario produces actual collisions.

## Baseline vs Latency
* what changed: Real detections still arrive, but delayed 5 steps before reaching the controller.
* which metric increased/decreased: `collision_count` increased (0 → 6/run) and `missed_response_count` increased (0 → 19/run).
* why it happened: By the time a delayed detection reaches the controller, the UAV is already too close to the obstacle to steer away in time.
* what this means for swarm safety: A meaningful safety risk stale information is nearly as dangerous as no information, since the reaction window has already closed.

## Baseline vs Dropout
* what changed: Periodic total sensor blackouts leave a UAV with no detections at all for several consecutive steps.
* which metric increased/decreased: `collision_count` increased the most of any single-error scenario (0 → 25.25/run), along with `missed_response_count` (0 → 80.5/run).
* why it happened: During a blackout the UAV receives zero detections real or phantom  so it can't perceive or respond to threats until the blackout ends.
* what this means for swarm safety: The most dangerous single-error scenario tested - extended blind spells give a UAV no way to avoid an obstacle it's flying straight toward.

## Baseline vs Noise
* what changed: Real detections arrive, but the obstacle's reported position is jittered away from its true location.
* which metric increased/decreased: `collision_count` increased (0 → 4.75/run) and `near_miss_count` increased (79 → 106.75/run).
* why it happened: The UAV steers relative to a noisy, wrong obstacle position, so its avoidance maneuver doesn't line up with where the obstacle actually is.
* what this means for swarm safety: A significant safety risk  the UAV is actively responding, but responding to the wrong location, which can steer it closer to danger instead of away from it.

## Naive fusion vs Trust-weighted fusion
* what changed: Combining detections across UAVs by simple averaging (naive) vs. weighting each UAV's detection by its self-reported confidence (trust-weighted).
* which metric increased/decreased: `collision_count` is far lower under naive fusion (0.4/run) than trust-weighted fusion (3.6/run).
* why it happened: Reported confidence is itself miscalibrated in this scenario, so trust-weighting ends up giving more influence to a confidently-wrong detection instead of damping it, while naive averaging is unaffected by confidence miscalibration entirely.
* what this means for swarm safety: Trust-weighted fusion isn't automatically safer than naive fusion - when the trust signal is unreliable, weighting by it can make collision outcomes worse rather than better.
