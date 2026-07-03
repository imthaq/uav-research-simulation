# Initial Results Summary

All 9 scenarios (`baseline`, `false\_positive`, `false\_negative`, `sensor\_noise`,
`latency`, `sensor\_dropout`, `confidence\_error`, `naive\_fusion`,
`trust\_weighted\_fusion`) run via `run\_experiments.py --runs 3` against the
default 4-UAV swarm. Numbers below are averages across the 3 runs per
scenario, taken from `results/results\_summary.csv`.

## Per-scenario averages (4 UAVs, 3 runs each)

|Scenario|Mission success|Avg collision-risk count|Avg missed response|Avg unnecessary avoidance|Avg fusion recovery|
|-|-|-|-|-|-|
|baseline|0/3|0.0|0.0|0.0|-|
|false\_positive|0/3|0.0|0.0|6.3|-|
|false\_negative|0/3|3.0|0.0|0.0|-|
|sensor\_noise|0/3|58.7|0.0|204.0|-|
|latency|0/3|0.0|0.0|0.0|-|
|sensor\_dropout|0/3|22.3|8.3|0.0|-|
|confidence\_error|0/3|0.0|0.0|0.0|-|
|naive\_fusion|0/3|0.0|0.0|79.0|68.0|
|trust\_weighted\_fusion|0/3|0.3|0.0|84.7|64.7|

## What scenarios were tested?

1. Baseline with no perception error
2. False positive scenario
3. False negative scenario
4. Sensor noise scenario
5. Latency scenario
6. Sensor dropout scenario
7. Confidence error scenario
8. Naive fusion scenario
9. Trust-weighted fusion scenario

## How many runs were performed?

27 total: 3 per scenario across 9 scenarios (`run\_experiments.py --runs 3`).
`metrics\_analysis.py` can also be run with a higher `--runs` for a larger
seeded sample (5 by default when run standalone).

## What metrics were collected?

Every run's CSV log (`logs/<scenario>\_run<n>.csv`) records, per UAV per
step: position, goal position, actual vs. perceived obstacle position,
perception error type, confidence value, fusion mode, action taken,
distance to nearest UAV, distance to obstacle, collision risk flag,
unnecessary avoidance flag, missed response flag, and mission completed
flag (full column list in `simulation\_readme.md`).

The aggregated `results\_summary.csv` (one row per scenario per run) records:
scenario, run\_number, fusion\_mode, false\_positive\_rate, false\_negative\_rate,
noise\_level, latency\_steps, dropout\_probability, confidence\_error\_level,
collision\_risk\_count, unnecessary\_avoidance\_count, missed\_response\_count,
fusion\_recovery\_count, mission\_success, avg\_response\_time\_s,
total\_near\_misses, avg\_formation\_error.

## What changed compared to baseline?

* Individual perception errors (false positives, false negatives, noise,
latency, dropout, confidence miscalibration) were each isolated one at a
time, so their effect could be compared directly against the same 4-UAV,
no-error baseline.
* Two additional scenarios (`naive\_fusion`, `trust\_weighted\_fusion`) apply
the *same* combined error profile (missed detections + position noise +
miscalibrated confidence) but let the swarm share and combine detections
across UAVs instead of each UAV relying only on its own sensor.

## Which perception error caused the most problem?

**Sensor noise** caused by far the most collision risk (avg 58.7 close
calls per run) and unnecessary avoidance (avg 204.0 per run) - noisy
position estimates repeatedly place the perceived obstacle in the wrong
spot, so UAVs both crowd too close to the real obstacle and steer away
from empty space. **Sensor dropout** was the next most disruptive: it
produced both collision risk (22.3) and the highest missed-response count
(8.3), since a UAV in a blackout has literally no way to react to a real
threat until the blackout ends.

## Does fusion help?

Fusion recovers a large share of individually-missed obstacle detections:
both `naive\_fusion` and `trust\_weighted\_fusion` ran under a harsher combined
error profile than any single-error scenario above (20% false-negative rate

* noise + confidence miscalibration) and still averaged 64-68 fusion-recovered
detections per run, driving missed\_response back down to 0.0 and collision
risk down to 0.0-0.3 - far better than what `sensor\_noise` or
`sensor\_dropout` alone produced with a lighter error profile. The trade-off
is more unnecessary avoidance (\~80 per run): sharing more (sometimes still
noisy) detections across the swarm means UAVs react to weaker/farther
signals more often, even when nothing is actually close. Trust-weighted
fusion did not clearly outperform naive fusion at these error levels; with
only 3 runs per scenario the difference is within noise, and a larger
`--runs` sample would be needed to say more.

## What needs improvement in the simulation?

* No scenario reaches `mission\_success` (all UAVs reach goal AND zero collisions) with the current 4-UAV, 100x100 world, and gain settings - UAVs consistently treat each other as obstacles near the goal formation and orbit rather than settle, which caps `uavs\_reached\_goal` well below 4/4 even in `baseline`. Retuning `goal\_gain` vs. `avoidance\_gain`, or widening `goal\_tolerance`, would likely let more of the swarm actually finish.
* Sensor dropout has no memory of the last known obstacle position once the blackout starts; carrying forward a decaying-confidence "last known" estimate (instead of nothing) during a blackout would likely reduce its missed-response count.
* With only 3 runs per scenario, results (especially the naive vs. trust-weighted fusion comparison) are noisy; `metrics\_analysis.py --runs 10` or higher would give a steadier read on which fusion strategy is actually better.

