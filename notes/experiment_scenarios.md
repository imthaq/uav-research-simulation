# Experiment Scenarios

## 1\. Baseline (No Perception Error)

**What will be changed:** Nothing. All UAV sensors report ground truth exactly, with no delay, no noise, no dropout.

**Why it matters:** Establishes the reference behavior and reference metric values that all other scenarios are compared against.

**Input variable changed:** None (`false\_positive\_rate = 0.0`, `false\_negative\_rate = 0.0`, all other error parameters at 0/off).

**Expected behavior:** Smooth, direct path to goal, avoidance triggered only when truly needed, no unnecessary maneuvers, fastest response time, mission succeeds.

**Observed behavior from prototype:** Across all 5 runs: `collision\_risk\_count = 0`, `unnecessary\_avoidance\_count = 0`, `missed\_response\_count = 0`, `avg\_formation\_error ≈ 9.50` (identical every run, as expected with zero randomness). However, `mission\_success = No` in every run — even with perfect perception, the swarm never satisfies the "all UAVs reached goal AND zero collisions" condition within `max\_steps = 600`. Safety metrics are clean, but the reference case itself never registers a formal success.

**Metric affected:** All metrics (collision risk, unnecessary avoidance, missed response, mission success, response time, formation error).

**Notes for improvement:** The baseline should succeed by definition — if it doesn't, the failure is coming from somewhere other than perception error (likely `max\_steps` too low relative to formation-slot convergence time, or `goal\_tolerance`/`slot\_radius` too tight). This should be root-caused and fixed before trusting comparisons against the other scenarios, since right now every scenario is being compared to a baseline that itself "fails."

## 2\. False Positive Scenario

**What will be changed:** One or more UAVs report a detected obstacle/drone that does not actually exist, at a chosen point in the run.

**Why it matters:** Tests how the swarm handles phantom threats and whether false alarms cause wasted effort or unsafe maneuvers.

**Input variable changed:** `false\_positive\_rate`: 0.0 → 0.08.

**Expected behavior:** UAV(s) swerve or slow down unnecessarily, possible delay in reaching goal, possible disruption of formation.

**Observed behavior from prototype:** `unnecessary\_avoidance\_count` rose from 0 (baseline) to a range of 3–19 across the 5 runs (mean ≈ 9.8) — confirming phantom detections do trigger avoidance maneuvers. `collision\_risk\_count` stayed at 0, as expected (phantom obstacles don't cause real collisions). `avg\_formation\_error` dropped slightly (8.8–9.4 vs. baseline's 9.5), consistent with UAVs deviating from formation to dodge non-existent threats.

**Metric affected:** Unnecessary avoidance count, mission success, response time.

**Notes for improvement:** The 3–19 spread across identically-configured runs is wide relative to the mean, meaning single-run results would be misleading — this scenario needs more than 5 trials (or a confidence interval reported) before drawing conclusions. Worth also testing a confidence/trust threshold in the avoidance logic to see if it can filter out low-persistence phantom detections without suppressing real ones.

## 3\. False Negative Scenario

**What will be changed:** A real obstacle/drone is present but one or more UAVs fail to detect it.

**Why it matters:** Tests the worst-case safety risk - the swarm not reacting to a real hazard.

**Input variable changed:** `false\_negative\_rate`: 0.0 → 0.25.

**Expected behavior:** UAV continues on path toward the undetected obstacle, higher chance of collision or near-miss, mission may fail.

**Observed behavior from prototype:** `collision\_risk\_count` jumped to 0–13 across runs (mean ≈ 6.4), and `missed\_response\_count` was nonzero in 2 of 5 runs (up to 3). One run (run 3) showed zero risk at all, meaning a 25% miss rate doesn't guarantee a dangerous miss happens near an actual threat — it depends on whether the missed detections happen to coincide with a close encounter.

**Metric affected:** Collision risk / near-miss count, missed response count, mission success.

**Notes for improvement:** Because outcomes depend on *when* the miss occurs relative to proximity to the obstacle, purely random false negatives under-sample the worst case. Consider adding a variant that forces a missed detection specifically while a UAV is within `near\_miss\_distance`, to reliably stress-test this failure mode instead of relying on chance overlap.

## 4\. High-Confidence Wrong Detection

**What will be changed:** A false detection (obstacle/drone that doesn't exist) is reported with high confidence, so the UAV's decision logic treats it as certain rather than questionable.

**Why it matters:** Tests whether high confidence in incorrect data leads to worse decisions than a low-confidence false positive would, and whether trust-based logic can be fooled.

**Input variable changed:** Not yet defined in `simulation\_config.json` — would require a new confidence/certainty field per detection plus decision logic that reacts differently based on it.

**Expected behavior:** Strong, possibly extreme avoidance reaction (e.g. full stop or large detour) based on data that is wrong, larger deviation from planned path than a normal false positive.

**Observed behavior from prototype:** Not implemented — the current `Perception`/`\_steer` logic has no concept of detection confidence, so this scenario cannot be run yet.

**Metric affected:** Unnecessary avoidance count, response time, mission success.

**Notes for improvement:** To implement this, add a `confidence` field to generated phantom detections and a confidence-weighted term in `\_steer()`'s avoidance strength, then a new `scenarios.high\_confidence\_false\_positive` config entry. Compare its `unnecessary\_avoidance\_count`/deviation-from-path against scenario 2's results as the direct baseline for "does confidence make false positives worse."

## 5\. Sensor Dropout

**What will be changed:** Detection capability is turned off for one or more UAVs for a set period of time (no detections reported at all during that window, regardless of ground truth).

**Why it matters:** Tests swarm resilience when a UAV effectively goes "blind" temporarily, which is a realistic hardware/communication failure mode.

**Input variable changed:** `dropout\_prob`: 0.0 → 0.02 (per-step chance of entering an 8-step blackout).

**Expected behavior:** UAV continues moving without reacting to any obstacles during the dropout window, higher collision risk if a hazard appears during that time, possible recovery once dropout ends.

**Observed behavior from prototype:** Highly bimodal across runs: 2 of 5 runs had zero risk events (`collision\_risk\_count = 0`, `missed\_response\_count = 0`), while the other 3 had `collision\_risk\_count` of 11, 26, and 41 with `missed\_response\_count` up to 20. When response time was measurable, it ranged 0.5–1.2s (vs. baseline's undefined/instant reaction).

**Metric affected:** Collision risk / near-miss count, missed response count, mission success.

**Notes for improvement:** Outcome is dominated by whether a random dropout window happens to overlap with proximity to the obstacle — same issue as scenario 3. A deterministic variant (force dropout to start exactly when a UAV is near the obstacle) would give a more repeatable, worst-case-focused signal than the current fully-random timing.

## 6\. Delayed Detection (Latency)

**What will be changed:** Detections are still reported correctly but with a fixed delay (e.g. N time steps after the true event).

**Why it matters:** Tests how much lag the swarm's avoidance logic can tolerate before reaction becomes too late to be effective.

**Input variable changed:** `latency\_steps`: 0 → 5.

**Expected behavior:** UAV reacts later than in baseline, avoidance maneuver may be more abrupt or insufficient, higher near-miss count as delay increases.

**Observed behavior from prototype:** Unexpectedly flat: `collision\_risk\_count = 0` and `unnecessary\_avoidance\_count = 0` in all 5 runs, identical to each other (no randomness in this scenario at all — `false\_positive\_rate`/`false\_negative\_rate` are both 0, so only the delay is active). `avg\_response\_time\_s` was undefined in every run because no UAV ever came within `near\_miss\_distance` of a threat, so the "response time" metric never had an event to measure. Notably, `avg\_formation\_error` was the highest of any scenario (10.70 vs. baseline's 9.50), suggesting the 5-step delay does affect formation-keeping even though it never shows up in the collision/response-time metrics.

**Metric affected:** Response time, collision risk / near-miss count, mission success.

**Notes for improvement:** This scenario currently can't demonstrate the thing it's meant to test — with these start positions and obstacle placement, UAVs never get close enough to the obstacle for a 5-step delay to matter. Either move the obstacle into the direct flight path, tighten `near\_miss\_distance`, or increase `latency\_steps` further so the delay's effect becomes visible in the collision/response-time metrics rather than only in formation error.

## 7\. Noisy Sensor Readings

**What will be changed:** Detected position/distance values are offset from the true values by random noise (magnitude varied across runs).

**Why it matters:** Tests robustness of avoidance logic to imprecise, realistic sensor data rather than perfect measurements.

**Input variable changed:** `position\_noise\_std`: 0.0 → 1.5.

**Expected behavior:** Avoidance maneuvers become less precise, UAV may steer around empty space or misjudge clearance, formation spacing becomes less consistent as noise increases.

**Observed behavior from prototype:** By far the worst scenario measured: `collision\_risk\_count` ranged 37–79 (mean ≈ 56) and `unnecessary\_avoidance\_count` ranged 149–221 (mean ≈ 187) — both far above every other scenario, including false\_positive and false\_negative individually. `avg\_formation\_error` was slightly better than false\_positive/false\_negative (8.0–8.7), but the swarm is simultaneously over-reacting (unnecessary avoidance) and under-protected (collision risk), which is the worst combination of the two failure modes tested so far.

**Metric affected:** Collision risk / near-miss count, unnecessary avoidance count, formation error.

**Notes for improvement:** `position\_noise\_std = 1.5` is only a single noise level — the scenario's stated purpose ("magnitude varied across runs") isn't actually being tested yet. Add a sweep (e.g. 0.5, 1.0, 1.5, 2.5) as separate config entries or a parameterized run, to find where the degradation curve is and whether there's a noise level the current avoidance logic can still tolerate safely.

## 8\. Naive Fusion

**What will be changed:** When multiple UAVs share detection data, all reports are combined with equal weight regardless of source reliability (simple averaging or "any UAV reports it, so it's treated as true").

**Why it matters:** Establishes a baseline for how a simple, unweighted data-sharing approach performs when some UAVs have degraded perception (noise, dropout, false positives, etc.).

**Input variable changed:** Not yet defined — the current prototype has no inter-UAV detection sharing at all; each UAV only acts on its own sensor readings.

**Expected behavior:** Swarm is easily misled by a single faulty UAV's bad data; one noisy or false-positive-prone UAV can trigger unnecessary avoidance across the group, or a false negative can be masked improperly if fusion logic assumes all inputs are equally trustworthy.

**Observed behavior from prototype:** Not implemented — no fusion layer exists yet in `simple\_swarm\_sim.py`.

**Metric affected:** Unnecessary avoidance count, missed response count, mission success.

**Notes for improvement:** Requires adding a shared-detection broadcast step before `\_steer()`, where each UAV's perceived detections are pooled with the swarm's and treated as equally trustworthy. Should reuse the existing per-UAV `Perception` error injection (noise/dropout/false-positive) so the fusion layer can be tested under the same fault conditions as scenarios 2/3/5/7.

## 9\. Trust-Weighted Fusion

**What will be changed:** Shared detection data is combined using per-UAV trust/reliability weighting (e.g. a UAV known to have noisy or degraded sensors contributes less to the fused decision).

**Why it matters:** Tests whether weighting detections by source reliability reduces the negative impact of a single faulty UAV compared to naive fusion.

**Input variable changed:** Not yet defined — depends on scenario 8's fusion layer existing first, plus a trust/weight parameter per UAV.

**Expected behavior:** Swarm is more resistant to a single faulty UAV's bad data; fewer unnecessary avoidance events and fewer missed responses compared to naive fusion under the same injected errors.

**Observed behavior from prototype:** Not implemented — depends on scenario 8 being built first.

**Metric affected:** Unnecessary avoidance count, missed response count, mission success, collision risk / near-miss count.

**Notes for improvement:** Build directly on top of scenario 8's fusion layer rather than as a separate mechanism, so the two can be run under identical injected-error conditions and compared head-to-head (naive vs. trust-weighted) on the same metrics — that comparison is the actual point of this scenario, not the trust-weighted result in isolation.

