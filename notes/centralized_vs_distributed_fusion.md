# Centralized vs. Distributed Fusion

`fusion_model.py` now supports two fusion **architectures** - independent
of which weighting scheme (`naive_fusion`, `trust_weighted_fusion`,
`covariance_intersection_fusion`, etc.) does the actual combining. The
architecture decides *where* fusion happens and *how tracks get there*;
the fusion mode still decides *how sources are weighted* once gathered.
Both are implemented in `fuse_step()` / `fuse_centralized()` /
`fuse_distributed()`.

## The two architectures

### Centralized

Every UAV's track is sent to one central fusion node (a ground station,
or a designated lead UAV). That node runs the same clustering + weighting
math the project already had, once, and produces a single final world
estimate per object, which is broadcast back out to the whole swarm.

- One shared answer - every UAV acts on the same fused belief.
- Costs an uplink message per reporting UAV plus one downlink broadcast.
- Nothing is usable until that full round trip completes.
- Single point of failure: if the central node or its link goes down,
  the whole swarm loses fusion at once.
- This reproduces the project's original (pre-Task 12) `fuse_step`
  behavior exactly, so it's the default architecture.

### Distributed

There is no central node. Each UAV keeps its own local track, broadcasts
a lightweight summary of it to the rest of the swarm, and separately
receives whatever summaries the others managed to get to it that step -
each peer-to-peer broadcast is an independent message that can be lost
(`COMM_DROP_PROBABILITY`). Each UAV then fuses its own track together
with whatever peer summaries actually arrived.

- No single point of failure - there's no central node to lose.
- Faster per-hop turnaround (one broadcast hop vs. a full uplink+downlink
  round trip), but no guarantee every UAV sees the same inputs.
- Because delivery isn't identical for every UAV, different UAVs can
  legitimately end up with *different* local estimates of the same
  object in the same step. That disagreement is itself something worth
  measuring, not just noise to average away.
- Costs more messages overall: every UAV broadcasts to every other UAV
  (`n * (n - 1)` attempted messages) vs. centralized's `n + 1`.

## What each metric measures, and where it comes from

| Metric | What it captures | Computable from `fusion_model.py` alone? |
|---|---|---|
| Estimation error | Distance between the fused estimate(s) and ground truth | Yes - `estimation_error_against_ground_truth()`, used strictly as an after-the-fact check (fusion itself never reads ground truth) |
| Communication load | How many messages the architecture needs per step | Yes - `comm_messages` / `comm_messages_delivered` on every fused row |
| Response time | Delay from detection to a usable fused estimate | Yes - `response_time_steps` on every fused row |
| Collision risk | How often a UAV's actual course brings it within near-miss distance of a threat it didn't act on in time | No - depends on the full control loop in `simple_swarm_sim.py` (`collision_risk_flag`) reacting to whichever fused estimate (shared or per-UAV) it's given |
| Mission success | Whether every UAV reaches its goal with zero collisions | No - `mission_completed_flag`, same reason |
| Formation error | RMSE of inter-UAV spacing vs. desired spacing | No - `avg_formation_error`, same reason |

The first three are properties of the fusion step itself and are
reported directly in every row `fuse_step`/`build_fused_log` produces.
The last three are properties of what the *swarm does* in response to
a fused estimate, so they only show up once each UAV's controller is
actually driven by that estimate - distributed's per-UAV estimates in
particular need `simple_swarm_sim.py`'s control loop to read a
`local_uav_id`-specific belief instead of one shared value. That wiring
is a follow-up (see "Next steps" below); this task adds the two
architectures and the metrics that can be measured without it.

## Worked example (synthetic, not a full sim run)

To make the message/latency/error trade-off concrete without needing
the rest of the pipeline, here's a small standalone example: 4 UAVs,
each with a noisy track of a real object sitting at ground truth
`(50.0, 50.0)`, fused with `trust_weighted_fusion` under both
architectures. (Full inputs/outputs: see the docstring examples in
`fusion_model.py`.)

**Centralized** (1 row, shared by the whole swarm):

| architecture | num_sources | comm_messages | response_time_steps | mean error |
|---|---|---|---|---|
| centralized | 4 | 5 | 3 | 0.552 |

**Distributed**, at three communication-drop levels (4 rows - one per UAV):

| comm_drop_probability | comm_messages (attempted) | comm_messages_delivered | response_time_steps | mean error | max error (worst UAV) |
|---|---|---|---|---|---|
| 0.0 (reliable mesh) | 12 | 12 | 2 | 0.552 | 0.552 |
| 0.4 (lossy) | 12 | 9 | 2 | 0.573 | 1.074 |
| 0.8 (very lossy) | 12 | 3 | 2 | 0.729 | 1.581 |

What this shows:

- **Communication load**: distributed always attempts more messages
  (`n*(n-1) = 12`) than centralized (`n+1 = 5`), regardless of whether
  those messages get through - that's the structural cost of a
  broadcast mesh vs. a hub.
- **Response time**: centralized pays a fixed round-trip cost (uplink +
  downlink = 2 steps here, on top of sensor latency); distributed pays
  only one hop (1 step on top of sensor latency), so it's consistently
  faster *when* an estimate is available at all.
- **Estimation error**: with a reliable mesh, distributed matches
  centralized's accuracy exactly (every UAV ends up with the same
  inputs, coincidentally). As the mesh gets lossier, the *mean* error
  degrades only a little, but the *max* error (the worst-off UAV, which
  is what actually matters for collision risk) grows much faster -
  degradation is uneven across the swarm, not just weaker error overall.
- Centralized has none of that spread by construction: every UAV shares
  the exact same (single) estimate, so there's no "worst UAV" - the
  failure mode instead is that *nobody* gets an estimate at all if the
  hub or its link fails, which this table doesn't capture since the
  model treats the central round trip as reliable today.

## Recommended trade-off summary

| | Centralized | Distributed |
|---|---|---|
| Communication load | Lower (`n+1`) | Higher (`n*(n-1)`) |
| Response time | Higher (full round trip) | Lower (one hop) |
| Estimation consistency | Perfectly consistent (one shared answer) | Can diverge across UAVs under lossy comms |
| Failure mode | Single point of failure (the hub) | Graceful degradation, no single point of failure |
| Best suited for | Small swarms, reliable comms to a hub, cases where consistent shared awareness matters most | Larger swarms, unreliable/partitioned comms, cases where reacting fast locally matters more than global agreement |

## How to run it

```
# Centralized (default) - same as before this task
python fusion_model.py --config simulation_config.json --architecture centralized --log logs/fused_centralized.csv

# Distributed
python fusion_model.py --config simulation_config.json --architecture distributed --seed 42 --log logs/fused_distributed.csv

# Both, in one CSV (adds an "architecture" column so they can be compared directly)
python fusion_model.py --config simulation_config.json --compare-architectures --seed 42 --log logs/fused_comparison.csv
```

Optional config overrides (add a top-level `"communication"` block to
`simulation_config.json`):

```json
"communication": {
  "central_uplink_latency_steps": 1,
  "central_downlink_latency_steps": 1,
  "distributed_hop_latency_steps": 1,
  "comm_drop_probability": 0.0
}
```

## Next steps

To get collision risk, mission success, and formation error into this
same comparison, `simple_swarm_sim.py`'s per-UAV control loop needs to
read its perceived-obstacle position from `build_fused_log`'s output for
its own `local_uav_id` (distributed) or the single shared row
(centralized), instead of always using its own local, unfused
perception. Once that's wired in, `run_experiments.py` /
`metrics_analysis.py` can be pointed at an `architecture` scenario axis
the same way they already handle `fusion_mode`, and the full six-metric
comparison table can be generated from real simulation runs instead of
the synthetic example above.
