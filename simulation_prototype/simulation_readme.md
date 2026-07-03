# UAV Swarm Perception-Error Simulation

## What this is

A 2D simulation of a UAV swarm flying from start positions to a shared target while avoiding one obstacle and each other. Each UAV's perception of the world can be corrupted (false positives, false negatives, position noise, latency, sensor dropout, miscalibrated confidence) to study how perception errors - and cross-UAV sensor fusion that tries to compensate for them - affect swarm safety and mission success.

## Folder layout

```
simulation\\\_prototype/
  simple\\\_swarm\\\_sim.py        - core simulation (world, perception, fusion, control, logging)
  simulation\\\_config.json     - world, swarm, sensing, control, and scenario settings
  run\\\_experiments.py         - runs every scenario for N repeated trials, saves per-run CSVs
  metrics\\\_analysis.py        - aggregates runs into results\\\_summary.csv, prints summary stats
  generate\\\_plots.py          - builds PNG charts from results\\\_summary.csv
  initial\\\_results\\\_summary.md - narrative summary of findings
  logs/                      - one CSV per scenario per run, e.g. baseline\\\_run1.csv
  results/                   - results\\\_summary.csv (canonical aggregated output)
  plots/                     - PNG charts generated from results\\\_summary.csv
```

## How to run

Run everything (all scenarios x 3 repeated trials, one CSV per run, aggregated summary):

&#x20;   python run\_experiments.py --runs 3



Run just the simulation once per scenario (writes logs/simulation\_log.csv):

&#x20;   python simple\_swarm\_sim.py --config simulation\_config.json --log logs/simulation\_log.csv



Optional: `--scenario <name>` to run only one scenario instead of all.

Aggregate metrics across seeded runs on their own (writes results/results\_summary.csv and a root-level copy):

&#x20;   python metrics\_analysis.py --runs 5



Generate charts from the aggregated summary:

&#x20;   python generate\_plots.py



## World setup

* 100 x 100 world (`world.width` / `world.height`)
* Shared target/goal at (90, 90) (`world.target`)
* One circular obstacle at (50, 50), radius 5 (`world.obstacle`)
* 4 UAVs starting near (5-15, 5-15) (`swarm.num\\\_uavs`, `swarm.start\\\_positions`)
* Each UAV gets its own goal point arranged in a small circle around the
shared target (not the exact same point), so they don't all try to
crowd into one spot at the end

## Configurable parameters (simulation\_config.json)

`swarm`: number of UAVs, start positions, UAV speed, formation spacing, safety distance
`world`: area size, goal/target position, obstacle position
`sim`: time step, max steps, simulation duration (seconds), random seed
`sensing`: sensor range, collision distance, near-miss distance, goal tolerance
`control`: goal-seeking gain, avoidance gain, tangential (slide-around) gain
`perception\\\_errors` (also overridable per scenario):

* `false\\\_positive\\\_rate` - chance of a fake ("phantom") detection each step
* `false\\\_negative\\\_rate` - chance a real detection is missed
* `position\\\_noise\\\_std` - Gaussian noise added to detected x/y position
* `latency\\\_steps` - delay between when a detection happens and when the controller receives it
* `dropout\\\_prob` / `dropout\\\_duration\\\_steps` - chance of a temporary sensor blackout (no detections at all) lasting several steps
* `confidence\\\_error\\\_level` - how far a detection's *reported* confidence can drift from its true confidence (miscalibration)
* `fusion\\\_mode` - `no\\\_fusion`, `naive\\\_fusion`, or `trust\\\_weighted\\\_fusion`

## Perception model

Each UAV senses obstacles and other UAVs within `sensor\\\_range`. What it
actually perceives (vs. ground truth) can be corrupted per scenario using
the parameters above. Every real or phantom detection also carries a
**confidence value** (0-1): higher for close, clean, real detections;
moderate for phantoms (a false detection typically still looks clean to the
sensor that produced it, which is what makes it dangerous to trust);
optionally miscalibrated by `confidence\\\_error\\\_level`.

## Sensor fusion model

When more than one UAV currently has a real (non-phantom) detection of the
obstacle, the fusion step can combine those independent detections into one
shared belief instead of each UAV using only its own noisy view:

* **no\_fusion** - each UAV uses only its own perception (default/legacy behavior).
* **naive\_fusion** - unweighted average of every contributing UAV's detected obstacle position.
* **trust\_weighted\_fusion** - same average, but weighted by each detection's confidence value, so more-trusted detections count for more.

Fusion can *recover* a detection an individual UAV's own sensor missed
(false negative or dropout) as long as at least one other UAV still saw it
and the fused position is within the recovering UAV's own sensor range -
this is logged per run as `fusion\\\_recovery\\\_count`. Phantom (false-positive)
detections are never fused, since each is a distinct, uncorroborated ghost
with nothing to average against.

## Control model

Simple potential-field steering:

* Goal-seeking vector toward the UAV's target
* Repulsive vector away from each perceived threat (obstacle or other UAV, after fusion where applicable), strength \~ avoidance\_gain / distance
* Tangential vector (perpendicular to repulsion) so the UAV slides around a threat instead of stalling head-on, strength \~ tangential\_gain / distance

Combined vector is normalized and scaled by uav\_speed.

## Scenarios (in simulation\_config.json)

Each scenario runs a minimum of 3 repeated trials with different seeds:

1. `baseline` - no perception error, no fusion
2. `false\\\_positive` - phantom detections only
3. `false\\\_negative` - missed detections only
4. `sensor\\\_noise` - noisy detected positions
5. `latency` - delayed detections
6. `sensor\\\_dropout` - periodic sensor blackouts
7. `confidence\\\_error` - miscalibrated confidence values, no fusion to compensate
8. `naive\\\_fusion` - moderate noise + missed detections, fused by unweighted averaging
9. `trust\\\_weighted\\\_fusion` - same error conditions as naive\_fusion, fused by confidence-weighted averaging

## Log output (one row per UAV per step)

Every run CSV (`logs/<scenario>\\\_run<n>.csv`, `logs/simulation\\\_log.csv`) includes:

* `scenario`, `step`, `time\\\_s`, `uav\\\_id`
* `uav\\\_pos\\\_x`, `uav\\\_pos\\\_y` - UAV position
* `goal\\\_pos\\\_x`, `goal\\\_pos\\\_y` - this UAV's goal position
* `actual\\\_obstacle\\\_x`, `actual\\\_obstacle\\\_y` - ground-truth obstacle position
* `perceived\\\_obstacle\\\_x`, `perceived\\\_obstacle\\\_y` - what the UAV (post-fusion, post-latency) believes the obstacle position is; blank if not currently detected
* `perception\\\_error\\\_type` - which corruption mechanism(s) fired this step (`none`, or a `+`-joined combo of `dropout`/`false\\\_positive`/`false\\\_negative`/`position\\\_noise`/`latency`/`confidence\\\_error`)
* `confidence\\\_value` - confidence attached to the perceived obstacle detection (blank if none)
* `fusion\\\_mode` - the fusion mode active for this run
* `action\\\_taken` - one of: `move`, `avoidance`, `false\\\_avoidance`, `at\\\_goal`
* `num\\\_perceived\\\_detections`, `num\\\_phantom\\\_detections`
* `dist\\\_to\\\_goal`
* `distance\\\_to\\\_nearest\\\_uav` - distance to the closest other UAV
* `distance\\\_to\\\_obstacle` - distance to the obstacle surface
* `nearest\\\_entity\\\_type`, `nearest\\\_entity\\\_distance` - closest entity of either kind (obstacle or UAV) and its distance
* `collision\\\_risk\\\_flag` - nearest entity within near\_miss\_distance
* `unnecessary\\\_avoidance\\\_flag` - avoidance triggered by a phantom only
* `missed\\\_response\\\_flag` - a real nearby threat that was not perceived
* `mission\\\_completed\\\_flag` - all UAVs reached goal AND zero collisions, as of this step
* `reached\\\_goal` - whether this UAV has reached its own goal slot yet

## Aggregated metrics (results/results\_summary.csv, results\_summary.csv)

One row per scenario per run:

* scenario, run\_number, fusion\_mode
* false\_positive\_rate, false\_negative\_rate, noise\_level, latency\_steps, dropout\_probability, confidence\_error\_level
* collision\_risk\_count, unnecessary\_avoidance\_count, missed\_response\_count, fusion\_recovery\_count
* mission\_success, avg\_response\_time\_s, total\_near\_misses, avg\_formation\_error

## Metrics (printed to stdout per scenario)

* steps\_run
* uavs\_reached\_goal / num\_uavs
* mission\_success (all UAVs reached goal AND zero collisions)
* collision\_count
* near\_miss\_count
* unnecessary\_avoidance\_count - avoidance triggered by a phantom only
* missed\_response\_count - a real nearby threat that was not perceived
* avoidance\_action\_count - avoidance triggered by a real detection
* fusion\_recovery\_count - steps where fusion supplied an obstacle detection a UAV had individually missed
* avg\_response\_time\_s - average delay between a threat becoming real (within near\_miss\_distance) and the UAV actually perceiving it
* avg\_formation\_error - RMSE of inter-UAV distance vs. desired\_formation\_spacing

