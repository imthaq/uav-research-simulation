# UAV Swarm Perception-Error Simulation

## What this is

A 2D simulation of a UAV swarm flying from start positions to a shared target while avoiding one obstacle and each other. Each UAV's perception of the world can be corrupted (false positives, false negatives, position noise, latency, sensor dropout) to study how perception errors affect swarm behavior.

## Files

* simple\_swarm\_sim.py - the simulation code
* simulation\_config.json - world, swarm, sensing, control, and scenario settings
* simulation\_log.csv - per-step, per-UAV log output from a run
* initial\_results\_summary.md - summary of execution results

## How to run

python simple\_swarm\_sim.py --config simulation\_config.json --log simulation\_log.csv

Optional: --scenario <name> to run only one scenario instead of all of them.

## World setup

* 100 x 100 world
* Shared target at (90, 90)
* One circular obstacle at (50, 50), radius 5
* 4 UAVs starting near (5-15, 5-15)
* Each UAV gets its own goal point arranged in a small circle around the
shared target (not the exact same point), so they don't all try to
crowd into one spot at the end

## Perception model

Each UAV senses obstacles and other UAVs within sensor\_range. What it
actually perceives (vs. ground truth) can be corrupted per scenario:

* false\_positive\_rate - chance of a fake ("phantom") detection each step
* false\_negative\_rate - chance a real detection is missed
* position\_noise\_std - Gaussian noise added to detected x/y position
* latency\_steps - delay between when a detection happens and when the
controller receives it
* dropout\_prob / dropout\_duration\_steps - chance of a temporary sensor
blackout (no detections at all) lasting several steps

## Control model

Simple potential-field steering:

* Goal-seeking vector toward the UAV's target
* Repulsive vector away from each perceived threat (obstacle or other UAV),
strength \~ avoidance\_gain / distance
* Tangential vector (perpendicular to repulsion) so the UAV slides around
a threat instead of stalling head-on, strength \~ tangential\_gain / distance

Combined vector is normalized and scaled by uav\_speed.

## Scenarios (in simulation\_config.json)

* baseline - no perception errors
* false\_positive - phantom detections only
* false\_negative - missed detections only
* sensor\_noise - noisy detected positions
* latency - delayed detections
* sensor\_dropout - periodic sensor blackouts

## Log output (simulation\_log.csv)

One row per UAV per step:

* scenario, step, time\_s, uav\_id, x, y
* num\_perceived\_detections, num\_phantom\_detections
* dist\_to\_target, reached\_goal
* event - one of: move, avoidance, false\_avoidance, at\_goal

## Metrics (printed to stdout per scenario)

* steps\_run
* uavs\_reached\_goal / num\_uavs
* mission\_success (all UAVs reached goal AND zero collisions)
* collision\_count
* near\_miss\_count
* unnecessary\_avoidance\_count - avoidance triggered by a phantom only
* missed\_response\_count - a real nearby threat that was not perceived
* avoidance\_action\_count - avoidance triggered by a real detection
* avg\_response\_time\_s - average delay between a threat becoming real
(within near\_miss\_distance) and the UAV actually perceiving it
* avg\_formation\_error - RMSE of inter-UAV distance vs. desired\_formation\_spacing

