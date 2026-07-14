# Radar Track Model Explanation

### Why tracking is needed on top of raw detections?
- A raw radar detection (from `radar_like_model.py`) is only valid for one time step: it can be a real hit, a missed detection, or clutter. `radar_track_model.py` turns this noisy, step-by-step stream into a *track* - a persistent belief about one object's position/velocity that survives occasional missed detections instead of disappearing every time the radar has a bad step.

### What a track is
- Each radar (one per UAV) keeps its own independent set of tracks. A track stores: `track_id`, estimated x/y, estimated vx/vy (once observed twice), a confidence value, `age`, `missed_count`, and a `status`.

### Track status states
- **tentative** - just created from an unmatched detection, not trusted yet.
- **confirmed** - matched 3 times in a row (`CONFIRM_HITS`), now considered reliable.
- **lost** - missed 3 times in a row (`MAX_MISSED`), the track is dropped after this final row.

### How detections are matched to tracks (nearest-neighbor)
- Each existing track predicts where it should be this step (coasting on its last known velocity). Every (track, detection) pair within `GATE_DISTANCE` (4.0 units) is a candidate match. Candidates are sorted by distance and claimed greedily, closest first, so each track and each detection is used at most once.

### What happens to a match
- The matched detection's x/y is blended into the track with exponential smoothing (`POSITION_ALPHA = 0.6`), and velocity is estimated from the position change and smoothed too (`VELOCITY_ALPHA = 0.5`). `missed_count` resets to 0 and `age`/hit streak go up.

### What happens on a miss
- An unmatched track is coasted forward using its last known velocity, `missed_count` increases, and once it hits `MAX_MISSED` the status becomes `lost`.

### What happens to an unmatched detection
- It spawns a brand-new `tentative` track.

### Why track loss matters for the swarm
- A track going `lost` means the UAV's radar has stopped believing an object is where it used to be - this is what drives `missed_response_flag` and, if it happens near the obstacle, collision risk. The `radar_track_loss` scenario in `simulation_config.json` (low P_D + high dropout) exists specifically to force this state so it can be observed, logged, and plotted (`track_loss_count` in `final_metrics_summary.csv`).

### Why this is simple nearest-neighbor and not a Kalman filter
- With one obstacle and a handful of radars, a full Kalman filter would add complexity without changing the qualitative results. Exponential smoothing + greedy nearest-neighbor association gives the same basic behavior (persistence across misses, smoothing of noisy positions) with far less code, which fits this project's scope.
