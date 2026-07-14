# Radar Results Discussion

All numbers below are averages over 3 seeded runs per scenario, from `logs/final_metrics_summary.csv` (Task 15/16 pipeline). Each sweep isolates one radar parameter while the rest stay at baseline defaults (see `simulation_config.json`).

### P_D (detection probability): 1.0 → 0.5
- Missed response count climbs from 0 (P_D=1.0) to ~5.7 (P_D=0.9), ~15.3 (P_D=0.7), and ~25.7 (P_D=0.5) - a clear, roughly linear relationship between lower detection probability and more missed responses.
- Track loss count follows the same pattern (4 → 4.7 → 10 → 28.7): as P_D drops, radars go more consecutive steps without a hit, so more tracks age past `MAX_MISSED` and are dropped.
- `wrong_radar_decision_count` tracks `missed_response_count` almost exactly here, since with P_FA=0 the only source of wrong decisions in this sweep is missed detections.
- Formation error and fusion error both drift up slightly as P_D drops, consistent with the swarm coordinating on a shakier, more intermittent radar picture.

### P_FA (false alarm probability) and clutter density
- Across the full P_FA sweep (0.0 → 0.3) and clutter density sweep (0 → 5), `unnecessary_avoidance_count` and `wrong_radar_decision_count` stayed at 0 in these runs. This is an honest limitation of the current model, not evidence that false alarms are harmless: `radar_like_model.py` places clutter/false-alarm points at a uniformly random range/bearing anywhere inside the radar's sensing area, so on a 4-UAV/1-obstacle scenario most fake detections land far from the swarm's actual flight path and never get close enough to trigger an avoidance maneuver. A future refinement would bias clutter generation toward the swarm's vicinity (or lower `GATE_DISTANCE`/avoidance-trigger radius) to make this effect observable.
- Clutter density did have a small, non-monotonic effect on formation error (3.77 → 3.69 → 3.81 → 4.02 from 0 to 5), and a mild downward trend in fusion error - plausible instability at this scale of noise, likely reflecting run-to-run seed variance rather than a strong causal effect at only 3 trials/setting.

### Range, bearing, and radial-velocity noise
- Range noise had little visible effect on `avg_formation_error` at these noise levels (low/med/high all ≈3.7-3.8), since the range-noise magnitudes tested are still small relative to the swarm's spacing (8 units).
- Bearing noise showed the clearest, most direct effect of any noise sweep: `avg_bearing_error` scaled almost exactly with the configured noise std (0.014 → 0.029 → 0.071 rad for low/med/high), confirming the bearing-noise injection in `radar_like_model.py` is working as intended.
- Radial-velocity noise showed no measurable effect on `avg_response_time_s` in these scenarios, because the current UAV decision logic does not use radial velocity for avoidance/response timing - only detected x/y position drives decisions (see `radar_model_explanation.md`). This is a known limitation to note for future work if radial velocity is to influence decisions directly.

### Radar latency: 0 → 5 steps
- `avg_response_time_s` grows monotonically with latency (0 → 0.2 → 0.6 → 1.0 s for 0/1/3/5 steps), exactly matching the expectation that delayed detections delay the UAV's reaction.
- `avg_range_error` and `avg_bearing_error` also grow with latency (0.239→0.288→0.499→0.799 for range error; 0.029→0.034→0.072→0.119 rad for bearing error). This is expected: a stale, latency-delayed measurement is being compared against the *current* true position, so the apparent "error" partly reflects how far the target/UAV moved during the delay, not sensor noise itself.

### Radar dropout: 0.0 → 0.3
- Missed response count rises steadily (0 → 4.7 → 9.3 → 16) as dropout probability increases, and track loss count rises alongside it (4 → 4 → 6 → 10.7) - consistent with dropout acting like a temporary, total P_D=0 blackout.
- Mission success stayed at 100% for all dropout levels tested here; the swarm's control logic is robust enough to coast through these blackout durations (`dropout_duration_steps`) without missing the goal outright, though `unnecessary_avoidance`/`missed_response` risk clearly increases.

### Fusion mode: no_fusion vs naive_fusion vs trust_weighted_fusion
- Both fusion modes reduce missed response versus no fusion (12.7 → 10.7 for both naive and trust-weighted), confirming that combining multiple UAVs' radar tracks recovers detections that a single UAV's radar missed on its own.
- The clearest difference between the two fusion strategies shows up in `fusion_error`, the average distance between the fused estimate and the true object position: naive_fusion averages 0.277, trust_weighted_fusion averages 0.292 in these matched-condition runs. At this noise level the two are close, with naive fusion slightly ahead - trust-weighting only pays off clearly when the *contributing* tracks differ meaningfully in reliability (e.g. one UAV has much worse noise/dropout than another); in a symmetric 4-UAV swarm with the same error profile applied to everyone, weighting by confidence has less to differentiate.
- Track loss count is lowest under naive_fusion (5.0) versus no_fusion_matched (6.3) and trust_weighted_fusion (6.0), suggesting fusion also has a mild stabilizing effect on individual track continuity by filling in gaps other UAVs still see.

### Track loss scenario (`radar_track_loss`)
- With P_D=0.3 and radar dropout=0.4 combined, the swarm sees dramatically worse outcomes across the board: missed response jumps to ~62.7, collision risk count to ~33.3 (versus 0 everywhere else), track loss to ~45.7, and mission success drops to 0%. This scenario exists specifically to demonstrate what happens once radar quality degrades enough that tracks are lost outright, and is the basis for the required track-loss visualizer video/plot evidence (Task 13).

### Overall takeaway
- The radar-like pipeline reproduces the expected qualitative relationships from the research direction (P_D/dropout/latency all directly degrade timeliness and reliability of UAV decisions; fusion recovers some of that loss). The clearest gap for future work is making false-alarm/clutter placement spatially relevant to the swarm's path, so P_FA/clutter's effect on `unnecessary_avoidance_count` becomes visible the same way P_D's effect on missed response already is.
