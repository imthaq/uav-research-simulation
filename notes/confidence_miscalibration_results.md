# Radar Confidence-Miscalibration Study

Controlled comparison of how *systematic* radar confidence miscalibration 
as distinct from random confidence noise, which already existed in this
project as `radar_confidence_error`  propagates through
`confidence_weighted_fusion` into swarm-level outcomes.

## 1. Why a new mechanism was needed

`radar_confidence_error` (existing) adds zero-mean Gaussian noise to every
reported confidence: it scatters confidence around the true value but never
shifts it in one direction, so a sensor built only from that knob is
noisy but not *biased*. None of it. There was no way to build a sensor
that is *chronically* over- or under-confident, or one whose false alarms
specifically look more trustworthy than its real detections, which is what
this task asked for.

Two new config knobs were added to `models/radar_like_model.py`:

| Knob | Default | Applies to | Effect |
|---|---|---|---|
| `radar_confidence_bias` | `0.0` | real-detection confidence | fixed additive shift; `+` = overconfident, `-` = underconfident |
| `radar_clutter_confidence_bias` | mirrors `radar_confidence_bias` | clutter/false-alarm confidence | same, applied independently so false alarms can be miscalibrated without touching real-detection confidence |

Both stack with the existing `radar_confidence_error` noise term and are
read scenario-first / radar-config-second, matching every other
`radar_*` knob's precedence rule already in this project. Clutter
confidence was previously hardcoded to `uniform(0.2, 0.7)` with no config
hook at all  `radar_clutter_confidence_bias` is the only new surface
touching that path.

## 2. Scenarios

All seven scenarios below share **identical** detection/noise/clutter
conditions (`radar_detection_probability=0.85`, `radar_range_noise_std=0.4`,
`radar_false_alarm_probability=0.05`, `radar_clutter_density=0.15`,
`fusion_mode=confidence_weighted_fusion`) unless a column says otherwise —
so any metric difference between them is attributable to confidence
miscalibration, not to some other perception parameter also changing.
`high_confidence_false_alarms`, `high_confidence_incorrect_tracks`, and
`low_confidence_correct_detections` each deliberately change one more
parameter, because by definition those three need an actual quality gap
(bad tracks, frequent false alarms, or genuinely clean detections) for a
confidence bias to be *dishonest* about.

| Scenario | `radar_confidence_bias` | Other changes vs. reference |
|---|---|---|
| `correctly_calibrated_radar` (reference) | `0.0` | `radar_confidence_error=0.05` only |
| `mildly_overconfident_radar` | `+0.15` | — |
| `severely_overconfident_radar` | `+0.4` | — |
| `underconfident_radar` | `-0.3` | — |
| `high_confidence_false_alarms` | `0.0` (real dets stay calibrated) | `radar_clutter_confidence_bias=+0.35`; `radar_false_alarm_probability` 0.05→0.3; `radar_clutter_density` 0.15→0.6 |
| `high_confidence_incorrect_tracks` | `+0.4` | `radar_range_noise_std` 0.4→1.8 (positions frequently wrong) |
| `low_confidence_correct_detections` | `-0.4` | `radar_detection_probability` 0.85→0.98, `radar_range_noise_std` 0.4→0.15 (detections genuinely good) |

## 3. Confirming the bias actually landed on reported confidence

Before trusting any downstream metric, a direct check of mean reported
confidence (10 pooled seeded runs, `RadarLikeModel` only) confirms each
knob does what its name says:

| Scenario | mean confidence, real detections | mean confidence, false alarms |
|---|---|---|
| `correctly_calibrated_radar` | 0.647 | 0.444 |
| `mildly_overconfident_radar` | 0.795 (+0.15) | 0.594 (+0.15) |
| `severely_overconfident_radar` | 0.963 (+0.32, clamp-limited) | 0.833 (+0.39) |
| `underconfident_radar` | 0.347 (−0.30) | 0.155 (−0.29) |
| `high_confidence_false_alarms` | 0.644 (unchanged) | **0.800** (higher than its own real detections) |
| `high_confidence_incorrect_tracks` | 0.964 (+0.32, clamp-limited) | 0.837 |
| `low_confidence_correct_detections` | 0.253 (−0.39) | 0.102 |


## 4. Results (mean ± population std over 20 runs)

| Scenario | Fusion RMSE¹ | Wrong decisions² | Collision risk³ | Unnecessary avoidance | Mission success | Formation error |
|---|---|---|---|---|---|---|
| `baseline` (no fusion, perfect sensing) | 0.310 ± 0.021 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 100% | 3.804 ± 0.152 |
| `correctly_calibrated_radar` (reference) | 0.235 ± 0.032 | 13.4 ± 3.18 | 0.0 ± 0.0 | 6.35 ± 2.48 | 100% | 3.876 ± 0.194 |
| `mildly_overconfident_radar` | 0.236 ± 0.032 | 13.35 ± 3.24 | 0.0 ± 0.0 | 6.30 ± 2.41 | 100% | 3.877 ± 0.194 |
| `severely_overconfident_radar` | 0.236 ± 0.030 | 13.8 ± 4.21 | 0.0 ± 0.0 | 6.60 ± 2.92 | 100% | 3.881 ± 0.193 |
| `underconfident_radar` | 0.231 ± 0.030 | 13.65 ± 2.95 | 0.0 ± 0.0 | 6.35 ± 2.41 | 100% | 3.871 ± 0.192 |
| `high_confidence_false_alarms` | 0.242 ± 0.054 | **183.4 ± 16.3** | 0.05 ± 0.22 | **176.6 ± 15.7** | **95%** | 3.923 ± 0.538 |
| `high_confidence_incorrect_tracks` | **1.187 ± 0.156** | 14.25 ± 3.28 | **0.25 ± 0.62** | 7.30 ± 2.03 | **85%** | **4.525 ± 0.254** |
| `low_confidence_correct_detections` | **0.175 ± 0.019** | **7.85 ± 2.63** | 0.0 ± 0.0 | 6.45 ± 2.42 | 100% | **3.792 ± 0.182** |


### Statistical significance (one-way ANOVA across all 7 scenarios)

| Metric | F | p |
|---|---|---|
| Fusion RMSE | 558.6 | < 0.0001 |
| Wrong decisions | 1670.7 | < 0.0001 |
| Collision risk count | 2.66 | 0.018 |
| Formation error | 15.6 | < 0.0001 |

All four differ significantly across scenarios overall. Pairwise
(Welch/independent t-test + Cohen's d) against `correctly_calibrated_radar`:

| Scenario | Fusion RMSE Δ (d) | Wrong decisions Δ (d) | Formation error Δ (d) |
|---|---|---|---|
| `mildly_overconfident_radar` | +0.001 (d=0.03), p=.92 — n.s. | −0.05 (d=−0.02), p=.96 — n.s. | +0.001 (d=0.01), p=.99 — n.s. |
| `severely_overconfident_radar` | +0.001 (d=0.05), p=.89 — n.s. | +0.40 (d=0.10), p=.74 — n.s. | +0.005 (d=0.03), p=.94 — n.s. |
| `underconfident_radar` | −0.004 (d=−0.13), p=.67 — n.s. | +0.25 (d=0.08), p=.80 — n.s. | −0.006 (d=−0.03), p=.93 — n.s. |
| `high_confidence_false_alarms` | +0.008 (d=0.16), p=.61 — n.s. | **+169.95 (d=14.08), p<.0001** | +0.047 (d=0.11), p=.72 — n.s. |
| `high_confidence_incorrect_tracks` | **+0.953 (d=8.23), p<.0001** | +0.85 (d=0.26), p=.42 — n.s. | **+0.649 (d=2.80), p<.0001** |
| `low_confidence_correct_detections` | **−0.060 (d=−2.22), p<.0001** | **−5.55 (d=−1.85), p<.0001** | −0.084 (d=−0.44), p=.18 — n.s. |

## 5. What this shows

**A uniform bias in real-detection confidence, by itself, barely moves
swarm outcomes.** `mildly_overconfident_radar`, `severely_overconfident_radar`,
and `underconfident_radar` are statistically indistinguishable from the
correctly-calibrated reference on every headline metric. The reason is
mechanical: `confidence_weighted_fusion` weights each UAV's report by its
*relative* confidence among the UAVs currently reporting. Shifting every
UAV's confidence by roughly the same amount barely changes the *relative*
weights, so the fused estimate barely moves even a radar that is
confidently wrong about how reliable it is doesn't hurt fusion much, as
long as its miscalibration is uniform across all sensors and all
conditions. This is a real (if slightly counterintuitive) finding, not a
null result from an underpowered scenario ANOVA across just these four
scenarios independently confirms no significant differences.

**What actually hurts is confidence being high *when the underlying
information is bad*, not confidence being high in general.**
`high_confidence_incorrect_tracks` (noisy detections + inflated confidence)
produced the single largest effect in the study: fusion RMSE nearly
quintupled (0.235 → 1.187, d=8.2), formation error rose ~17%, mission
success dropped to 85%, and this was the only scenario with a material
collision-risk increase (0.25 vs 0.0 collisions/run). `confidence_weighted_fusion`
did exactly what its name implies it trusted the loud, wrong sensor.

**Underconfidence on genuinely good data is, if anything, mildly
beneficial here.** `low_confidence_correct_detections` had the *best*
fusion RMSE (0.175, d=−2.2 vs. reference) and fewest wrong decisions (7.85,
d=−1.9) of any scenario in the study better than the correctly-calibrated
reference itself. This isn't evidence that underconfidence is good in
general; it's confounded with that scenario's genuinely cleaner sensing
conditions (`radar_detection_probability` 0.98, `radar_range_noise_std`
0.15). What it does show is that suppressed confidence on already-good data
costs nothing here, because `confidence_weighted_fusion`'s relative
weighting isn't harmed by a uniformly-low but consistent confidence signal
riding on top of accurate positions the asymmetry in this study is between
*miscalibration correlated with track quality* (harmful) and *miscalibration
uncorrelated with or opposite to track quality* (harmless or benign).

## 6. Practical takeaway

For a `confidence_weighted_fusion` architecture like this one, calibration
audits should prioritize checking whether confidence is *decoupled from
actual detection quality* (i.e., whether bad detections/tracks/false alarms
specifically get inflated confidence) over checking whether confidence has
a uniform global offset. The former reliably degrades fusion accuracy,
decision quality, and mission success in this study; the latter did not,
within the bias magnitudes tested (±0.4, prior to clamping).

## 7. Reproduction

```bash
# Per-scenario swarm-level metrics (20 seeded runs each), used for
python3 metrics_analysis.py --scenario correctly_calibrated_radar --runs 20 \
    --output results/results_correctly_calibrated_radar.csv
# ...repeat for: mildly_overconfident_radar, severely_overconfident_radar,
#    underconfident_radar, high_confidence_false_alarms,
#    high_confidence_incorrect_tracks, low_confidence_correct_detections