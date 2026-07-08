# Fusion Analysis

## Fusion modes tested
- No fusion
- naive_fusion 
- trust_weighted_fusion

All three share the identical error profile: 20% false-negative rate, position noise (std 1.5), confidence error level 0.15. 5 seeded runs each (seeds 42-46).

## Metrics compared

| Metric | No fusion | naive_fusion | trust_weighted_fusion |
|---|---|---|---|
| Mission success | 1/5 | 4/5 | 0/5 |
| Avg collision count | 25.8 | 0.4 | 3.6 |
| Avg missed-response count | 20.6 | 7.4 | 9.0 |
| Avg fusion-recovered detections | 0.0 | 78.2 | 78.8 |
| Avg near-miss count | 75.0 | 43.6 | 36.8 |
| Avg formation error | 4.646 | 4.154 | 3.987 |
| Avg confidence error | 0.119 | 0.121 | 0.121 |

## Result summary
Fusion of either kind is a large improvement over no fusion: both naive_fusion and trust_weighted_fusion recover roughly 78 detections per run that would otherwise be missed, cutting missed-response counts nearly in half or more (20.6 → 7.4/9.0) and cutting collisions dramatically (25.8 → 0.4 for naive, 25.8 → 3.6 for trust-weighted).

## Limitations
### No Fusion : 
- With no fusion the risk of getting collided into other drones or obstacles increases drastically.
### Naive fusion: 
- Naive fusion decreases the chances of collosion by some percent by still it cannot be trusted weather the object detected is true or not.



## Whether results support the hypothesis
Yes it does, both fusion modes cut collisions by roughly 7%-64% relative to no fusion.
