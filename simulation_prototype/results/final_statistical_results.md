# Final Statistical Results

## Group Comparisons

### No Fusion vs Fusion

**No Fusion (N=110) vs Fusion (N=200)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 7.7636 | 10.0650 | 0.0000 | 0.0000 | 30.8809 | 32.2082 | 0.0000 | 163.0000 | 0.0000 | 167.0000 | ±5.7710 | ±4.4638 | -0.0725 | 5.3702e-01 | ns |
| mission_success_num | 0.8364 | 0.7750 | 1.0000 | 1.0000 | 0.3716 | 0.4186 | 0.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0695 | ±0.0580 | 0.1524 | 1.8506e-01 | ns |
| avg_response_time_s | 1.1401 | 1.2387 | 0.0000 | 0.0380 | 4.3292 | 4.0872 | 0.0000 | 25.0800 | 0.0000 | 28.2220 | ±0.8090 | ±0.5665 | -0.0236 | 8.4510e-01 | ns |
| fused_position_rmse | 0.3130 | 0.5694 | 0.2956 | 0.2812 | 0.3415 | 0.6220 | 0.0000 | 2.1363 | 0.0000 | 3.6761 | ±0.0638 | ±0.0862 | -0.4751 | 4.2072e-06 | *** |

### Naive Fusion vs Trust-Weighted Fusion

**Naive (N=35) vs Trust-Weighted (N=115)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 0.0000 | 17.4087 | 0.0000 | 0.0000 | 0.0000 | 41.0175 | 0.0000 | 0.0000 | 0.0000 | 167.0000 | ±0.0000 | ±7.4968 | -0.4836 | 1.3406e-05 | *** |
| mission_success_num | 1.0000 | 0.6435 | 1.0000 | 1.0000 | 0.0000 | 0.4811 | 1.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0000 | ±0.0879 | 0.8444 | 1.5011e-12 | *** |
| avg_response_time_s | 0.0097 | 2.0793 | 0.0000 | 0.0620 | 0.0203 | 5.2346 | 0.0000 | 0.0750 | 0.0000 | 28.2220 | ±0.0067 | ±0.9567 | -0.4505 | 4.5684e-05 | *** |
| fused_position_rmse | 0.3133 | 0.6559 | 0.1997 | 0.3277 | 0.2906 | 0.7257 | 0.1696 | 1.0976 | 0.0000 | 3.6761 | ±0.0963 | ±0.1326 | -0.5255 | 7.1064e-05 | *** |

### Fixed Trust vs Dynamic Trust

**Fixed Trust (Baseline) (N=5) vs Dynamic Trust Scenario (N=5)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 0.0000 | 1.4000 | 0.0000 | 0.0000 | 0.0000 | 3.1305 | 0.0000 | 0.0000 | 0.0000 | 7.0000 | ±0.0000 | ±2.7440 | -0.6325 | 3.7390e-01 | ns |
| mission_success_num | 1.0000 | 0.8000 | 1.0000 | 1.0000 | 0.0000 | 0.4472 | 1.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0000 | ±0.3920 | 0.6325 | 3.7390e-01 | ns |
| avg_response_time_s | 0.0026 | 0.0026 | 0.0000 | 0.0000 | 0.0058 | 0.0058 | 0.0000 | 0.0130 | 0.0000 | 0.0130 | ±0.0051 | ±0.0051 | 0.0000 | 1.0000e+00 | ns |
| fused_position_rmse | 0.2955 | 1.6674 | 0.2869 | 1.6272 | 0.0173 | 0.1054 | 0.2843 | 0.3251 | 1.5353 | 1.7814 | ±0.0152 | ±0.0924 | -18.1616 | 5.3685e-06 | *** |

### Centralized vs Distributed Fusion
(Data not fully split by centralized/distributed in this CSV slice.)

### Normal Communication vs Packet Loss

**Normal Comm (N=240) vs Packet Loss (N=70)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 5.3583 | 22.5857 | 0.0000 | 0.0000 | 23.6306 | 48.3368 | 0.0000 | 163.0000 | 0.0000 | 167.0000 | ±2.9897 | ±11.3236 | -0.5570 | 5.0735e-03 | ** |
| mission_success_num | 0.9000 | 0.4429 | 1.0000 | 0.0000 | 0.3006 | 0.5003 | 0.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0380 | ±0.1172 | 1.2868 | 1.7021e-10 | *** |
| avg_response_time_s | 0.6632 | 3.0569 | 0.0060 | 0.4630 | 3.1561 | 6.2284 | 0.0000 | 25.0800 | 0.0000 | 28.2220 | ±0.3993 | ±1.4591 | -0.5907 | 2.6646e-03 | ** |
| fused_position_rmse | 0.4526 | 0.5667 | 0.2678 | 0.3721 | 0.5309 | 0.6172 | 0.0000 | 3.1849 | 0.0000 | 3.6761 | ±0.0672 | ±0.1446 | -0.2069 | 1.6377e-01 | ns |

### Normal Radar vs Degraded Radar

**Normal (N=190) vs Degraded (N=120)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 10.5526 | 7.1833 | 0.0000 | 0.0000 | 32.0297 | 31.2254 | 0.0000 | 163.0000 | 0.0000 | 167.0000 | ±4.5544 | ±5.5869 | 0.1062 | 3.6043e-01 | ns |
| mission_success_num | 0.8421 | 0.7250 | 1.0000 | 1.0000 | 0.3656 | 0.4484 | 0.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0520 | ±0.0802 | 0.2930 | 1.7200e-02 | * |
| avg_response_time_s | 1.1386 | 1.3068 | 0.0000 | 0.0750 | 3.8006 | 4.7063 | 0.0000 | 25.0800 | 0.0000 | 28.2220 | ±0.5404 | ±0.8421 | -0.0403 | 7.4199e-01 | ns |
| fused_position_rmse | 0.4977 | 0.4478 | 0.2655 | 0.3236 | 0.5947 | 0.4792 | 0.0000 | 3.1849 | 0.0000 | 3.6761 | ±0.0846 | ±0.0857 | 0.0901 | 4.1786e-01 | ns |

### Low Clutter vs High Clutter

**Low Clutter (N=5) vs High Clutter (N=10)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | ±0.0000 | ±0.0000 | 0.0000 | nan | ns |
| mission_success_num | 1.0000 | 0.5000 | 1.0000 | 0.5000 | 0.0000 | 0.5270 | 1.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0000 | ±0.3267 | 1.1402 | 1.4956e-02 | * |
| avg_response_time_s | 0.0026 | 0.0251 | 0.0000 | 0.0130 | 0.0058 | 0.0288 | 0.0000 | 0.0130 | 0.0000 | 0.0750 | ±0.0051 | ±0.0178 | -0.9314 | 3.7923e-02 | * |
| fused_position_rmse | 0.2955 | 0.3919 | 0.2869 | 0.3146 | 0.0173 | 0.2278 | 0.2843 | 0.3251 | 0.1693 | 0.7082 | ±0.0152 | ±0.1412 | -0.5078 | 2.1549e-01 | ns |

### Normal P_D vs Low P_D

**Normal P_D (N=205) vs Low P_D (N=105)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 9.7805 | 8.2095 | 0.0000 | 0.0000 | 30.9525 | 33.2738 | 0.0000 | 163.0000 | 0.0000 | 167.0000 | ±4.2372 | ±6.3645 | 0.0495 | 6.8760e-01 | ns |
| mission_success_num | 0.8488 | 0.6952 | 1.0000 | 1.0000 | 0.3591 | 0.4625 | 0.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0492 | ±0.0885 | 0.3867 | 3.3733e-03 | ** |
| avg_response_time_s | 1.0528 | 1.4983 | 0.0000 | 0.0880 | 3.6709 | 5.0048 | 0.0000 | 25.0800 | 0.0130 | 28.2220 | ±0.5025 | ±0.9573 | -0.1068 | 4.2051e-01 | ns |
| fused_position_rmse | 0.4770 | 0.4810 | 0.2577 | 0.3464 | 0.5769 | 0.5043 | 0.0000 | 3.1849 | 0.0000 | 3.6761 | ±0.0790 | ±0.0965 | -0.0072 | 9.4980e-01 | ns |

### Normal Latency vs High Latency

**Normal Latency (N=275) vs High Latency (N=35)**

| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |
|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|
| collision_risk_count | 5.8582 | 35.8857 | 0.0000 | 8.0000 | 26.1305 | 53.0470 | 0.0000 | 163.0000 | 0.0000 | 167.0000 | ±3.0884 | ±17.5745 | -0.9910 | 2.1914e-03 | ** |
| mission_success_num | 0.8691 | 0.2286 | 1.0000 | 0.0000 | 0.3379 | 0.4260 | 0.0000 | 1.0000 | 0.0000 | 1.0000 | ±0.0399 | ±0.1411 | 1.8367 | 1.5177e-10 | *** |
| avg_response_time_s | 0.6135 | 5.8406 | 0.0130 | 2.4000 | 3.0334 | 7.6666 | 0.0000 | 25.0800 | 1.0000 | 28.2220 | ±0.3585 | ±2.5399 | -1.3645 | 3.1360e-04 | *** |
| fused_position_rmse | 0.4409 | 0.7729 | 0.2846 | 0.3732 | 0.4981 | 0.8217 | 0.0000 | 3.1849 | 0.0000 | 3.6761 | ±0.0589 | ±0.2722 | -0.6110 | 2.4967e-02 | * |

## Correlations: Perception Parameters vs Outcomes

| Parameter | Outcome | Spearman Rho | p-value | Sig |
|-----------|---------|--------------|---------|-----|
| false_positive_rate | collision_risk_count | 0.2881 | 2.4561e-07 | *** |
| false_positive_rate | mission_success_num | -0.2633 | 2.6066e-06 | *** |
| false_positive_rate | avg_response_time_s | 0.1419 | 1.2409e-02 | * |
| false_positive_rate | fused_position_rmse | 0.0749 | 1.8837e-01 | ns |
| false_negative_rate | collision_risk_count | 0.0201 | 7.2381e-01 | ns |
| false_negative_rate | mission_success_num | -0.1504 | 7.9745e-03 | ** |
| false_negative_rate | avg_response_time_s | 0.5498 | 6.8897e-26 | *** |
| false_negative_rate | fused_position_rmse | 0.2873 | 2.6603e-07 | *** |
| noise_level | collision_risk_count | -0.0134 | 8.1441e-01 | ns |
| noise_level | mission_success_num | -0.1217 | 3.2226e-02 | * |
| noise_level | avg_response_time_s | 0.4448 | 1.8110e-16 | *** |
| noise_level | fused_position_rmse | 0.2185 | 1.0502e-04 | *** |
| latency_steps | collision_risk_count | 0.5979 | 1.9538e-31 | *** |
| latency_steps | mission_success_num | -0.5108 | 5.2868e-22 | *** |
| latency_steps | avg_response_time_s | 0.5309 | 6.0558e-24 | *** |
| latency_steps | fused_position_rmse | 0.2347 | 2.9912e-05 | *** |
| dropout_probability | collision_risk_count | 0.4754 | 6.9790e-19 | *** |
| dropout_probability | mission_success_num | -0.5041 | 2.1949e-21 | *** |
| dropout_probability | avg_response_time_s | 0.5809 | 2.2587e-29 | *** |
| dropout_probability | fused_position_rmse | 0.3108 | 2.2750e-08 | *** |
