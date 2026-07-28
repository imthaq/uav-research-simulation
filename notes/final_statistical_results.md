# Final Statistical Results

## 1. Descriptive Statistics
|                      |      mean |   median |       std |    min |     max |   95% CI (±) |
|:---------------------|----------:|---------:|----------:|-------:|--------:|-------------:|
| collision_risk_count | 108.2     |  86.5    | 49.0531   | 66     | 255     |    13.5968   |
| total_near_misses    |  59.08    |  47      | 23.185    | 31     | 140     |     6.42656  |
| avg_formation_error  |   3.87894 |   3.7535 |  0.430711 |  3.124 |   5.034 |     0.119387 |
| mission_success      |   0.58    |   1      |  0.498569 |  0     |   1     |     0.138196 |
| avg_response_time_s  |   0.15598 |   0      |  0.401935 |  0     |   1.7   |     0.111411 |

## 2. Correlations (Perception vs Swarm Outcomes)
- **false_positive_rate** vs collision_risk: rho=nan, p=nan
- **false_negative_rate** vs collision_risk: rho=0.00, p=0.9892
- **noise_level** vs collision_risk: rho=0.09, p=0.5207
- **latency_steps** vs collision_risk: rho=0.13, p=0.3724
- **dropout_probability** vs collision_risk: rho=0.53, p=0.0001

## 3. Scenario & Strategy Comparisons
*Tested on Average Formation Error (Welch's t-test and Cohen's d)*
- **No fusion vs Fusion (Naive)**: t=-2.48, p=0.0375, Cohen's d=-0.88
- **Naive fusion vs Trust-weighted fusion**: t=1.22, p=0.2582, Cohen's d=0.77
- **Fixed trust vs Dynamic trust**: N/A (Parameters not in dataset)
- **Centralized vs Distributed fusion**: N/A (Parameters not in dataset)
- **Normal communication vs Packet loss**: t=-0.08, p=0.9403, Cohen's d=-0.05
- **Normal radar vs Degraded radar**: t=24.72, p=0.0000, Cohen's d=15.64
- **Low clutter vs High clutter**: t=nan, p=nan, Cohen's d=nan
- **Normal P_D vs Low P_D**: t=-1.44, p=0.2238, Cohen's d=-0.91
- **Normal latency vs High latency**: t=-inf, p=0.0000, Cohen's d=nan
