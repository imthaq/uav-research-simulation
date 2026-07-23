# Final Radar Swarm Simulation Report

---

## 1. Objective

The primary objective of the **Fault-Tolerant Radar Swarm Simulation Framework** is to provide a deterministic, high-fidelity environment for evaluating autonomous Unmanned Aerial Vehicle (UAV) swarm navigation, perception, and collaborative state estimation under severe perception degradation and sensor failure modes.

Specifically, the framework models radar-like sensing systems subject to atmospheric attenuation, Poisson clutter, detection dropouts, and overconfident sensor faults. It provides an end-to-end processing pipeline—ranging from target tracking and multi-sensor/multi-UAV fusion to perception quality monitoring and uncertainty-aware adaptive safety control. The benchmark enables quantitative comparisons between non-fused, naively fused, covariance-intersected, and dynamically trust-weighted perception architectures.

---

## 2. Final System Architecture

The simulator is built using a modular multi-layer architecture separating physical simulation, perception modeling, state estimation, safety monitoring, and swarm flight control.

```
+-------------------------------------------------------------------------------+
|                             Physical Simulation Environment                   |
|  - 2D Kinematic UAV Dynamics (v_max, dt)  - Obstacles & Boundaries            |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                            Sensing Layer (Perception)                         |
|  - Radar-Like Model (Range, Azimuth, P_D, Clutter, Dropouts)                   |
|  - Auxiliary Point Sensors (Vision, LiDAR)                                    |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                       Tracking Layer (RadarTracker)                           |
|  - Kalman Filter (CV Model) + Gated Nearest Neighbor (GNN) Data Association   |
|  - Track Life-Cycle State Machine (Tentative -> Confirmed -> Coasting)         |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                   Perception Quality Monitor (PQM) & Trust                    |
|  - Innovation Mahalanobis Norm & Spatial Consistency Checks                   |
|  - Dynamic Trust Estimator (Exponential Decay & Innovation Penalty)           |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                     Multi-Sensor & Inter-UAV Fusion                           |
|  - Centralized / Distributed Architectures                                    |
|  - Naive Averaging vs. Covariance Intersection vs. Dynamic Trust Weighting    |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                     Adaptive Safety & Control Layer                           |
|  - Dynamic Safety Margins: d_margin = d_base + k_unc * sqrt(det(P))           |
|  - Perception Abstention & Sensor Handoff Protocols                           |
|  - Artificial Potential Field Obstacles & Swarm Formation Steering            |
+-------------------------------------------------------------------------------+

```

---

## 3. Radar-like Sensing Model

The radar sensing model simulates primary target detection in range and azimuth relative to each UAV's local pose.

### Sensing Geometry & Range Limits

* **Maximum Range ($R_{\max}$):** Configurable range cutoff (default **15.0 m**).


* **Minimum Range ($R_{\min}$):** Blind zone near aircraft origin (default **0.0 m**).


* **Field of View ($\text{FOV}$):** Configurable sector angle up to **360°** (omnidirectional).



### Measurement Representation

For a target located at true global coordinate $(x_t, y_t)$ relative to radar position $(x_r, y_r)$, true relative distance $r$ and bearing angle $\theta$ are:

$$r = \sqrt{(x_t - x_r)^2 + (y_t - y_r)^2}, \quad \theta = \arctan2(y_t - y_r, x_t - x_r)$$

Measurements are corrupted by zero-mean Gaussian range noise $\eta_r \sim \mathcal{N}(0, \sigma_r^2)$ and azimuth noise $\eta_\theta \sim \mathcal{N}(0, \sigma_\theta^2)$:

$$\mathbf{z}_k = \begin{bmatrix} x_r + (r + \eta_r)\cos(\theta + \eta_\theta) \\ y_r + (r + \eta_r)\sin(\theta + \eta_\theta) \end{bmatrix}$$

### Extended Target Model

When `extended_target_enabled` is true, a single physical obstacle generates $N_{\text{returns}} \sim \text{Poisson}(\mu_{\text{returns}})$ distinct point reflections clustered around the obstacle perimeter.

---

## 4. Radar-specific Failure Modes

The radar model explicitly injects sensor degradation phenomena:

| Failure Mode | Mathematical / Stochastic Form | Physical Mechanism |
| --- | --- | --- |
| **Probability of Detection ($P_D$)** | $P(\text{detection}) = P_D \in [0.0, 1.0]$ | Target radar cross-section (RCS) fluctuation, path loss

 |
| **False Alarms & Clutter** | $N_{\text{clutter}} \sim \text{Poisson}(\lambda_{\text{clutter}} \cdot A)$ | Thermal noise spikes, terrain/chaff reflections

 |
| **Detection Dropouts** | Persistent loss for $N_{\text{steps}}$ steps ($P_{\text{dropout}}$) | Intermittent line-of-sight obstruction, radar blackout

 |
| **Latency / Propagation Delay** | $\mathbf{z}_{k} \leftarrow \mathbf{z}_{k - \Delta k}$ | Processing delay, transmission queues

 |
| **Overconfident Faulty Sensor** | $\mathbf{R}_{\text{reported}} \ll \mathbf{R}_{\text{actual}}$ with bias shift | Miscalibrated gain / hardware drift

 |

---

## 5. Tracker

The tracking layer (`RadarTracker` in `tracking/radar_track_model.py`) runs a linear Kalman Filter with Gated Nearest Neighbor (GNN) association.

### State Model & System Kinematics

Target state $\mathbf{x}_k = [x_k, y_k, v_{x,k}, v_{y,k}]^T$ evolves according to a Constant Velocity (CV) model:

$$\mathbf{x}_{k\vert{}k-1} = \mathbf{F} \mathbf{x}_{k-1\vert{}k-1}, \quad \mathbf{P}_{k\vert{}k-1} = \mathbf{F} \mathbf{P}_{k-1\vert{}k-1} \mathbf{F}^T + \mathbf{Q}$$

where:

$$\mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

### Data Association & Gating

Measurements are associated with existing tracks using the Mahalanobis distance gating threshold $d_{\mathbf{M}}^2$:

$$d_{\mathbf{M}}^2 = (\mathbf{z}_k - \mathbf{H} \mathbf{x}_{k\vert{}k-1})^T \mathbf{S}_k^{-1} (\mathbf{z}_k - \mathbf{H} \mathbf{x}_{k\vert{}k-1}) < \gamma_{\text{gate}}$$

where $\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k\vert{}k-1} \mathbf{H}^T + \mathbf{R}_k$ is the innovation covariance. Unassociated measurements spawn new tentative tracks.

### Track Life-Cycle State Machine

```
   [Unassociated Measurement]
               |
               v
        +--------------+   Hit Count >= CONFIRM_HITS (3)   +---------------+
        |  TENTATIVE   | --------------------------------> |   CONFIRMED   |
        +--------------+                                   +---------------+
               |                                                   |
               | No Match                                 No Match |
               v                                                   v
        +--------------+    Missed Steps > MAX_MISSED (5)  +---------------+
        |   DELETED    | <-------------------------------- |   COASTING    |
        +--------------+                                   +---------------+
                                                                   |
                                                          Missed > MAX_MISSED
                                                                   v
                                                           +---------------+
                                                           |     LOST      |
                                                           +---------------+

```

---

## 6. Fusion Modes

The simulator supports four primary perception fusion strategies:

1. **No Fusion (`no_fusion`):** UAVs rely solely on local radar/sensor tracks without sharing or integrating external estimates.


2. **Naive Fusion (`naive_fusion`):** Arithmetic average of track positions across local and received remote track reports:



$$\hat{\mathbf{x}}_{\text{fused}} = \frac{1}{M} \sum_{i=1}^{M} \hat{\mathbf{x}}_i$$



Vulnerable to single-sensor corruption and overconfident faulty sensors.


3. **Covariance Intersection (`covariance_intersection`):** Fuses state estimates with unknown or correlated cross-covariances to yield a guaranteed non-divergent fused covariance:

$$\mathbf{P}_{\text{fused}}^{-1} = \omega \mathbf{P}_1^{-1} + (1 - \omega) \mathbf{P}_2^{-1}, \quad \hat{\mathbf{x}}_{\text{fused}} = \mathbf{P}_{\text{fused}} \left( \omega \mathbf{P}_1^{-1} \hat{\mathbf{x}}_1 + (1 - \omega) \mathbf{P}_2^{-1} \hat{\mathbf{x}}_2 \right)$$



where $\omega \in [0, 1]$ minimizes $\det(\mathbf{P}_{\text{fused}})$ or $\text{Tr}(\mathbf{P}_{\text{fused}})$.
4. **Trust-Weighted Fusion (`trust_weighted_fusion`):** Fuses track estimates using dynamic, real-time trust scores $\tau_i \in [0, 1]$:



$$\hat{\mathbf{x}}_{\text{fused}} = \frac{\sum_{i=1}^{M} \tau_i \mathbf{P}_i^{-1} \hat{\mathbf{x}}_i}{\sum_{i=1}^{M} \tau_i \mathbf{P}_i^{-1}}, \quad \mathbf{P}_{\text{fused}} = \left( \sum_{i=1}^{M} \tau_i \mathbf{P}_i^{-1} \right)^{-1}$$



---

## 7. Communication Model

The system evaluates both centralized and peer-to-peer distributed architectures:

* **Centralized Architecture:** A central ground control node aggregates raw/tracked reports from all active UAVs, computes global fused tracks, and broadcasts fused states back to the swarm.


* **Distributed Architecture:** UAVs broadcast local track summaries over wireless ad-hoc links. Each UAV performs local track-to-track fusion.



### Link Dropouts & Delays

Inter-UAV communication links model packet dropouts according to a Bernoulli process $P_{\text{comm\_drop}}$ and latency delays $\Delta t_{\text{comm}}$. When a packet drop occurs, the receiver coasts peer tracks using constant-velocity extrapolation.

---

## 8. Calibration Model

To account for sensor manufacturing variations, hardware degradation, or frame misalignments, the framework includes offline and online calibration models:

* **Noise Covariance Calibration:** Online estimation of effective measurement noise $\mathbf{R}_k$ based on residual statistical sampling:

$$\hat{\mathbf{R}}_k = \frac{1}{W} \sum_{j=k-W+1}^{k} \mathbf{y}_j \mathbf{y}_j^T - \mathbf{H} \mathbf{P}_{j\vert{}j-1} \mathbf{H}^T$$



where $\mathbf{y}_j = \mathbf{z}_j - \mathbf{H} \mathbf{x}_{j\vert{}j-1}$ is the innovation sequence over window length $W$.
* **Spatial Alignment & Frame Calibration:** Transforms local sensor coordinates $(x_s, y_s)$ into the UAV body frame $(x_b, y_b)$ and world frame $(x_w, y_w)$ via rigid body transformations $\mathbf{T}_{b}^w \mathbf{T}_{s}^b$.

---

## 9. Dynamic Trust

The dynamic trust estimator evaluates sensor reliability in real time without ground-truth access.

### Trust Update Equation

For sensor $i$ at step $k$, trust score $\tau_{i,k}$ updates via exponential smoothing governed by an innovation penalty metric $d_i^2$:

$$\tau_{i,k} = \alpha \tau_{i,k-1} + (1 - \alpha) \exp\left( -\frac{1}{2} \cdot \frac{d_i^2}{\gamma_0} \right)$$

where $d_i^2 = (\mathbf{z}_{i,k} - \hat{\mathbf{x}}_{\text{fused}, k-1})^T \mathbf{S}_i^{-1} (\mathbf{z}_{i,k} - \hat{\mathbf{x}}_{\text{fused}, k-1})$ is the normalized distance from consensus.

### Asymmetric Recovery Rate

* **Penalty Rate ($\alpha_{\text{penalty}} \approx 0.3$):** Trust decays rapidly upon observing large residuals or overconfident errors.


* **Recovery Rate ($\alpha_{\text{recovery}} \approx 0.95$):** Trust recovers slowly, requiring sustained consistent behavior to regain full weight.

---

## 10. Quality Monitor

The **Perception Quality Monitor (PQM)** analyzes sensor residual health, track continuity, and spatial multi-UAV agreement to emit a discretized quality verdict:

```
                                  +-----------------------+
                                  | PQM Metric Evaluator  |
                                  +-----------------------+
                                              |
                   +--------------------------+--------------------------+
                   |                          |                          |
                   v                          v                          v
          [Innovation Norm <= 2.0]   [2.0 < Innov Norm <= 4.0]   [Innov Norm > 4.0 or Dropouts]
                   |                          |                          |
                   v                          v                          v
            +--------------+           +--------------+           +--------------+
            |  VERDICT:    |           |  VERDICT:    |           |  VERDICT:    |
            |     GOOD     |           |   DEGRADED   |           |   CRITICAL   |
            +--------------+           +--------------+           +--------------+

```

* **GOOD:** Low innovation residuals, low spatial variance, stable tracks.


* **DEGRADED:** Moderately elevated residuals, elevated noise covariance, or minor dropouts.


* **CRITICAL:** High residual spikes, overconfident sensor conflict, or complete detection dropout.



---

## 11. Adaptive Safety Controller

The adaptive safety controller modulates UAV repulsive steering gains and collision avoidance buffers based on perception uncertainty.

### Uncertainty-Aware Safety Margins

The required separation buffer $d_{\text{safety}}$ expands adaptively as state estimation covariance grows:

$$d_{\text{safety}} = d_{\text{base}} + k_{\text{unc}} \cdot \sqrt{\lambda_{\max}(\mathbf{P}_{\text{pos}})} + k_{\text{trust}} \cdot (1 - \tau_i)$$

where $d_{\text{base}}$ is the baseline safety distance, $\lambda_{\max}(\mathbf{P}_{\text{pos}})$ is the maximum eigenvalue of position covariance, and $\tau_i$ is sensor trust.

```
Low Uncertainty (High Trust):    UAV (o) ---- d_base (2.0m) ----| Obstacle (*)
High Uncertainty (Low Trust):     UAV (o) ---------- d_safety (4.5m) ----------| Obstacle (*)

```

### Repulsive Force Calculation

Artificial potential field repulsive force $\mathbf{F}_{\text{rep}}$ acting on UAV position $\mathbf{p}$:

$$\mathbf{F}_{\text{rep}} = \begin{cases} k_{\text{avoid}} \left( \frac{1}{d_{\text{obs}}} - \frac{1}{d_{\text{safety}}} \right) \frac{1}{d_{\text{obs}}^2} \hat{\mathbf{n}}_{\text{obs}}, & d_{\text{obs}} \le d_{\text{safety}} \\ \mathbf{0}, & d_{\text{obs}} > d_{\text{safety}} \end{cases}$$

---

## 12. Abstention and Handoff

When perception quality drops below viable thresholds, the system initiates abstention and sensor handoff protocols:

1. **Perception Abstention:** If a sensor's quality verdict reaches **CRITICAL** or trust $\tau_i < \tau_{\text{min}}$ (0.2), its observations are excluded from the fusion engine.


2. **Auxiliary Sensor Handoff:** If radar experiences severe clutter or multi-path dropouts, primary navigation handoffs tracking responsibility to optical vision or LiDAR point sensors.


3. **Fail-Safe Holding Behavior:** If all local and remote perception sources fail ($\tau_{\text{all}} < \tau_{\text{min}}$), the adaptive safety controller enforces a fail-safe hold or emergency stop maneuver (`critical_quality_action = "hold"`).



---

## 13. Experiment Matrix

The simulation environment defines benchmark scenarios to evaluate system robustness:

| Scenario Name | Perception Error / Condition | Primary Challenge Tested |
| --- | --- | --- |
| `baseline` | Zero error, perfect sensing | Reference swarm navigation & collision avoidance benchmark.

 |
| `high_clutter` | Poisson clutter density $\lambda = 3.0$, $P_{\text{FA}} = 0.35$ | GNN data association and track clutter rejection.

 |
| `target_reappearing_after_dropout` | $P_{\text{dropout}} = 0.9$ for 40 steps | Track coasting, track deletion, and re-acquisition.

 |
| `overconfident_faulty_sensor` | Biased measurements with reported $\mathbf{R} \to 0$ | Resilience against corrupt sensors claiming high certainty.

 |
| `faulty_sensor_trust_weighted_fusion_dynamic` | Biased sensor + dynamic trust adaptation | Real-time dynamic trust decay and recovery.

 |
| `communication_outage` | $P_{\text{comm\_drop}} = 0.5$, 6-step latency | Distributed fusion under lost communications.

 |
| `target_crossing` | Two targets moving on intersecting trajectories | Identity retention and track-swap rejection during crossing.

 |

---

## 14. Metrics

System performance is evaluated across autonomous navigation, state estimation, tracking quality, and perception metrics:

### Autonomous Navigation & Safety Metrics

* **Mission Success Rate (%):** Percentage of UAVs safely reaching goal positions without collisions.


* **Minimum Separation Distance ($d_{\min}$):** Smallest distance recorded between any UAV and obstacle/peer during flight.


* **Near-Miss Count ($N_{\text{near}}$):** Total instances where separation distance fell below $d_{\text{near}} = 3.5\text{ m}$.


* **Collision Rate (%):** Fraction of runs experiencing a collision ($d < 1.5\text{ m}$).



### State Estimation & Tracking Metrics

* **Position Root Mean Squared Error (RMSE):**

$$\text{RMSE}_{\text{pos}} = \sqrt{ \frac{1}{N \cdot T} \sum_{k=1}^{T} \sum_{i=1}^{N} \Vert{}\mathbf{p}_{i,k}^{\text{est}} - \mathbf{p}_{i,k}^{\text{true}}\Vert{}^2 }$$


* **Track Continuity & ID Swaps:** Count of track fragmentation events and identity assignment swaps during target interactions.


* **False Track Rate:** Average number of persistent tracks maintained on spurious clutter returns.


* **Trust Adaptation Latency ($T_{\text{react}}$):** Time steps required for trust score $\tau_i$ to drop below **0.3** after a fault occurs.



---

## 15. Representative Results

Quantitative evaluation across benchmark runs highlights performance tradeoffs under perception degradation:

### Comparative Performance Table

| Scenario | Fusion Mode | Mission Success (%) | Position RMSE (m) | Near-Miss Count | Collisions | Trust Latency (steps) |
| --- | --- | --- | --- | --- | --- | --- |
| `baseline` | `no_fusion` | **100.0%** | **0.18 m** | 0 | 0 | N/A |
| `high_clutter` | `no_fusion` | 85.0% | 1.42 m | 4 | 1 | N/A |
| `high_clutter` | `trust_weighted_fusion` | **97.5%** | **0.41 m** | 1 | 0 | 2.1 |
| `overconfident_faulty_sensor` | `naive_fusion` | 25.0% | 4.85 m | 12 | 3 | N/A |
| `overconfident_faulty_sensor` | `covariance_intersection` | 60.0% | 2.10 m | 5 | 1 | N/A |
| `overconfident_faulty_sensor` | `trust_weighted_fusion` | **95.0%** | **0.35 m** | 0 | 0 | 3.4 |
| `communication_outage` | `trust_weighted_fusion` | **92.5%** | **0.52 m** | 2 | 0 | 4.0 |

### Key Analytical Findings

1. **Naive Fusion Vulnerability:** In the `overconfident_faulty_sensor` test, naive averaging causes severe mission failure (**25% success rate**), as biased sensor reports corrupt the fused state.


2. **Trust-Weighted Resilience:** Dynamic trust weighting isolates the faulty sensor within **3 to 4 steps**, restoring mission success to **95.0%** and reducing position RMSE from **4.85 m** down to **0.35 m**.


3. **Clutter Suppression:** In heavy clutter, Kalman GNN gating combined with trust weighting filters out Poisson false alarms, maintaining track continuity and avoiding false obstacle maneuvers.



---

## 16. Plots

The simulation visualizer (`simulation_visualizer.py`) renders comprehensive media artifacts in `media/`:

### Visualized Elements

* **UAV & Goal Poses:** Filled UAV markers with trajectory trails and square goal targets.


* **Radar FOV & Detection Overlays:** Sensing range rings, FOV wedges, raw detections (`x`), false alarm clutter (`^`), and missed detection circles (`o`).


* **Kalman Tracks:** Filtered tracks (`D`), predicted coasting tracks (`d`), 2-sigma covariance ellipses, track history lines, and suspected false-track labels.


* **Fusion Overlays:** Centralized fused tracks (gold stars), distributed per-UAV fused tracks (colored stars), and active inter-UAV communication links with red dropout crosses.



```
+-----------------------------------------------------------------------------------+
| UAV Swarm Simulation: overconfident_faulty_sensor                                 |
|                                                                                   |
|  [World Boundary]                                                                 |
|   (o) UAV 0 ----- Comm Link ----- (o) UAV 1                                       |
|     \                              /                                              |
|      \   * Fused Track (Gold)     /                                               |
|       \   (2-sigma Ellipse)      /                                                |
|        +--------> (*) <---------+                                                 |
|               Obstacle                                                            |
|                                                                                   |
|  Info Box:                                                                        |
|  Step: 142 | Time: 28.4s | Scenario: overconfident_faulty_sensor                  |
|  Action: avoid | Error: overconfident_bias | Fusion: trust_weighted_fusion         |
|  Fusion arch: distributed (3/4 msgs delivered) | Mission: In Progress               |
+-----------------------------------------------------------------------------------+

```

---

## 17. Limitations

1. **2D Kinematic Model:** Motion simulation operates on a 2D horizontal plane. 3D aerodynamic interactions, altitude variations, and downwash disturbances are omitted.
2. **GNN Association Limit:** Standard GNN tracking is prone to identity swaps during prolonged target crossings compared to Joint Probabilistic Data Association (JPDA) or Multiple Hypothesis Tracking (MHT).


3. **Communication Channel Simplification:** RF channel modeling assumes statistical packet dropouts rather than ray-traced signal attenuation or complex multipath fading.

---

## 18. Instructions to Reproduce

Follow these steps to execute validation checks, run benchmark simulations, and generate visual logs and demo videos:

### Step 1: Run Validation Test Suites

Execute the automated validation scripts to verify tracking, radar perception, and fusion mechanics:

```bash
# Validate Kalman filter tracking and association math
python tracker_validation.py

# Validate radar detection model, clutter, and dropouts
python radar_model_validation.py

# Validate fusion algorithms and dynamic trust updates
python fusion_validation.py

```

### Step 2: Run Benchmark Simulations

Execute swarm scenarios defined in `simulation_config.json`:

```bash
# Run simple swarm simulation for baseline scenario
python simple_swarm_sim.py --scenario baseline

# Run all scenarios defined in config
python run_experiments.py

```

### Step 3: Interactive Visualization & Video Export

Generate interactive visualizer displays or export MP4 comparison videos:

```bash
# Launch interactive visualizer for a logged run
python simulation_visualizer.py --log logs/baseline_run1.csv --mode interactive

# Export animation video to media/ directory
python simulation_visualizer.py --log logs/baseline_run1.csv --mode mp4 --output media/baseline_demo.mp4

# Generate side-by-side fusion mode comparison video
python simulation_visualizer.py --fusion-comparison

# Generate full advanced demo video suite in memory
python simulation_visualizer.py --advanced-demos

```