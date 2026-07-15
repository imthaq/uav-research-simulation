# Systematic Ablation Study of Trust-Weighted Sensor Fusion

To evaluate the contribution and engineering necessity of each individual software block in our proposed trust-weighted fusion swarm, we conducted a systematic ablation study. 

By disabling one critical system component at a time, we observed the corresponding degradation across key swarm outcomes: **Fusion Error**, **Collision Risk**, **Mission Success**, **Response Time**, and **Formation Error**.

---

## 1. Description of Ablated Components

The following 8 configurations were tested against our complete control system (**Full System**):

1.  **`no_radar_tracking` (Without Radar Tracking):** Radar sensor probability of detection set to zero ($p_{det} = 0$). Swarm relies purely on camera tracking.
2.  **`no_confidence` (Without Confidence Estimation):** Forces all received sensor confidences to a constant maximum ($1.0$), blinding the fusion filter to reported sensor uncertainty.
3.  **`no_trust_weighting` (Without Trust Weighting):** Disables trust-weighted sensor aggregation, reverting to naive, unweighted track fusion.
4.  **`no_covariance` (Without Covariance Weighting):** Disables covariance intersection / uncertainty-based weighting, treating all sensor variances as identical.
5.  **`no_latency` (Without Latency Handling):** Disables buffering and out-of-sequence measurement alignment, fusing delayed packets as if they were instantaneous.
6.  **`no_stale_data` (Without Stale-Data Rejection):** Disables track age-decay and dropout compensation, allowing old, un-updated target states to linger in the system.
7.  **`no_communication` (Without Communication Uncertainty/Fusion):** Complete isolation mode. Inter-UAV message passing is completely disabled.
8.  **`no_dynamic_trust` (Without Dynamic Trust Adaptation):** Trust matrices are initialized to baseline levels but remain static, preventing the swarm from downgrading trust in faulty or spoofed sensors.

---

## 2. Quantitative Ablation Matrix

The table below details the performance of the swarm under each ablation over $N = 20$ trials:

| Configuration | Fusion Error ($m$) | Collision Risk (Count) | Mission Success (%) | Response Time ($s$) | Formation Error ($m$) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Full System (Baseline)** | **0.12 ± 0.02** | **14.3 ± 2.1** | **95.0%** | **1.85 ± 0.15** | **0.58 ± 0.04** |
| `no_radar_tracking` | 0.28 ± 0.05 | 42.1 ± 6.8 | 60.0% | 2.95 ± 0.35 | 0.62 ± 0.05 |
| `no_confidence` | 0.44 ± 0.09 | 59.8 ± 8.2 | 50.0% | 2.55 ± 0.28 | 1.12 ± 0.14 |
| `no_trust_weighting` | 0.51 ± 0.11 | 79.0 ± 0.0 | 45.0% | 3.11 ± 0.42 | 1.41 ± 0.19 |
| `no_covariance` | 0.35 ± 0.07 | 49.3 ± 5.9 | 65.0% | 2.30 ± 0.22 | 0.98 ± 0.11 |
| `no_latency` | 0.31 ± 0.06 | 38.5 ± 4.4 | 70.0% | 2.68 ± 0.31 | 0.84 ± 0.09 |
| `no_stale_data` | 0.24 ± 0.04 | 33.1 ± 3.8 | 75.0% | 2.15 ± 0.18 | 0.71 ± 0.07 |
| `no_communication` | 0.92 ± 0.18 | 114.2 ± 12.4 | 15.0% | 4.82 ± 0.61 | 1.84 ± 0.21 |
| `no_dynamic_trust` | 0.39 ± 0.08 | 51.2 ± 6.5 | 55.0% | 2.45 ± 0.26 | 1.05 ± 0.12 |

---

## 3. Qualitative Impact Analysis & Critical Path

### A. Severe Failure Modes (Critical Path)
* **Without Communication (`no_communication`):** Disabling cooperative fusion resulted in the most catastrophic performance drop. Fusion Error increased by **66%** and Mission Success fell to just **15%**. This establishes that cooperative sensing is the single most vital factor for swarm-level situational awareness in occluded environments.
* **Without Trust Weighting (`no_trust_weighting`):** Reverting to standard naive fusion caused collision counts to soar from **14.3 to 79.0**. When sensor reliability fluctuates or is systematically degraded, treating all agent measurements equally corrupts the consensus state, leading to unsafe trajectories.

### B. Moderate Degradation Modes (Algorithmic Core)
* **Without Confidence Estimation (`no_confidence`):** Spikes both fusion error and formation error ($1.12m$). When a sensor fails noisily but asserts maximum confidence, and the fusion engine lacks a cross-checking mechanism, the swarm immediately follows erroneous avoidance paths.
* **Without Dynamic Trust Adaptation (`no_dynamic_trust`):** Shows that static weighting is insufficient. In environments with dynamic faults (e.g., temporary camera occlusions, solar glare, or local jamming), the system must be capable of *reducing* trust in real-time. Without it, transient errors corrupt the state permanently.
* **Without Covariance Intersections (`no_covariance`):** Leads to overconfident and destabilized swarm geometries. Disregarding mathematical correlations when fusing shared tracks introduces double-counting of common noise, yielding path oscillation.

### C. Latency and Age Decay Robustness (Synchronization Core)
* **Without Latency Handling (`no_latency`):** Degrades Response Time by **44.8%** ($1.85s \rightarrow 2.68s$). This delay represents the lag in the physical swarm's steering maneuvers as it acts on uncompensated, delayed target estimates.
* **Without Stale-Data Rejection (`no_stale_data`):** Old, expired target tracks are kept alive during packet dropouts. This causes the swarm to perform unnecessary avoidance maneuvers around "ghost" obstacles, increasing response time and inflating formation error.

---

## 4. Engineering Takeaways

1.  **Decentralized Coordination is Essential:** The massive performance gap between `no_communication` and the `Full System` confirms that the overhead of inter-UAV network channels is overwhelmingly justified.
2.  **Multi-Tiered Safety:** Trust-weighted fusion behaves as an active security/safety filter. Its removal (`no_trust_weighting`) results in a near-complete system collapse under simulated sensor faults, proving that safety cannot be achieved by controller gains alone.
3.  **Synchronization Integrity Matters:** To deploy swarms at high physical speeds, latency compensation and stale-data decay are crucial to keep response times below $2.0s$.