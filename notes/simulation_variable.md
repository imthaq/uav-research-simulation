# Simulation Variables

\---

## 1.1 Detection and Identification of Non-Cooperative UAV Using a COTS mmWave Radar (mmHawkeye)

**Simulation/test type:** Real hardware evaluation with COTS mmWave radar (not a software simulation).
**Access:** Free (arXiv + university PDF).

**Input variables**

* Detection accuracy — Means: correctness of UAV detection/ID. Matters: paper reports 95.8% detection accuracy at up to 80m range. Swarm effect: Nil (single-UAV, single-radar test). Test method: real-radar field trials at varied range/settings.
* Sensor noise — Means: weak/noisy reflected signal. Matters: paper explicitly designs for low-SNR and uncertain reflected signals. Swarm effect: Nil. Test method: spectrum folding to boost SNR, tested on real hardware.
* All other requested variables (false positive rate, false negative rate, confidence score, confidence calibration error, latency, sensor dropout, occlusion level, faulty sensor behavior, fusion reliability, wrong sensor trust, overconfident wrong detection): **Nil**.

**Output metrics**

* All eight requested output metrics (collision risk, formation error, mission success, response time, unnecessary avoidance, missed response, false alarm reaction, swarm stability): **Nil** — single-UAV detection paper, no swarm or mission-outcome metric.

\---

## 1.2 A Lightweight CNN-Based Method for Micro-Doppler Feature-Based UAV Detection and Classification

**Simulation/test type:** Simulated + experimental RDRD dataset (Range-Doppler maps).
**Access:** Free (MDPI open access).

**Input variables**

* Detection accuracy — Means: classifier correctness. Matters: reports 98.08% average classification accuracy. Swarm effect: Nil. Test method: CFAR thresholding + CNN on Range-Doppler maps, evaluated on RDRD dataset.
* All other requested variables: **Nil** (paper does not report false positive/negative rate, confidence, noise, latency, dropout, occlusion, faulty sensor, fusion, trust, or overconfidence by these names).

**Output metrics**

* All eight requested output metrics: **Nil** — single-target classification paper, not swarm.

\---

## 1.3 High-Resolution FMCW Radar for Small UAV Detection Using GNU Software-Defined Radio

**Simulation/test type:** Real-world hardware test (DJI Phantom 3/4).
**Access:** Could not access free full text — only ResearchGate abstract page found; full PDF behind access wall. Noting issue and moving on.

**Input variables**

* Detection accuracy — Means: range/detectability of small RCS targets. Matters: reports 300m detection range for 0.01 m² RCS targets. Swarm effect: Nil. Test method: real-world DSCR filtering + GPU signal processing on DJI Phantom test flights.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 1.4 Small Drone Detection Using Hybrid Beamforming 24 GHz Fully Integrated CMOS Radar

**Simulation/test type:** Hardware fabrication + bench test (azimuth/elevation).
**Access:** Free (MDPI open access).

**Input variables**

* Detection accuracy — Means: 3D tracking capability. Matters: reports 3D tracking with <400 mW power consumption. Swarm effect: Nil. Test method: azimuth/elevation bench tests on fabricated 65nm CMOS IC.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — single-radar hardware test, not swarm.

\---

## 1.6 CNN-Based Drone Detection Using Overlaid FMCW Range–Doppler Images

**Simulation/test type:** Custom real-world dataset (DEMORAD radar software).
**Access:** Free (MDPI open access).

**Input variables**

* Detection accuracy — Means: classifier correctness distinguishing drones from birds/objects. Matters: GoogLeNet accuracy 96.72% (normal images) → 99.96% (overlaid RD images); SVM 67.75% → 80.29%. Swarm effect: Nil. Test method: custom dataset collected at varied distances, compared across SVM and GoogLeNet classifiers.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 1.7 Micro-Doppler Signature Detection and Recognition of UAVs Based on OMP Algorithm

**Simulation/test type:** MATLAB 2022b simulation.
**Access:** Free (MDPI open access).

**Input variables**

* Sensor noise — Means: clutter mixed with rotor micro-Doppler signal. Matters: paper reports effective clutter suppression in complex environments with low computational complexity. Swarm effect: Nil. Test method: OMP sparse representation tested in MATLAB simulation.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 1.8 Rotor–Body Echo Separation Using a Cyclic-Power-Guided Soft Mask from UAV Radar Signals

**Simulation/test type:** Simulated + experimental hovering UAV echoes.
**Access:** Free (MDPI open access).

**Input variables**

* Sensor noise — Means: body-clutter mixed with rotor signature. Matters: paper reports stable rotor signature separation from body clutter at 5–30 dB SNR. Swarm effect: Nil. Test method: simulated and experimental hovering UAV echo tests across SNR range.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 1.10 Small UAV Target Detection Algorithm Using YOLOv8n-RFL Based on Radar Detection Technology

**Simulation/test type:** Custom labelled Range-Doppler image dataset.
**Access:** Free (MDPI open access).

**Input variables**

* Detection accuracy — Means: correctness of small-target detection. Matters: reports Precision 87.65%, Recall 84.27%, mAP50 87.14%, F1 86.48%. Swarm effect: Nil. Test method: custom dataset of 3,900 labelled RD images, train/val/test split, compared against baseline models.
* False negative rate (implied by Recall) — Means: missed detections. Matters: Recall of 84.27% implies \~15.7% miss rate at low altitude per paper's own limitation note. Swarm effect: Nil. Test method: same labelled dataset evaluation.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — single-target detection paper.

\---

## 2.9 Synchronized Multi-Directional FMCW mmWave Radar–Inertial Odometry for UAVs in Low-Light Indoor Environments

**Simulation/test type:** Stated experimental result ("high positioning accuracy... clutter rejection") but classification doc gives no dataset/method detail beyond a one-line summary.
**Access:** Could not verify — not independently searched; relying only on classification doc's own claim, which does not specify whether this is simulation or hardware. Noting this gap and moving on.

**Input variables**

* Sensor noise — Means: clutter in 4D radar data. Matters: classification doc states "superior clutter rejection in low-light indoor environments." Swarm effect: Nil. Test method: Nil (not detailed in source).
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 2.10 Robust BEV Perception via Dual 4D Radar–Camera Fusion Under Adverse Conditions with Fog-Aware Enhancement

**Simulation/test type:** Multi-modal adverse-weather/low-visibility dataset (per classification doc).
**Access:** Not independently verified — relying on classification doc summary only. Noting this gap and moving on.

**Input variables**

* Detection accuracy — Means: semantic tracking correctness. Matters: classification doc states "significantly higher semantic tracking accuracy in fog/night conditions vs camera-only methods." Swarm effect: Nil. Test method: Nil (exact dataset/metric not detailed in source).
* Sensor noise — Means: fog/low-light degradation. Matters: motivates the "Fog-Aware Enhancement" design. Swarm effect: Nil. Test method: Nil.
* Fusion reliability — Means: radar+camera fusion vs. camera-only. Matters: explicitly compared against camera-only baseline per classification doc. Swarm effect: Nil. Test method: Nil (not detailed beyond comparison claim).
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 2.11 A Learning Framework for Cooperative Collision Avoidance of UAV Swarms Leveraging Domain Knowledge (reMARL)

**Simulation/test type:** Gym-like multi-agent reinforcement learning simulation environment.
**Access:** Free (arXiv).

**Input variables**

* All thirteen requested input variables (false positive rate, false negative rate, detection accuracy, confidence score, confidence calibration error, sensor noise, latency, sensor dropout, occlusion level, faulty sensor behavior, fusion reliability, wrong sensor trust, overconfident wrong detection): **Nil** — paper is about reward-shaping for collision avoidance, not perception/sensor variables.

**Output metrics**

* Collision risk — Means: drone-drone overlap/collision. Matters: reMARL's reward design directly targets avoiding overlapping contours with other drones. Swarm effect: this is the swarm output measured. Test method: Gym-like simulation comparing reMARL against baseline algorithms.
* Response time — Means: time for swarm to react/coordinate. Matters: reports 98.75% shorter reaction time vs. baseline. Swarm effect: faster reaction reduces collision window. Test method: timed comparison in simulation against baseline MARL algorithms.
* Mission success (framed as smooth trajectory coordination) — Means: completing coordinated movement without centralized control. Matters: paper states each drone independently maximizes reward to avoid overlap, achieving smooth trajectory coordination. Swarm effect: this is the outcome measured. Test method: simulated multi-drone trials, scalability noted to degrade at large swarm sizes.
* All other requested output metrics (formation error, unnecessary avoidance, missed response, false alarm reaction, swarm stability): **Nil**.

\---

## 2.12 An Evaluation of COTS-Based Radar for Very Small Drone Sense and Avoid Application

**Simulation/test type:** Real hardware trials (Infineon Distance2Go radar), static and dynamic.
**Access:** Could not access free full text — ResearchGate abstract page only. Noting issue and moving on.

**Input variables**

* Detection accuracy — Means: positional accuracy for very small UAV detection. Matters: reports 0.5m detection accuracy in both static and dynamic conditions. Swarm effect: Nil (single-radar test, not swarm). Test method: static and dynamic detection trials with COTS radar.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 2.13 Radar–Camera Fusion in Perspective View and Bird's Eye View for 3D Object Detection

**Simulation/test type:** Multimodal 3D object detection dataset (per classification doc; not independently verified).
**Access:** Not independently verified — relying on classification doc summary only. Noting this gap and moving on.

**Input variables**

* Detection accuracy — Means: 3D spatial detection correctness. Matters: classification doc states "enhanced depth estimation and accurate 3D spatial mapping." Swarm effect: Nil. Test method: Nil (not detailed beyond summary).
* Fusion reliability — Means: radar RCS/depth fused with camera. Matters: cross-modal attention mechanism projects radar into camera perspective, then BEV space. Swarm effect: Nil. Test method: Nil (exact dataset/metric not detailed in source).
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil**.

\---

## 3.1 Camera-Radar Fusion with Radar Channel Extension and Dual-CBAM-FPN for Object Detection

**Simulation/test type:** Stated method (feature-level fusion architecture); dataset not detailed in classification doc.
**Access:** Free (MDPI/Sensors, DOI 10.3390/s24165317), not independently re-verified beyond classification doc's own description. Noting this and moving on.

**Input variables**

* False positive rate — Means: background clutter misclassified as object. Matters: classification doc states multi-scale feature fusion and attention modules filter background clutter. Swarm effect: Nil. Test method: Nil (not detailed beyond architecture description).
* False negative rate — Means: missed small targets. Matters: paper retains low-level radar features during extraction specifically to prevent missing small targets. Swarm effect: Nil. Test method: Nil.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — explicitly stated as focused on static object detection, not swarm coordination.

\---

## 3.2 Enabling Robots to Autonomously Search Dynamic Cluttered Post-Disaster Environments

**Simulation/test type:** Stated method (Tube Model Predictive Control); test platform is general search/rescue robots, not aerial swarm specifically.
**Access:** Free (arXiv).

**Input variables**

* False positive rate — Means: phantom (non-existent) obstacle detections. Matters: classification doc states the method aggregates predicted trajectories into obstacle belts to create safe margins against phantom detections. Swarm effect: Nil (tested on general robots, not UAV swarm). Test method: Nil beyond this statement.
* False negative rate — Means: previously undetected dynamic obstacles appearing. Matters: replanning mechanism rapidly adjusts when such obstacles appear. Swarm effect: Nil. Test method: Nil.
* Sensor noise (framed as "bounded perception uncertainty") — Means: known margin of error in perception. Matters: TMPC formulation maintains safety despite known margins of perception error. Swarm effect: Nil. Test method: Nil beyond TMPC framework description.

**Output metrics**

* Mission success — Means: target reachability without failure. Matters: classification doc lists "mission time, target reachability, collision avoidance rates" as reported metrics. Swarm effect: Nil (general robots, not aerial swarm). Test method: Nil (exact values not given in classification doc).
* Collision risk — Means: collision avoidance performance. Matters: same source as above — collision avoidance rates explicitly listed as a measured outcome. Swarm effect: Nil. Test method: Nil (no numeric value given).
* Response time (framed as "mission time") — Means: time to complete search/reach target. Matters: same source — mission time explicitly listed. Swarm effect: Nil. Test method: Nil.
* All other requested output metrics (formation error, unnecessary avoidance, missed response, false alarm reaction, swarm stability): **Nil**.

\---

## 3.4 Multi-Sensory Data Fusion in Terms of UAV Detection in 3D Space

**Simulation/test type:** Stated method (conditional complementary filtration, multi-stage clustering); dataset not detailed.
**Access:** Free (MDPI/Sensors, DOI 10.3390/s22124323), not independently re-verified beyond classification doc. Noting this and moving on.

**Input variables**

* False positive rate — Means: anomalous signatures misclassified as UAV. Matters: multi-stage clustering discards anomalous signatures that do not match known UAV profiles. Swarm effect: Nil. Test method: Nil beyond this statement.
* False negative rate — Means: UAVs in radar blind spots going undetected. Matters: integrates active and passive detection to capture UAVs in radar blind spots. Swarm effect: Nil. Test method: Nil.
* All other requested variables: **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — single-target spatial recognition paper, no swarm metric reported.

\---

## 3.10 Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions

**Simulation/test type:** CARLA simulator (clear + fog driving scenes).
**Access:** Free (MDPI/PMC open access).

**Input variables**

* False positive rate — Means: spurious detections from fog scattering. Matters: YOLOv5 is affected by mis-detections and false positives due to atmospheric scattering caused by fog particles. Swarm effect: Nil (ground-vehicle, single-object study). Test method: attention module down-weights spurious camera detections; tested on CARLA clear+fog datasets.
* False negative rate — Means: small/distant objects missed by camera in fog. Matters: radar features recover detections camera misses on small/distant objects in heavy fog. Swarm effect: Nil. Test method: same CARLA dataset, radar-camera fusion comparison.
* Detection accuracy — Means: overall detector correctness. Matters: small CR-YOLOnet model achieves accuracy of 0.849 at 69 fps; fusion gave 24.19% mAP improvement over plain YOLOv5 (0.765 mAP). Swarm effect: Nil. Test method: trained/tested on clear vs. fog CARLA datasets.
* Sensor noise — Means: SNR degradation under fog. Matters: signal-to-noise ratio (SNR) is reduced, while measurement noise rises dramatically under foggy conditions. Swarm effect: Nil. Test method: stated as motivation, not separately swept.
* Latency — Means: detection speed. Matters: small CR-YOLOnet ran at 69 fps; medium variant achieved 72 FPS. Swarm effect: Nil. Test method: measured directly as fps during evaluation.
* Fusion reliability — Means: radar+camera fusion vs. camera-only baseline. Matters: proposed method outperformed YOLOv5+DeepSORT with 35.15% increase in tracking accuracy, 32.65% increase in precision, 37.56% faster, 46.81% fewer identity switches. Swarm effect: Nil. Test method: direct comparison against YOLOv5+DeepSORT baseline.
* All other requested variables (confidence score, confidence calibration error, sensor dropout, occlusion level, faulty sensor behavior, wrong sensor trust, overconfident wrong detection): **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — single ground-vehicle detection study, no swarm or mission metric.

\---

## 3.11 An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images

**Simulation/test type:** Three public infrared UAV datasets.
**Access:** Free (CMC/Tech Science Press open access).

**Input variables**

* False negative rate — Means: missed small/distant IR targets. Matters: infrared UAV target detection presents significant challenges due to the interplay between small targets and complex backgrounds; traditional methods often fail in scenarios involving long-range targets, high noise levels, or intricate backgrounds. Swarm effect: Nil. Test method: evaluated on three public IR UAV datasets.
* False positive rate — Means: background clutter misclassified as target. Matters: uncertainty maps flag low-confidence background activations, suppressing spurious detections in cluttered backgrounds. Swarm effect: Nil. Test method: same three-dataset evaluation.
* Detection accuracy — Means: segmentation/detection correctness. Matters: results reveal significant improvements in detection precision and robustness vs. state-of-the-art models. Swarm effect: Nil. Test method: comparison against state-of-the-art models on the three datasets.
* Confidence calibration error (framed as probabilistic uncertainty maps) — Means: model's pixel-wise uncertainty about its own output. Matters: Bayesian CNN generates both segmentation maps and probabilistic uncertainty maps; uncertainty predictions refine segmentation outcomes. Swarm effect: Nil. Test method: uncertainty maps generated and used to refine segmentation, evaluated on the three datasets.
* All other requested variables (confidence score, sensor noise, latency, sensor dropout, occlusion level, faulty sensor behavior, fusion reliability, wrong sensor trust, overconfident wrong detection): **Nil**.

**Output metrics**

* All eight requested output metrics: **Nil** — single-image segmentation paper, explicitly untested on swarm density or multi-UAV scenarios per its own limitation note.

\---

## 3.12 When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in UAV Safety

**Simulation/test type:** \~5,000 simulated PX4-Autopilot/PX4-Avoidance flights via Surrealist (Gazebo).
**Access:** Free (arXiv + Springer).

**Input variables**

* Confidence calibration error (framed as Decision Uncertainty) — Means: inconsistency in control-signal output used as black-box uncertainty proxy. Matters: Decision Uncertainty is obtained by analyzing the inconsistencies in the control signals of the autonomous system, which could lead to unstable, erratic, unpredictable behavior. Swarm effect: Nil (single-UAV). Test method: Surrealist-generated flight dataset, manual analysis of a subset to correlate uncertainty with safety violations.
* Overconfident wrong detection — Means: system behaves consistently while actually unsafe. Matters: up to 89% of unsafe UAV states exhibit significant decision uncertainty, and up to 74% of uncertain decisions lead to unsafe states. Swarm effect: Nil. Test method: same Surrealist dataset, correlation analysis between uncertainty and unsafety labels.
* False positive rate — Means: false safety alarms in the runtime detector. Matters: classification doc states an explicit false-positive rate of 0% reported on training data for the Superialist detector. Swarm effect: Nil. Test method: autoencoder reconstruction-loss averaging over multiple time windows.
* False negative rate — Means: missed unsafe states. Matters: up to 11–36% of unsafe states show no significant uncertainty signal and are not caught by the detector. Swarm effect: Nil. Test method: same correlation analysis as above.
* All other requested variables (detection accuracy, confidence score, sensor noise, latency, sensor dropout, occlusion level, faulty sensor behavior, fusion reliability, wrong sensor trust): **Nil**.

**Output metrics**

* Collision risk (framed as distance-to-obstacle) — Means: how close the UAV came to an obstacle. Matters: Surrealist's search process aims to minimize a predefined distance measure (e.g., minimum distance between drone and obstacles). Swarm effect: Nil (single-UAV). Test method: iteratively placing static obstacles to drive the drone toward unsafe paths.
* Mission success (framed as safe vs. unsafe flight classification) — Means: whether the flight completed without a safety violation. Matters: dataset includes four flight conditions — safe, unsafe, certain, and uncertain. Swarm effect: Nil. Test method: labeling \~5,000 simulated flights along these four categories.
* All other requested output metrics (formation error, response time, unnecessary avoidance, missed response, false alarm reaction, swarm stability): **Nil** — explicitly single-UAV, not swarm-scale, per the paper's own limitation note.

\---

## 3.14 Drone Swarm Strategy for the Detection and Tracking of Occluded Targets in Complex Environments

**Simulation/test type:** Stated cooperative swarm strategy; classification doc gives no explicit dataset/simulation platform detail.
**Access:** Free (Nature/Communications Engineering open access), not independently re-verified beyond classification doc's own description. Noting this and moving on.

**Input variables**

* False positive rate — Means: incorrect target detections. Matters: classification doc states multi-UAV consensus validation of detections is used to handle this. Swarm effect: Nil. Test method: Nil (not detailed beyond consensus-validation description).
* False negative rate — Means: missed targets due to occlusion. Matters: coordinated re-observation and repositioning to overcome occlusion. Swarm effect: this is the central mechanism the swarm uses to recover from misses. Test method: Nil (no quantitative detail given in classification doc).
* Occlusion level — Means: target visibility blocked by environment. Matters: this is the paper's core subject — target visibility uncertainty due to environmental occlusion. Swarm effect: swarm coordinates repositioning specifically to counter this. Test method: Nil (exact simulation/test setup not detailed beyond "complex environments").

**Output metrics**

* Swarm stability (framed as detection/tracking success rate under occlusion) — Means: whether the swarm maintains successful tracking despite occlusion. Matters: classification doc lists "swarm detection and tracking success rate under occlusion conditions" as the reported metric. Swarm effect: this is the outcome measured. Test method: Nil (no numeric value given in classification doc).
* All other requested output metrics (collision risk, formation error, mission success, response time, unnecessary avoidance, missed response, false alarm reaction): **Nil**.

\---

## 3.15 An Autonomous Drone Swarm for Detecting and Tracking Anomalies Among Dense Vegetation

**Simulation/test type:** Stated swarm coordination method; classification doc gives no explicit dataset/simulation platform detail.
**Access:** Free (arXiv + Nature/Communications Engineering), not independently re-verified beyond classification doc's own description. Noting this and moving on.

**Input variables**

* False positive rate — Means: incorrect anomaly flags. Matters: classification doc states swarm consensus validation of anomalies is used. Swarm effect: Nil. Test method: Nil (not detailed beyond consensus-validation description).
* False negative rate — Means: missed anomalies in occluded vegetation. Matters: coordinated re-observation of uncertain regions with alternate viewing angles. Swarm effect: this is the swarm's stated recovery mechanism. Test method: Nil (no quantitative detail given).
* Occlusion level — Means: dense vegetation blocking visibility. Matters: this is the paper's core subject — occlusion from dense vegetation, limited visibility, field-of-view constraints. Swarm effect: drives the swarm's re-observation strategy. Test method: Nil (exact simulation/test setup not detailed beyond "dense vegetation").

**Output metrics**

* Swarm stability (framed as detection accuracy and coverage efficiency under occlusion) — Means: how well the swarm maintains detection/coverage despite occlusion. Matters: classification doc lists "swarm detection accuracy and coverage efficiency under vegetation occlusion" as the reported metric. Swarm effect: this is the outcome measured. Test method: Nil (no numeric value given in classification doc).
* All other requested output metrics (collision risk, formation error, mission success, response time, unnecessary avoidance, missed response, false alarm reaction): **Nil**.

\---

## 3.18 The Influence of Limited Visual Sensing on the Reynolds Flocking Algorithm

**Simulation/test type:** Monte-Carlo simulations + genetic algorithm optimization.
**Access:** Paywalled (IEEE Xplore) — abstract only available. Noting this issue and moving on; all entries below are name-only, not verified method/formula.

**Input variables**

* Occlusion level (framed as field-of-view / sensor orientation limits) — Means: reduction in field of view and orientation of visual sensors. Matters: paper studies how this affects Reynolds flocking algorithm performance. Swarm effect: confirmed lateral vision is essential for coordinating movements. Test method: extensive Monte-Carlo simulations integrated with genetic algorithm optimization; exact procedure beyond this: Nil (full text inaccessible).
* All other requested variables: **Nil**.

**Output metrics**

* Swarm stability (named "order" in the paper) — Means: Nil, exact definition unavailable. Matters: introduced as one of four metrics quantifying impact of limited visual sensing. Swarm effect: this is the metric itself. Test method: Nil (formula/procedure not in abstract).
* Collision risk (named "safety" in the paper) — Means: Nil, exact definition unavailable. Matters: same source, one of the four named metrics. Swarm effect: this is the metric itself. Test method: Nil.
* Formation error (named "union" in the paper) — Means: Nil, exact definition unavailable. Matters: same source. Swarm effect: this is the metric itself. Test method: Nil.
* A fourth metric "connectivity" is also named with no further detail: Nil.
* All other requested output metrics (mission success, response time, unnecessary avoidance, missed response, false alarm reaction): **Nil**.

\---

## 3.19 Vision-Based Neighbor Selection Method for Occlusion-Resilient UAV Swarm Coordination in 3D Environments

**Simulation/test type:** Point-mass Python environment + Gazebo Classic physics-based simulation.
**Access:** Free (open-access journal).

**Input variables**

* Occlusion level — Means: visual occlusions between swarm agents. Matters: study addresses the problem of visual occlusions that disrupt decentralized flocking. Swarm effect: degrades neighbor visibility, the core problem solved. Test method: tested across swarm sizes (up to 50 agents and beyond) using three neighbor-selection strategies — metric, topographic, and Delaunay.
* All other requested variables: **Nil**.

**Output metrics**

* Swarm stability (framed as "alignment") — Means: degree of heading/velocity agreement across the swarm. Matters: topographic selection achieves high alignment (above 0.9) in small swarms (up to 50 agents), while Delaunay ensures robust alignment across all swarm sizes. Swarm effect: this is the outcome measured. Test method: simulation across swarm sizes, comparing strategies; also against communication-enabled baseline (alignment above 0.9 vs. 0.85 in small swarms).
* Formation error (framed as "cohesion"/"union") — Means: whether the swarm stays as one connected group. Matters: Delaunay ensures perfect cohesion (union = 1) across all swarm sizes. Swarm effect: this is the outcome measured. Test method: same simulation, union metric computed per strategy per swarm size.
* All other requested output metrics (collision risk, mission success, response time, unnecessary avoidance, missed response, false alarm reaction): **Nil**.

\---

## Papers excluded — no simulation/experiment/dataset confirmed

* **1.5** LiDAR Technology for UAV Detection — review-style paper covering fundamentals; "synthetic drone data generated via skinner.ai" is mentioned but no result/metric stated. Borderline; excluded for lack of reported result.
* **1.9** UAV Detection with Passive Radar — classification doc explicitly states "No dataset or simulation used; review/discussion paper."
* **2.1–2.8** Drone swarm surveillance, obstacle avoidance (DWA+ORCA), orchard patrolling, transformer fusion, multi-agent DRL, formation control review, heterogeneous UAV-UGV teams, DPAF-SA — classification doc gives only a one-line relevance note for each, no stated method/dataset/result confirming an actual simulation was run. Excluded for insufficient evidence.
* **3.3** Advances and Challenges in Drone Detection and Classification — explicitly a "comprehensive state-of-the-art literature review," no experiment of its own.
* **3.5** Risk-Aware AI Architecture for BVLOS UAV Safety — explicitly stated as "conceptual framework without extensive multi-agent simulation data."
* **3.6** A2C-LLM — method stated (Actor-Critic LLM) but no dataset, simulation platform, or numeric result given in classification doc.
* **3.7** 3D UAV Trajectory Planning — method (MDE-CH) stated with comparison to MPSO/MGA, but classification doc gives no simulation platform or numeric metric for sensor/swarm variables; excluded as it is a pure optimization/planning paper unrelated to the requested perception variables.
* **3.8** UAV Collision Avoidance with Causal Representation Disentanglement — uses AirSim simulation, but classification doc states "False Positive/Negative/Uncertainty/Swarm-Level Metric: N/A" for all relevant fields; excluded for lack of reported variable data despite having a simulation platform.
* **3.9** Radar and Camera Fusion — Comprehensive Survey — explicitly a survey, no experiment of its own.
* **3.13** Decentralised Multi-UAV Cooperative Searching — paywalled, classification doc itself states "specific sensor modalities not detailed in available abstract"; excluded for insufficient access.
* **3.16** Impact Study of Faulty Sensors on Flocking-Based Cooperative Control — paywalled (IEEE TENCON 2025); could not access full text. Classification doc itself states "specific sensor fault types and magnitudes not fully detailed." Noting this access issue — this paper is the most directly relevant to "faulty sensor behavior" in the entire repository, but cannot be filled in without the full text.
* **3.17** Belief States for Cooperative MARL under Partial Observability — theoretical framework; classification doc states "real-world deployment validation and scalability testing limited," and gives no simulation dataset/result. Excluded for lack of confirmed simulation.
* **3.20** Confidential-weighted cooperative merging of observations — paper text in Russian per classification doc; limited English documentation. Excluded for access/language issue.

