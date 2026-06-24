# Paper Classification: UAV Research Repository

This document categorizes all reviewed papers into their respective research tracks. Papers marked as SystemC-only, too old without replacement, or otherwise excluded from the research scope are omitted.

---

## 1. Core Radar / Anti-Drone Papers

These papers form the primary technical backbone of the anti-drone detection and classification research direction, covering mmWave/FMCW radar systems, micro-Doppler signatures, and CNN-based classification.

---

### 1.1 Detection and Identification of Non-Cooperative UAV Using a COTS mmWave Radar
- **Year:** 2024
- **Link:** https://tns.thss.tsinghua.edu.cn/sun/publications/2024.mmHawkeye_TOSN.pdf
- **Radar Type:** COTS mmWave Radar
- **Method:** mmWave radar signal processing for non-cooperative UAV identification
- **Relevance:** Core paper for mmWave-based drone detection using commercial off-the-shelf hardware.

---

### 1.2 A Lightweight CNN-Based Method for Micro-Doppler Feature-Based UAV Detection and Classification
- **Year:** 2025
- **Link:** https://www.mdpi.com/2079-9292/14/24/4831
- **Radar Type:** 8.75 GHz FMCW
- **Method:** CNN with coordinate attention mechanism, CFAR thresholding on Range-Doppler maps
- **Dataset:** RDRD dataset (simulated & experimental)
- **Result:** 98.08% average classification accuracy
- **Limitation:** Relies on clean Range-Doppler maps; untested in extreme clutter
- **Relevance:** Compact neural network for micro-Doppler classification; lightweight footprint suits real-time edge deployment. Achieving >98% accuracy in drone-vs-other discrimination makes this a benchmark reference.

---

### 1.3 High-Resolution FMCW Radar for Small UAV Detection Using GNU Software-Defined Radio
- **Year:** 2025
- **Link:** https://www.researchgate.net/publication/391792954_High-Resolution_FMCW_Radar_for_Small_UAV_Detection_Using_GNU_Software-Defined_Radio
- **Radar Type:** 2.45 GHz FMCW (SDR-based)
- **Method:** DSCR filtering, GPU-accelerated signal processing
- **Dataset:** DJI Phantom 3/4 real-world tests
- **Result:** 300 m detection range for 0.01 m² RCS targets
- **Limitation:** ISM band susceptibility to interference
- **Relevance:** Demonstrates practical low-cost SDR radar achieving 300 m detection on sub-cm² RCS drones — directly applicable to small UAV threat detection.

---

### 1.4 Small Drone Detection Using Hybrid Beamforming 24 GHz Fully Integrated CMOS Radar
- **Year:** 2025
- **Link:** https://www.mdpi.com/2504-446X/9/7/453
- **Radar Type:** 24 GHz FMCW mmWave
- **Method:** Hybrid analog beamforming, 65 nm CMOS IC
- **Dataset:** Hardware fabrication & azimuth/elevation bench tests
- **Result:** 3D tracking with <400 mW power consumption
- **Limitation:** Lower range resolution compared to 77/79 GHz mmWave systems
- **Relevance:** Hardware-level mmWave solution for distributed anti-drone arrays. Ultra-low power consumption enables scalable multi-node deployment.

---

### 1.5 LiDAR Technology for UAV Detection: From Fundamentals and Operational Principles to Advanced Detection and Classification Techniques
- **Year:** 2025
- **Link:** https://www.mdpi.com/1424-8220/25/9/2757
- **Method:** LiDAR-based remote sensing; PointPillars model trained on synthetic drone data
- **Dataset:** Synthetic drone dataset (generated via skinner.ai); PointPillars model
- **Limitation:** High computational cost; expensive hardware; sensitive to sensor calibration errors and environmental factors
- **Relevance:** Provides high-resolution 3D imaging to complement radar, enabling precise object discrimination beyond what radar or acoustic sensors alone can achieve.

---

### 1.6 Convolutional Neural Network-Based Drone Detection and Classification Using Overlaid Frequency-Modulated Continuous-Wave (FMCW) Range–Doppler Images
- **Year:** 2024
- **Link:** https://www.mdpi.com/1424-8220/24/17/5805
- **Radar Type:** FMCW
- **Method:** CNN + overlaid Range-Doppler map images; SVM and GoogLeNet (CNN) classifiers
- **Dataset:** Custom dataset collected at various distances using DEMORAD radar software
- **Result:** GoogLeNet accuracy: 96.72% (normal images) → 99.96% (overlaid RD images); SVM: 67.75% → 80.29%
- **Limitation:** High computational cost from multi-image overlay generation; no open-source dataset available
- **Relevance:** Demonstrates that overlaying multiple Range-Doppler frames dramatically boosts CNN classification accuracy (to ~99.96%), validating the overlaid RD image approach for distinguishing drones from birds and other aerial objects.

---

### 1.7 Micro-Doppler Signature Detection and Recognition of UAVs Based on OMP Algorithm
- **Year:** 2023
- **Link:** https://www.mdpi.com/1424-8220/23/18/7922
- **Method:** Micro-Doppler echo model; OMP (Orthogonal Matching Pursuit) sparse representation; clutter suppression; MATLAB 2022b simulation
- **Result:** Effective clutter suppression in complex environments with low computational complexity via sparse representation
- **Limitation:** Parameter estimation degrades at very high blade speeds due to overlapping sinusoidal waveforms in time-frequency domain
- **Relevance:** Establishes the micro-Doppler echo model pipeline — rotor speed-derived frequency differences used to extract signature parameters for UAV identification. Foundational for the classification layer.

---

### 1.8 Rotor–Body Echo Separation Using a Cyclic-Power-Guided Soft Mask from UAV Radar Signals
- **Year:** 2026
- **Link:** https://www.mdpi.com/1424-8220/26/4/1382
- **Radar Type:** FMCW
- **Method:** Cyclic-Power-Guided Soft Mask; time-frequency signal separation
- **Dataset:** Simulated and experimental hovering UAV echoes
- **Result:** Stable rotor signature separation from body clutter at 5–30 dB SNR
- **Limitation:** Primarily tested on hovering drones; fast-moving drones may require algorithmic tuning
- **Relevance:** Directly addresses a critical signal processing challenge: isolating rotor micro-Doppler from the dominant body echo. Improves ML classification accuracy at low SNR — essential for real-world deployment.

---

### 1.9 UAV Detection with Passive Radar: Algorithms, Applications, and Challenges
- **Year:** 2025 (Jan 20)
- **Link:** https://www.mdpi.com/2504-446X/9/1/76
- **Radar Type:** Passive Radar
- **Method:** Passive radar relying on ambient external electromagnetic sources (no self-emission) for target tracking
- **Dataset:** No dataset or simulation used; review/discussion paper on the state of passive radar technology for anti-drone applications
- **Result:** Passive radar offers four key advantages over active systems: no self-emitted signal, no onboard amplifiers/transmitters (lower cost), strong anti-jamming capability, and operability in complex environments using opportunistic signals
- **Limitation:** Mobile platform deployment introduces clutter Doppler shifts from platform motion, expanding clutter in the RD map and masking slow-moving or low-speed UAVs
- **Relevance:** Establishes passive radar as a cost-effective and jam-resistant anti-drone mechanism. When combined with UAV trajectory tracking, 2D image mapping, Range-Doppler imaging, and deep learning for bird-vs-drone discrimination, it becomes a highly capable detection system. Directly relevant to our anti-drone detection research.

---

### 1.10 Small UAV Target Detection Algorithm Using the YOLOv8n-RFL Based on Radar Detection Technology
- **Year:** 2025 (Aug 19)
- **Link:** https://www.mdpi.com/1424-8220/25/16/5140
- **Radar Type:** FSFM (Frequency Shift Frequency Modulation)
- **Method:** Echo signal converted to Range-Doppler (RD) planar graph; improved YOLOv8n-RFL applied to RD graph for target detection
- **Dataset:** Custom dataset of 3,900 labelled RD images (3,120 train / 390 val / 390 test); annotated using LabelImg 2023
- **Result:** Precision 87.65%, Recall 84.27%, mAP50 87.14%, F1 86.48% — outperforming all comparison models on detection accuracy, target coverage, and comprehensive performance
- **Limitation:** High computational training cost; performance may degrade at very low altitude due to environmental clutter and noise factors
- **Relevance:** Directly deployable on UAV radar systems to detect small, low-altitude drones in cluttered environments. YOLOv8n-RFL's strong mAP on RD graphs makes it a practical candidate for real-time onboard inference.

---

---

## 2. UAV Swarm Safety Support Papers

These papers support the multi-drone coordination, obstacle avoidance, navigation, and sensor-fusion aspects of the research. They are relevant to swarm safety, formation control, and integrating radar/sensor data for UAV-side situational awareness.

---

### 2.1 Drone Swarm for Distributed Video Surveillance of Roads and Car Tracking
- **Year:** 2024
- **Link:** https://www.mdpi.com/2504-446X/8/11/695
- **Relevance:** Support paper for distributed swarm-based aerial surveillance architectures.

---

### 2.2 Research on Multi-UAV Autonomous Obstacle Avoidance Algorithm Integrating Improved Dynamic Window Approach and ORCA
- **Year:** 2025
- **Link:** https://www.nature.com/articles/s41598-025-99111-8
- **Relevance:** Strong support paper combining Dynamic Window Approach with ORCA for multi-UAV autonomous collision avoidance.

---

### 2.3 Enhanced Multi-Agent Coordination Algorithm for Drone Swarm Patrolling in Durian Orchards
- **Year:** 2025
- **Link:** https://www.nature.com/articles/s41598-025-88145-7
- **Relevance:** Strong support paper for multi-agent swarm coordination in structured patrol missions.

---

### 2.4 A Transformer-Based Multimodal Adaptive Fusion System for UAV Obstacle Avoidance Integrating Photoelectric and Nano-Radar Sensors
- **Year:** 2026
- **Link:** https://doi.org/10.1166/jno.2026.3850
- **Relevance:** Very close to the target UAV support direction — multimodal sensor fusion for UAV obstacle avoidance. Quality to be verified.

---

### 2.5 Efficient Multi-Agent Deep Reinforcement Learning Algorithm for Multi-UAV Collision Avoidance
- **Year:** 2026
- **Link:** https://www.sciencedirect.com/science/article/pii/S1568494626005934
- **Relevance:** Strong support paper for deep RL-based collision avoidance in multi-UAV systems.

---

### 2.6 Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review
- **Year:** 2024
- **Link:** https://www.mdpi.com/2504-446X/8/7/320
- **Relevance:** Strong survey paper covering state-of-the-art formation control challenges and methodologies for UAV swarms.

---

### 2.7 Safe Formation Scaling and Motion Planning for Heterogeneous UAV–UGV Teams in Cluttered Environments
- **Year:** 2026
- **Link:** https://www.nature.com/articles/s41598-026-37211-9
- **Relevance:** Support paper for heterogeneous aerial-ground team coordination in obstacle-rich environments.

---

### 2.8 DPAF-SA: A Formation Control Algorithm for Dynamic Allocation and Fusion of Potential Fields for UAV Swarms
- **Year:** 2026
- **Link:** https://www.mdpi.com/2079-9292/15/2/257
- **Relevance:** Strong support paper; dynamic potential field fusion for real-time UAV swarm formation maintenance.

---

### 2.9 Synchronized Multi-Directional FMCW mmWave Radar–Inertial Odometry for Robust Positioning and Autonomous Navigation of UAVs in Low-Light Indoor Environments
- **Year:** 2026
- **Link:** https://www.mdpi.com/2504-446X/10/2/120
- **Radar Type:** FMCW mmWave (multi-directional)
- **Method:** Hybrid-RIO sensor fusion; 3-point RANSAC-LSQ; 4D radar data (range, azimuth, elevation, Doppler)
- **Result:** High positioning accuracy with superior clutter rejection in low-light indoor environments
- **Limitation:** High compute overhead for real-time 4D radar data processing
- **Relevance:** Demonstrates advanced mmWave clutter rejection for UAV navigation. Mathematical handling of false alarms directly transfers to anti-drone tracking methodologies.

---

### 2.10 Robust BEV Perception via Dual 4D Radar–Camera Fusion Under Adverse Conditions with Fog-Aware Enhancement
- **Year:** 2026
- **Link:** https://www.mdpi.com/2079-9292/15/6/1284
- **Radar Type:** 4D mmWave Radar + Camera
- **Method:** Doppler-Aware Radar Encoder (DARE); Deformable BEV Fusion; Feature Denoising
- **Dataset:** Multi-modal adverse weather / low-visibility datasets
- **Result:** Significantly higher semantic tracking accuracy in fog/night conditions vs camera-only methods
- **Limitation:** Computationally heavy BEV transformations; requires precise spatio-temporal sensor calibration
- **Relevance:** Proven architectural blueprint for radar-vision fusion under adverse conditions. DARE's Doppler-based noise rejection is directly applicable to maintaining aerial target locks when optical sensors fail.

---

### 2.11 A Learning Framework for Cooperative Collision Avoidance of UAV Swarms Leveraging Domain Knowledge
- **Year:** 2025 (Jul 15)
- **Link:** https://arxiv.org/pdf/2507.10913
- **Method:** reMARL — domain knowledge-driven cooperative MARL reward framework; Gym-like simulation environment
- **Result:** 98.75% shorter reaction time and 85.37% lower energy cost vs baseline algorithms
- **Limitation:** Scalability and energy efficiency degrade at large swarm sizes
- **Relevance:** Each drone independently maximizes reward to avoid overlapping contours with other drones, achieving smooth trajectory coordination without centralized control — key for swarm autonomy.

---

### 2.12 An Evaluation of COTS-Based Radar for Very Small Drone Sense and Avoid Application
- **Year:** 2022 (Mar 1)
- **Link:** https://www.researchgate.net/publication/361998921_An_Evaluation_of_Cots-Based_Radar_for_Very_Small_Drone_Sense_and_Avoid_Application
- **Method:** Infineon Distance2Go COTS radar; static and dynamic detection trials
- **Result:** 0.5 m detection accuracy for very small UAVs in both static and dynamic conditions
- **Limitation:** SAA development for very small drones still in early stage; high cost barriers for reliable range sensors
- **Relevance:** Demonstrates COTS radar viability for detecting acoustically quiet micro-drones by exploiting motor RF emissions — useful for intra-swarm collision avoidance at close range.

---

### 2.13 Radar–Camera Fusion in Perspective View and Bird's Eye View for 3D Object Detection
- **Year:** 2025
- **Link:** https://www.mdpi.com/1424-8220/25/19/6106
- **Radar Type:** mmWave Radar + Camera (multi-sensor)
- **Method:** Cross-modal attention mechanism; RCS/depth radar image projection; BEV mapping
- **Dataset:** Multimodal 3D object detection datasets
- **Result:** Enhanced depth estimation and accurate 3D spatial mapping
- **Limitation:** Sensitive to errors in radar-camera extrinsic calibration
- **Relevance:** Provides a fusion framework projecting mmWave RCS and depth into camera perspective, then BEV space — directly applicable to UAV swarm obstacle detection and spatial awareness.

---

## 3. Perception Uncertainty and Sensor Fusion in UAV Swarms

These papers address how UAV swarms handle unreliable, noisy, or conflicting sensor data — covering false positive/negative mitigation, uncertainty-aware planning, and multi-modal sensor fusion architectures. They are distinct from the anti-drone and swarm safety tracks.

---

### 3.1 Camera-Radar Fusion with Radar Channel Extension and Dual-CBAM-FPN for Object Detection
- **Year:** 2024
- **Authors:** X. Sun, Y. Jiang, H. Qin, J. Li, Y. Ji
- **Link:** https://doi.org/10.3390/s24165317
- **Method:** Feature-level camera-radar fusion via dual-CBAM-FPN architecture
- **Sensor/Perception Type:** Radar-Camera fusion
- **False Positive Handling:** Multi-scale feature fusion and attention modules filter background clutter
- **False Negative Handling:** Retains low-level radar features during extraction to prevent missing small targets
- **Uncertainty Type:** Perception aliasing and misidentification
- **Swarm-Level Metric:** None explicitly — focuses on object detection accuracy
- **Limitation:** Focused on static target detection rather than dynamic swarm coordination
- **Relevance:** Provides a foundational approach for improving raw perception reliability, critical before evaluating swarm-level decision-making. Establishes a strong baseline for building reliable multi-modal perception pipelines in UAV simulations.
- **Classification:** **Sensor fusion paper**

---

### 3.2 Enabling Robots to Autonomously Search Dynamic Cluttered Post-Disaster Environments
- **Year:** 2025
- **Authors:** K. Rado, M. Baglioni, A. Jamshidnejad
- **Link:** https://doi.org/10.48550/arxiv.2505.03283
- **Method:** Integrated heuristic motion planning with uncertainty-aware Tube Model Predictive Control (TMPC)
- **Sensor/Perception Type:** Generalized perception (bounded uncertainty)
- **False Positive Handling:** Aggregates predicted trajectories into obstacle belts to create safe margins against phantom detections
- **False Negative Handling:** Replanning mechanism rapidly adjusts when previously undetected dynamic obstacles appear
- **Uncertainty Type:** Bounded perception uncertainty and external disturbances
- **Swarm-Level Metric:** Mission time, target reachability, collision avoidance rates
- **Limitation:** Tested primarily on general search and rescue robots rather than high-speed aerial swarms
- **Relevance:** TMPC formulation provides a robust mathematical framework for maintaining safety despite known margins of perception error. Applying this to a simulated UAV swarm models how agents survive temporary sensor failures.
- **Classification:** **Occlusion/limited sensing paper**

---

### 3.3 Advances and Challenges in Drone Detection and Classification Techniques: A State-of-the-Art Review
- **Year:** 2023
- **Authors:** U. Seidaliyeva, L. Ilipbayeva, K. Taissariyeva, N. Smailov, E. T. Matson
- **Link:** https://doi.org/10.3390/s24010125
- **Method:** Comprehensive state-of-the-art literature review
- **Sensor/Perception Type:** Multimodal (Radar, RF, Acoustic, Camera)
- **False Positive Handling:** Evaluates how fusing disjointed data streams filters out environmental noise like birds or weather
- **False Negative Handling:** Demonstrates that multi-sensor networks mitigate the limited altitude and range coverage of single sensors
- **Uncertainty Type:** Environmental degradation and target dynamic behavior
- **Swarm-Level Metric:** Detection range and multi-target tracking scalability
- **Limitation:** Provides architectural overviews rather than specific, deployable algorithms
- **Relevance:** Definitive guide on why individual sensors fail and justifies the necessity of fusion for dependable swarm operations. Useful for defining theoretical error boundaries for simulation sensor models.
- **Classification:** **Sensor fusion paper**

---

### 3.4 Multi-Sensory Data Fusion in Terms of UAV Detection in 3D Space
- **Year:** 2022
- **Authors:** J. Dudczyk, R. Czyba, K. Skrzypczyk
- **Link:** https://doi.org/10.3390/s22124323
- **Method:** Conditional complementary filtration and multi-stage clustering
- **Sensor/Perception Type:** Radar, Radio system, ADS-B
- **False Positive Handling:** Multi-stage clustering discards anomalous signatures that do not match known UAV profiles
- **False Negative Handling:** Integrates active and passive detection to capture UAVs operating in radar blind spots
- **Uncertainty Type:** Electromagnetic interference and spatial localization errors
- **Swarm-Level Metric:** 3D spatial recognition range and multi-target classification
- **Limitation:** Lacks integration with optical/camera systems, missing out on visual validation
- **Relevance:** Validates a mathematical approach to filtering conflicted data streams in 3D space, useful for spatial uncertainty modeling. Relevant for simulating how swarms process overlapping, noisy signals.
- **Classification:** **Sensor fusion paper**

---

### 3.5 Risk-Aware AI Architecture for BVLOS UAV Safety: Integrating Sensor Fusion and SATCOM
- **Year:** 2026
- **Authors:** Nick Barua
- **Link:** https://www.preprints.org/manuscript/202604.0858
- **Method:** Three-layer Risk-Aware UAV Safety Architecture (RASA)
- **Sensor/Perception Type:** Multi-modal sensor fusion combined with SATCOM
- **False Positive Handling:** AI-driven risk model weights sensor reliability against current environmental conditions
- **False Negative Handling:** Leverages SATCOM data links to maintain situational awareness when local sensors fail
- **Uncertainty Type:** Coupled perception and communication uncertainty
- **Swarm-Level Metric:** Beyond Visual Line of Sight (BVLOS) safety coverage
- **Limitation:** Conceptual framework without extensive multi-agent simulation data
- **Relevance:** Directly models the interaction between perception uncertainty and system safety, providing a mechanism to quantify risk. Excellent theoretical foundation for scoring swarm safety and performance in dependability simulations.
- **Classification:** **UAV swarm safety paper**

---

### 3.6 A2C-LLM: An Actor-Critic-Enhanced Large Language Model for UAV Swarm Multi-Target Task Allocation
- **Year:** 2026
- **Authors:** Jie Bao, Yuping Zhang, Ronghao Zhang, Peng Zhang
- **Link:** https://doi.org/10.3390/drones10060398
- **Method:** Actor-Critic enhanced Large Language Model with a Markov Decision Process
- **Sensor/Perception Type:** Environmental state and semantic perception
- **False Positive Handling:** Critic network evaluates actions to filter out hallucinated or physically unfeasible tasks
- **False Negative Handling:** Reinforcement learning feedback ensures the swarm adapts to dynamic targets missed by initial planning
- **Uncertainty Type:** Semantic-numerical gap in dynamic environments
- **Swarm-Level Metric:** Task completion rate and robustness in adversarial scenarios
- **Limitation:** Focuses on high-level macro allocation rather than the raw physics of sensor fusion
- **Relevance:** Demonstrates how swarms can safely execute complex tasks even when the initial semantic understanding of the environment is uncertain. Unique angle on handling cognitive and perception-based uncertainty at the swarm coordination level.
- **Classification:** **Core swarm perception paper**

---

### 3.7 3D UAV Trajectory Planning for IoT Data Collection via Matrix-Based Evolutionary Computation
- **Year:** 2024 (Oct 8)
- **Authors:** Pei-Fa Sun, Yujae Song
- **Link:** https://arxiv.org/pdf/2410.05759
- **Method:** Matrix-Based Differential Evolution with Constraint Handle (MDE-CH)
- **Sensor/Perception Type:** Distributed sensors
- **False Positive Handling:** N/A
- **False Negative Handling:** N/A
- **Uncertainty Type:** N/A
- **Swarm-Level Metric:** N/A
- **Limitation:** Main focus is solving computational cost and energy consumption; prior approaches neglect 3D terrain impact
- **Relevance:** MDE-CH enables optimal UAV trajectory planning under 3D terrain constraints, reducing computational cost and energy consumption. Outperforms MPSO and MGA matrix-based optimizers. Directly applicable for defining energy-efficient flight paths in our UAV systems.
- **Classification:** **UAV swarm safety paper**

---

### 3.8 UAV Collision Avoidance in Unknown Scenarios with Causal Representation Disentanglement
- **Year:** 2024 (Dec 25)
- **Authors:** Zhun Fan, Zihao Xia et al.
- **Link:** https://www.mdpi.com/2504-446X/9/1/10
- **Method:** Deep Reinforcement Learning (DRL) with Causal Representation Disentanglement (CRD) — separates causal from non-causal image features before feeding to the DRL model
- **Sensor/Perception Type:** Onboard forward-facing camera and IMU; AirSim simulation
- **False Positive Handling:** N/A
- **False Negative Handling:** N/A
- **Uncertainty Type:** N/A
- **Swarm-Level Metric:** N/A
- **Limitation:** Environmental factors not included (simulation only); model produces longest path to destination despite collision avoidance success
- **Relevance:** Addresses DRL overfitting caused by redundant image features. CRD achieves significant improvements in navigation success rate and SPL metrics over standard CRL approaches. Useful for building generalizable collision avoidance systems that do not rely on seen environments.
- **Classification:** **UAV swarm safety paper**

---

### 3.9 Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey
- **Year:** 2024
- **Authors:** Kun Shi, Shibo He, Zhenyu Shi, Anjun Chen, Zehui Xiong, Jiming Chen, Jun Luo
- **Link:** https://arxiv.org/abs/2410.19872
- **Venue:** arXiv preprint (2410.19872)
- **Method:** Comprehensive literature survey and taxonomy of radar-camera fusion methods (data-level, feature-level, decision-level) across applications including drones/robotics
- **Sensor/Perception Type:** Radar + camera (multiple architectures surveyed across automotive, drone, and robotics domains)
- **False Positive Handling:** Surveys multiple false-positive suppression strategies used across radar-camera fusion literature (e.g., cross-modal confirmation, attention-based suppression)
- **False Negative Handling:** Surveys multiple false-negative recovery strategies (e.g., radar recovering vision misses in poor visibility, vice versa)
- **Uncertainty Type:** Surveyed range of uncertainty types across reviewed papers: sensor degradation, cross-modal association, and detection confidence uncertainty
- **Swarm-Level Metric:** None - this is a survey paper, not a method paper with its own swarm metric
- **Limitation:** As a survey rather than experimental paper, it does not propose or validate a new method itself; coverage of swarm-scale or multi-UAV fusion remains thin relative to single-target automotive fusion work
- **Relevance:** Serves as a free, open-access entry point covering the same UAV radar-camera fusion territory as paywalled conference papers (e.g., Huang et al. 2023), while also providing broader fusion-method taxonomy useful for positioning our work
- **Classification:** **Sensor fusion paper**

---

### 3.10 Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions
- **Year:** 2023
- **Authors:** Isaac Ogunrinde, Shonda Bernadin
- **Link:** https://www.mdpi.com/1424-8220/23/14/6255
- **Venue:** Sensors (MDPI), vol. 23, no. 14, article 6255
- **Method:** Attention-based deep fusion of YOLOv5 camera detections with radar features under degraded visibility
- **Sensor/Perception Type:** Camera + radar (ground vehicle context, transferable principle to UAV detection)
- **False Positive Handling:** Attention module down-weights spurious camera detections caused by scattering artifacts
- **False Negative Handling:** Radar features recover detections camera misses on small/distant objects in heavy fog
- **Uncertainty Type:** Sensor-degradation-induced uncertainty (atmospheric scattering reducing detection confidence)
- **Swarm-Level Metric:** None - single-object detection metrics (precision/recall under fog severity)
- **Limitation:** Domain is ground-vehicle/automotive, not aerial; no multi-target or swarm scenario; fog is the only degradation studied (not clutter, multipath, or UAV-specific occlusion)
- **Relevance:** Evidence for how learned fusion specifically targets false positives vs. false negatives separately - a design pattern transferable to UAV swarm detection under degraded sensing
- **Classification:** **Sensor fusion paper** + **False positive/false negative paper**

---

### 3.11 An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images
- **Year:** 2025
- **Authors:** C. Wu, W. Tang, Y. Rao, Y. Chen, H. Ding, S. Zhu, et al.
- **Link:** https://www.techscience.com/cmc/v83n1/60078/html
- **Venue:** Computers, Materials & Continua (CMC), vol. 83, no. 1, pp. 1415-1434
- **Method:** Three-stage Bayesian CNN segmentation framework producing both segmentation maps and probabilistic uncertainty maps
- **Sensor/Perception Type:** Infrared (thermal) camera, single-modality
- **False Positive Handling:** Uncertainty maps flag low-confidence background activations, suppressing spurious detections in cluttered backgrounds
- **False Negative Handling:** Uncertainty-guided refinement boosts saliency of faint/small long-range targets that plain segmentation misses
- **Uncertainty Type:** Epistemic/model uncertainty via Bayesian CNN, expressed as pixel-wise probabilistic uncertainty maps
- **Swarm-Level Metric:** None - single-target segmentation/detection accuracy metrics
- **Limitation:** First application of uncertainty modeling in infrared UAV detection; single-sensor, single-target, untested on swarm density or multi-UAV occlusion scenarios
- **Relevance:** Strong precedent for using uncertainty maps as a principled false-positive/false-negative control mechanism, extendable to multi-sensor swarm detection
- **Classification:** **False positive/false negative paper**

---

### 3.12 When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in Unmanned Aerial Vehicle Safety
- **Year:** 2025
- **Authors:** Sajad Khatiri, Fatemeh Mohammadi Amin, Sebastiano Panichella, Paolo Tonella
- **Link:** https://arxiv.org/abs/2501.08908
- **Venue:** arXiv preprint (2501.08908)
- **Method:** Large-scale empirical study correlating black-box Decision Uncertainty (control-signal inconsistency) with flight safety violations; runtime detector (Superialist) built using a convolutional autoencoder
- **Sensor/Perception Type:** Black-box control-signal monitoring (heading/waypoint data from PX4-Avoidance camera-based obstacle avoidance module); not a new perception sensor itself
- **False Positive Handling:** Mitigated by averaging reconstruction loss over multiple time windows to reduce false alarms; explicit false-positive rate of 0% reported on training data
- **False Negative Handling:** Discusses missed unsafe states: up to 11-36% of unsafe states show no significant uncertainty signal and are not caught by the detector
- **Uncertainty Type:** Behavioral/decision uncertainty (black-box, derived from control-signal inconsistency) as a proxy for underlying epistemic/aleatoric perception uncertainty
- **Swarm-Level Metric:** Uses safety/uncertainty correlation statistics (p(unsafe|uncertain) up to 74%, p(uncertain|unsafe) up to 89%) as a system-level dependability metric; not a swarm metric but the closest system-level safety metric in this set
- **Limitation:** Empirical/correlational rather than algorithmic; single-UAV, single-camera obstacle avoidance only; not focused on swarm-scale or multi-sensor perception; uncertainty-unsafety correlation is only moderate (26-50% of uncertain decisions do not lead to unsafe states)
- **Relevance:** Strong motivation and quantitative evidence for why false positive/negative and uncertainty handling matter at the safety level in a dependability lab context; useful as a framing/motivation citation and as a methodological precedent for black-box runtime monitoring
- **Classification:** **UAV swarm safety paper**

---

### 3.13 Decentralised Multi-UAV Cooperative Searching Multi-Target in Cluttered and GPS-Denied Environments
- **Year:** 2022
- **Authors:** Zhu X., Vanegas F., Gonzalez L.F.
- **Link:** https://ieeexplore.ieee.org/document/9843665
- **Venue:** IEEE Aerospace Conf. (AERO), Big Sky, Montana, Mar 2022, pp. 1-10
- **DOI:** 10.1109/AERO53065.2022.9843665
- **Method:** Decentralised cooperative search algorithm for multi-UAV systems operating in GPS-denied cluttered environments
- **Sensor/Perception Type:** Onboard sensors with local communication
- **False Positive Handling:** Cooperative validation through multi-UAV confirmation mechanisms
- **False Negative Handling:** Distributed search strategy ensures comprehensive coverage and redundant target detection
- **Uncertainty Type:** Navigation uncertainty without GPS; target localization uncertainty
- **Swarm-Level Metric:** Search coverage efficiency and target detection rate under decentralized coordination
- **Limitation:** Paywalled publication; specific sensor modalities not detailed in available abstract
- **Relevance:** Addresses foundational multi-UAV coordination challenge in denied environments with limited perception feedback
- **Classification:** **Core swarm perception paper**

---

### 3.14 Drone swarm strategy for the detection and tracking of occluded targets in complex environments
- **Year:** 2023
- **Authors:** Amala Arokia Nathan R.J., Kurmi I., Bimber O.
- **Link:** https://www.nature.com/articles/s44172-023-00104-0
- **Venue:** Communications Engineering, Nature Portfolio, Vol. 2, Art. 55, Aug 2023
- **DOI:** 10.1038/s44172-023-00104-0
- **Method:** Cooperative swarm strategy for target detection and tracking despite occlusion in complex environments
- **Sensor/Perception Type:** Camera-based swarm perception with coordinated repositioning
- **False Positive Handling:** Multi-UAV consensus validation of detections
- **False Negative Handling:** Coordinated re-observation and repositioning to overcome occlusion
- **Uncertainty Type:** Target visibility uncertainty due to environmental occlusion
- **Swarm-Level Metric:** Swarm detection and tracking success rate under occlusion conditions
- **Limitation:** Environment-specific strategies; generalization to diverse occlusion scenarios unclear
- **Relevance:** Directly addresses multi-UAV coordination for handling occlusion through cooperative perception strategies
- **Classification:** **Occlusion/limited sensing paper**

---

### 3.15 An autonomous drone swarm for detecting and tracking anomalies among dense vegetation
- **Year:** 2025
- **Authors:** Amala Arokia Nathan R.J., Strand S., Mehrwald D., Shutin D., Bimber O.
- **Link:** https://arxiv.org/abs/2407.10754
- **Venue:** Communications Engineering, Nature Portfolio, Vol. 4, Nov 2025
- **DOI:** 10.1038/s44172-025-00546-8
- **Method:** Autonomous drone swarm coordination for detecting and tracking anomalies in occluded vegetation environments
- **Sensor/Perception Type:** Camera + onboard processing with swarm coordination
- **False Positive Handling:** Swarm consensus validation of anomalies
- **False Negative Handling:** Coordinated re-observation of uncertain regions with alternate viewing angles
- **Uncertainty Type:** Occlusion from dense vegetation; limited visibility and field-of-view constraints
- **Swarm-Level Metric:** Swarm detection accuracy and coverage efficiency under vegetation occlusion
- **Limitation:** Vegetation-specific; generalization to other densely occluded environments requires further investigation
- **Relevance:** Demonstrates practical swarm-level solution to target detection and tracking under heavy occlusion
- **Classification:** **Core swarm perception paper** + **Occlusion/limited sensing paper**

---

### 3.16 Impact Study of Faulty Sensors on Flocking-Based Cooperative Control of Nonholonomic Robots
- **Year:** 2025
- **Authors:** Iftekhar L.
- **Link:** https://www.researchgate.net/publication/400928356_Impact_Study_of_Faulty_Sensors_on_Flocking-Based_Cooperative_Control_of_Nonholonomic_Robots
- **Venue:** TENCON 2025, IEEE Region 10 Conference, Oct 2025
- **DOI:** 10.1109/TENCON66050.2025.11375012
- **Method:** Empirical analysis of flocking algorithm robustness under various faulty sensor conditions
- **Sensor/Perception Type:** Sensors in flocking control loop
- **False Positive Handling:** Algorithm adaptation mechanisms to sensor error signals
- **False Negative Handling:** Reduced performance detection and recovery
- **Uncertainty Type:** Sensor fault models; measurement noise and bias
- **Swarm-Level Metric:** Flocking success rate and stability under faulty sensors
- **Limitation:** Paywalled publication; specific sensor fault types and magnitudes not fully detailed
- **Relevance:** Quantifies impact of sensor faults on fundamental swarm coordination algorithms
- **Classification:** **Faulty sensor paper**

---

### 3.17 Belief States for Cooperative Multi-Agent Reinforcement Learning under Partial Observability
- **Year:** 2025
- **Authors:** Paul J. Pritz and Kin K. Leung
- **Link:** https://arxiv.org/pdf/2504.08417
- **Venue:** Department of Computing, Imperial College London, London, United Kingdom
- **DOI:** https://doi.org/10.48550/arXiv.2504.08417
- **Method:** Belief state representation for cooperative multi-agent reinforcement learning under partial observability
- **Sensor/Perception Type:** Partial observation of environment through distributed agents
- **False Positive Handling:** Belief propagation and state estimation filters noisy observations
- **False Negative Handling:** Cooperative belief updates and information sharing recover missed observations
- **Uncertainty Type:** Epistemic uncertainty from partial observability; incomplete environmental information
- **Swarm-Level Metric:** Cooperative task completion rate under partial information conditions
- **Limitation:** Theoretical framework; real-world deployment validation and scalability testing limited
- **Relevance:** Provides principled approach to handling perception uncertainty in cooperative multi-agent systems through belief state representations
- **Classification:** **Core swarm perception paper**

---

### 3.18 The influence of limited visual sensing on the Reynolds flocking algorithm
- **Year:** 2019
- **Authors:** Enrica Soria, Fabrizio Schiano, Dario Floreano
- **Link:** https://infoscience.epfl.ch/server/api/core/bitstreams/b9588081-1c04-4fc2-8198-33e0bd75c829/content
- **Venue:** IEEE Robotics and Automation Letters (RA-L)
- **Method:** Theoretical and empirical analysis of Reynolds flocking algorithm under limited visual field constraints
- **Sensor/Perception Type:** Camera with limited field-of-view and neighbor detection constraints
- **False Positive Handling:** N/A - focus on neighbor identification accuracy under limited sensing
- **False Negative Handling:** Reduced missed neighbors through adaptive behavior and information sharing
- **Uncertainty Type:** Limited field-of-view; neighbor detection uncertainty; spatial awareness limitations
- **Swarm-Level Metric:** Swarm cohesion maintenance and coordination success rate
- **Limitation:** Limited to flocking behavior; not tested on tracking or multi-target detection tasks
- **Relevance:** Foundational analysis of how limited sensing impact swarm coordination algorithms and collective behavior
- **Classification:** **Occlusion/limited sensing paper**

---

### 3.19 VISION-BASED NEIGHBOR SELECTION METHOD FOR OCCLUSION-RESILIENT UNCREWED AERIAL VEHICLE SWARM COORDINATION IN THREE-DIMENSIONAL ENVIRONMENTS
- **Year:** 2025
- **Authors:** Oleksii Smovzhenko, Andrii Pysarenko
- **Link:** https://itvisnyk.kpi.ua/article/view/331602/327083
- **Venue:** National Technical University of Ukraine "Igor Sikorsky Kyiv Polytechnic Institute"
- **Method:** Vision-based neighbor selection algorithm resilient to occlusion in three-dimensional space
- **Sensor/Perception Type:** Camera-based vision system with spatial reasoning
- **False Positive Handling:** Selective neighbor identification avoids spurious connectivity
- **False Negative Handling:** Alternative neighbor selection pathways when direct visibility is occluded
- **Uncertainty Type:** Occlusion-induced loss of neighbor visibility; spatial uncertainty in 3D coordination
- **Swarm-Level Metric:** Swarm coordination stability and network connectivity under mutual occlusion
- **Limitation:** Vision-only approach; no integration with other sensor modalities
- **Relevance:** Direct algorithmic solution for occlusion-resilient swarm coordination maintaining connectivity in three-dimensional space
- **Classification:** **Occlusion/limited sensing paper**

---

### 3.20 Confidential-weighted cooperative merging of observations for safe navigation of UAV swarms
- **Year:** 2026
- **Authors:** Konnikov
- **Link:** https://journals.rcsi.science/2454-0714/article/view/410545/677982
- **Venue:** Software Systems and Computational Methods
- **Method:** Weighted cooperative merging of observations with confidence-based weighting for UAV swarm navigation
- **Sensor/Perception Type:** Distributed observations from multiple swarm members
- **False Positive Handling:** Low-confidence observations weighted down in fusion process
- **False Negative Handling:** Redundant observation coverage from multiple swarm members ensures target capture
- **Uncertainty Type:** Observation confidence and reliability; heterogeneous sensor quality across swarm
- **Swarm-Level Metric:** Swarm navigation safety and overall coordination robustness
- **Limitation:** Paper text in Russian; limited English language documentation and abstract detail
- **Relevance:** Directly addresses safe swarm navigation through principled confidence-weighted observation fusion mechanism
- **Classification:** **UAV swarm safety paper**

---

## Summary Table

| # | Title | Year | Category |
|---|-------|------|----------|
| 1 | Detection and Identification of Non-Cooperative UAV Using a COTS mmWave Radar | 2024 | Core Radar / Anti-Drone |
| 2 | A Lightweight CNN-Based Method for Micro-Doppler Feature-Based UAV Detection and Classification | 2025 | Core Radar / Anti-Drone |
| 3 | High-Resolution FMCW Radar for Small UAV Detection Using GNU Software-Defined Radio | 2025 | Core Radar / Anti-Drone |
| 4 | Small Drone Detection Using Hybrid Beamforming 24 GHz Fully Integrated CMOS Radar | 2025 | Core Radar / Anti-Drone |
| 5 | LiDAR Technology for UAV Detection: From Fundamentals and Operational Principles to Advanced Detection and Classification Techniques | 2025 | Core Radar / Anti-Drone |
| 6 | Convolutional Neural Network-Based Drone Detection and Classification Using Overlaid Frequency-Modulated Continuous-Wave (FMCW) Range–Doppler Images | 2024 | Core Radar / Anti-Drone |
| 7 | Micro-Doppler Signature Detection and Recognition of UAVs Based on OMP Algorithm | 2023 | Core Radar / Anti-Drone |
| 8 | Rotor–Body Echo Separation Using a Cyclic-Power-Guided Soft Mask from UAV Radar Signals | 2026 | Core Radar / Anti-Drone |
| 9 | UAV Detection with Passive Radar: Algorithms, Applications, and Challenges | 2025 | Core Radar / Anti-Drone |
| 10 | Small UAV Target Detection Algorithm Using the YOLOv8n-RFL Based on Radar Detection Technology | 2025 | Core Radar / Anti-Drone |
| 11 | Drone Swarm for Distributed Video Surveillance of Roads and Car Tracking | 2024 | UAV Swarm Safety Support |
| 12 | Research on Multi-UAV Autonomous Obstacle Avoidance Algorithm Integrating Improved Dynamic Window Approach and ORCA | 2025 | UAV Swarm Safety Support |
| 13 | Enhanced Multi-Agent Coordination Algorithm for Drone Swarm Patrolling in Durian Orchards | 2025 | UAV Swarm Safety Support |
| 14 | A Transformer-Based Multimodal Adaptive Fusion System for UAV Obstacle Avoidance Integrating Photoelectric and Nano-Radar Sensors | 2026 | UAV Swarm Safety Support |
| 15 | Efficient Multi-Agent Deep Reinforcement Learning Algorithm for Multi-UAV Collision Avoidance | 2026 | UAV Swarm Safety Support |
| 16 | Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review | 2024 | UAV Swarm Safety Support |
| 17 | Safe Formation Scaling and Motion Planning for Heterogeneous UAV–UGV Teams in Cluttered Environments | 2026 | UAV Swarm Safety Support |
| 18 | DPAF-SA: A Formation Control Algorithm for Dynamic Allocation and Fusion of Potential Fields for UAV Swarms | 2026 | UAV Swarm Safety Support |
| 19 | Synchronized Multi-Directional FMCW mmWave Radar–Inertial Odometry: Robust Positioning and Autonomous Navigation Experiments for UAVs in Low-Light Indoor Environments | 2026 | UAV Swarm Safety Support |
| 20 | Robust BEV Perception via Dual 4D Radar–Camera Fusion Under Adverse Conditions with Fog-Aware Enhancement | 2026 | UAV Swarm Safety Support |
| 21 | A Learning Framework For Cooperative Collision Avoidance of UAV Swarms Leveraging Domain Knowledge | 2025 | UAV Swarm Safety Support |
| 22 | An Evaluation of COTS-Based Radar for Very Small Drone Sense and Avoid Application | 2022 | UAV Swarm Safety Support |
| 23 | Radar–Camera Fusion in Perspective View and Bird's Eye View for 3D Object Detection | 2025 | UAV Swarm Safety Support |
| 24 | Camera-Radar Fusion with Radar Channel Extension and Dual-CBAM-FPN for Object Detection | 2024 | Sensor fusion paper |
| 25 | Enabling Robots to Autonomously Search Dynamic Cluttered Post-Disaster Environments | 2025 | Occlusion/limited sensing paper |
| 26 | Advances and Challenges in Drone Detection and Classification Techniques: A State-of-the-Art Review | 2023 | Sensor fusion paper |
| 27 | Multi-Sensory Data Fusion in Terms of UAV Detection in 3D Space | 2022 | Sensor fusion paper |
| 28 | Risk-Aware AI Architecture for BVLOS UAV Safety: Integrating Sensor Fusion and SATCOM | 2026 | UAV swarm safety paper |
| 29 | A2C-LLM: An Actor-Critic-Enhanced Large Language Model for UAV Swarm Multi-Target Task Allocation | 2026 | Core swarm perception paper |
| 30 | 3D UAV Trajectory Planning for IoT Data Collection via Matrix-Based Evolutionary Computation | 2024 | UAV swarm safety paper |
| 31 | UAV Collision Avoidance in Unknown Scenarios with Causal Representation Disentanglement | 2024 | UAV swarm safety paper |
| 32 | Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey | 2024 | Sensor fusion paper |
| 33 | Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions | 2023 | Sensor fusion paper + False positive/false negative paper |
| 34 | An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images | 2025 | False positive/false negative paper |
| 35 | When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in Unmanned Aerial Vehicle Safety | 2025 | UAV swarm safety paper |
| 36 | Decentralised Multi-UAV Cooperative Searching Multi-Target in Cluttered and GPS-Denied Environments | 2022 | Core swarm perception paper |
| 37 | Drone swarm strategy for the detection and tracking of occluded targets in complex environments | 2023 | Occlusion/limited sensing paper |
| 38 | An autonomous drone swarm for detecting and tracking anomalies among dense vegetation | 2025 | Core swarm perception paper + Occlusion/limited sensing paper |
| 39 | Impact Study of Faulty Sensors on Flocking-Based Cooperative Control of Nonholonomic Robots | 2025 | Faulty sensor paper |
| 40 | Belief States for Cooperative Multi-Agent Reinforcement Learning under Partial Observability | 2025 | Core swarm perception paper |
| 41 | The influence of limited visual sensing on the Reynolds flocking algorithm | 2019 | Occlusion/limited sensing paper |
| 42 | VISION-BASED NEIGHBOR SELECTION METHOD FOR OCCLUSION-RESILIENT UNCREWED AERIAL VEHICLE SWARM COORDINATION IN THREE-DIMENSIONAL ENVIRONMENTS | 2025 | Occlusion/limited sensing paper |
| 43 | Confidential-weighted cooperative merging of observations for safe navigation of UAV swarms | 2026 | UAV swarm safety paper |