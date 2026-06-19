# Paper Classification: UAV Research Repository
*Last updated: 2026-06-19*

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
