Correct the format for .md file 
# Summary of Perception Uncertainty and Sensor Fusion
#      in UAV Swarms Papers

## 1: Camera-Radar Fusion with Radar Channel Extension and Dual-CBAM-FPN for Object Detection

This paper introduces a feature-level fusion network that integrates radar and camera data to overcome the misidentification issues of single sensors. By preserving low-level radar signals during feature extraction, it successfully identifies small targets that cameras alone might miss. The dual-CBAM-FPN architecture uses multi-scale attention modules to filter background clutter, establishing a strong baseline for building reliable multi-modal perception pipelines in UAV simulations.

## 2: Enabling Robots to Autonomously Search Dynamic Cluttered Post-Disaster Environments

This research pairs heuristic path planning with uncertainty-aware Tube Model Predictive Control (TMPC) to navigate dynamic environments under bounded uncertainty. It uses dynamic obstacle belts to create safe margins against phantom detections, and a continuous replanning mechanism to rapidly adjust when previously undetected obstacles appear. Applying this logic to a simulated UAV swarm elegantly models how agents survive temporary sensor failures while guaranteeing mission safety.

## 3: Advances and Challenges in Drone Detection and Classification Techniques: A State-of-the-Art Review

This review exhaustively details the failure modes of individual drone detection sensors against environmental and dynamic challenges. It argues that false readings can only be systematically reduced through heterogeneous multi-sensor fusion across radar, RF, acoustic, and camera modalities. It is highly useful for defining the theoretical error boundaries for simulation sensor models and justifying the necessity of fusion for dependable swarm operations.

## 4: Multi-Sensory Data Fusion in Terms of UAV Detection in 3D Space

The authors develop an algorithm that fuses radar, radio, and ADS-B transponder data to accurately track UAVs in 3D space using conditional complementary filtration and multi-stage clustering. By relying on complementary filtration, the system effectively ignores false positives caused by electromagnetic interference, discarding anomalous signatures that do not match known UAV profiles. This architecture is directly relevant for simulating how swarms process overlapping, noisy signals from multiple sensor streams.

## 5: Risk-Aware AI Architecture for BVLOS UAV Safety: Integrating Sensor Fusion and SATCOM

This conceptual paper introduces a three-layer Risk-Aware UAV Safety Architecture (RASA) that formally ties multi-modal perception uncertainty to communication reliability via SATCOM. It establishes an auditable AI-driven risk model that weights sensor reliability against current environmental conditions and predicts system failure when sensors degrade. This is an excellent theoretical foundation for scoring swarm safety and performance under coupled perception and communication uncertainty in dependability simulations.

## 6: A2C-LLM: An Actor-Critic-Enhanced Large Language Model for UAV Swarm Multi-Target Task Allocation

This research bridges high-level reasoning with low-level execution by using an Actor-Critic reinforcement learning framework to correct LLM-generated swarm strategies. The Critic network evaluates and filters out hallucinated or physically unfeasible tasks, while RL feedback ensures the swarm adapts to dynamic targets missed by initial planning. It offers a unique angle on handling cognitive and perception-based uncertainty at the swarm coordination level, demonstrating how swarms can safely execute complex tasks even when the initial semantic understanding of the environment is uncertain.

## 7: 3D UAV Trajectory Planning for IoT Data Collection via Matrix-Based Evolutionary Computation

This research addresses the computational cost and energy consumption limitations of existing UAV trajectory planning approaches by using matrix-based differential evolution with a constraint handle (MDE-CH). By incorporating 3D terrain constraints all UAV trajectories smoothly navigate around terrain while trajectories optimized under 3D constraints show more significant path changes near ground nodes compared to those without terrain consideration. MDE-CH outperforms both the matrix-based particle swarm optimizer (MPSO) and matrix-based genetic algorithm (MGA), making it directly applicable for defining energy-efficient and safe flight paths in our UAV systems.

## 8: UAV Collision Avoidance in Unknown Scenarios with Causal Representation Disentanglement

This research focuses on limiting the information fed to DRL models during decision-making by separating causal from non-causal image features directly applicable to multi-UAV swarm safety supervision.


## 9: Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey

Addresses the fragmented state of radar-camera fusion research by providing a unified taxonomy and survey of methods, datasets, and applications spanning automotive, drone, and robotics domains. Uses a structured literature survey methodology, classifying fusion approaches by fusion stage (data/feature/decision level) and by application domain. Shows that radar-camera fusion consistently outperforms single-sensor approaches across surveyed domains, and identifies open research gaps including swarm-scale and occlusion-heavy scenarios. As a survey rather than an experimental paper, it does not itself test swarm-density, multi-UAV occlusion, or false-alarm-rate scenarios - confirming this remains an open gap for our work to address.



## 10: Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions

Tackles the joint rise in false positives and false negatives that camera-only detectors suffer under adverse weather (fog) due to atmospheric scattering. Uses an attention-based deep fusion network combining camera and radar features so each modality compensates for the other's failure mode. Shows YOLOv5's fog-induced mis-detections and false positives are substantially reduced once fused with radar via the attention framework. Automotive, single-target, and weather-only scope leaves multi-UAV swarm density, inter-UAV occlusion, and clutter-driven false alarms unaddressed.

## 11: An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images

Targets the difficulty of detecting small, faint UAV targets in infrared imagery against complex, noisy backgrounds, where detectors both miss real targets and hallucinate clutter as targets. Uses a Bayesian CNN that generates both a segmentation map and a pixel-wise uncertainty map, using the uncertainty signal to refine final detections. Shows significant improvements in detection precision and robustness versus state-of-the-art models on three public infrared datasets. As the first uncertainty-modeling approach in this domain, it is confined to a single sensor and single target, leaving multi-UAV, multi-sensor swarm uncertainty handling unexplored.

## 12: When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in Unmanned Aerial Vehicle Safety

Investigates whether and how uncertainty in a UAV's decision-making (control signal inconsistency) predicts real flight safety violations, motivating the need for runtime safety supervision. Uses a large-scale simulation study (5,000+ PX4 flights) plus a convolutional-autoencoder-based runtime anomaly detector (Superialist) trained on control-signal data to detect decision uncertainty. Shows up to 89% of unsafe states show prior uncertainty signals, and the detector achieves up to 96% precision/93% recall in detecting uncertainty, with early warning up to 50 seconds before unsafe states. Single-UAV, single-sensor (camera-based obstacle avoidance), correlational rather than algorithmic, and with no swarm-scale or multi-sensor fusion - leaving exactly the multi-UAV, multi-sensor dependability gap our work targets.
