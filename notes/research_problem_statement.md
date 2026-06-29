# Research Problem Statement

---

## The Perception Baseline

**Papers:**
- 1.1 Detection and Identification of Non-Cooperative UAV Using a COTS mmWave Radar (2024)
- 1.2 A Lightweight CNN-Based Method for Micro-Doppler Feature-Based UAV Detection and Classification (2025)
- 1.3 High-Resolution FMCW Radar for Small UAV Detection Using GNU Software-Defined Radio (2025)
- 1.4 Small Drone Detection Using Hybrid Beamforming 24 GHz Fully Integrated CMOS Radar (2025)
- 1.5 LiDAR Technology for UAV Detection: From Fundamentals and Operational Principles to Advanced Detection and Classification Techniques (2025)
- 1.6 Convolutional Neural Network-Based Drone Detection and Classification Using Overlaid FMCW Range-Doppler Images (2024)
- 1.7 Micro-Doppler Signature Detection and Recognition of UAVs Based on OMP Algorithm (2023)
- 1.8 Rotor-Body Echo Separation Using a Cyclic-Power-Guided Soft Mask from UAV Radar Signals (2026)
- 1.9 UAV Detection with Passive Radar: Algorithms, Applications, and Challenges (2025)
- 1.10 Small UAV Target Detection Algorithm Using the YOLOv8n-RFL Based on Radar Detection Technology (2025)
- 3.3 Advances and Challenges in Drone Detection and Classification Techniques: A State-of-the-Art Review (2023)
- 3.9 Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey (2024)

---

### Q1. What problem are we studying?

These papers show where single-sensor UAV detection currently stands. They cover COTS mmWave radar, FMCW SDR-based systems, 24 GHz beamforming CMOS radar, passive radar, LiDAR with PointPillars, CNN classifiers on Range-Doppler maps, micro-Doppler OMP classification, rotor-body echo separation, YOLO on RD graphs, and two broad surveys. Together they show that detection technology has improved a lot: classification accuracy goes above 98% under clean conditions, detection range reaches 300 m on very small targets, and lightweight models can run in real time. The issue is that none of these systems has been tested inside a live multi-UAV swarm. All benchmarks use clean datasets, controlled settings, or single targets. So the question becomes: if these systems work well on their own, what happens to swarm behavior when they stop working well, for example when clutter, low signal, interference, or environmental noise pushes them into failure?

### Q2. Why do false positives and false negatives matter in UAV swarms?

These papers document exactly where each sensor fails. CNN classifiers that depend on clean Range-Doppler maps produce wrong detections in heavy clutter, which look like phantom obstacles to the swarm. Passive radar loses slow-moving targets because platform motion spreads clutter across the map. YOLO on RD graphs misses targets at very low altitude. Rotor-body separation methods tuned for hovering drones break on fast-moving ones. LiDAR becomes unreliable when calibration drifts. In single-UAV settings, each failure is manageable on its own. In a swarm, a phantom detection (false positive) forces a drone to swerve unnecessarily, breaking formation and pushing into neighbors. A missed detection (false negative) removes an obstacle from the swarm's picture entirely, with collision as the direct outcome.

### Q3. Why does sensor fusion reliability matter?

Every sensor in this group has a known failure condition. mmWave radar is hit by ISM-band interference. Passive FMCW systems suffer from clutter caused by platform movement. LiDAR is expensive and sensitive to calibration error. CNN classifiers break under data it was not trained on. The two surveys in this group make the point that no single sensor handles all conditions, which is why fusion is needed. But fusion creates a new problem: if a degraded sensor is mixed with a healthy one without adjusting for quality, the merged output carries the error forward. These papers set the starting conditions for our study, a picture of what detection looks like when it is working well and where it starts to break down.

### Q4. Why is this important for safety?

These papers show that detection systems intended for real swarm use are tested on the best-case versions of their inputs. Networks that achieve over 98% accuracy on curated datasets have no published results in cluttered, multi-target, close-formation swarm conditions. OMP classification degrades at high rotor speeds. Passive radar loses slow UAVs in clutter. The gap between what accuracy these systems achieve in benchmarks and what accuracy they achieve in real swarm conditions is unknown. Safety cannot be assumed from benchmark numbers alone. We need to know what happens to swarm behavior when accuracy drops.

### Q5. Why is simulation suitable for this study?

These papers provide the numbers needed to build realistic fault models in simulation: detection ranges, SNR thresholds, clutter rejection bounds, and the conditions where each sensor starts to fail. In simulation, we can push a CNN classifier into its documented clutter failure mode, put a passive radar into its Doppler-clutter problem, and watch what the swarm does next without putting any hardware or flights at risk. The benchmark results from these papers define what normal operation looks like, so deviations caused by injected faults are easy to measure.

---

## Swarm Coordination Norms

**Papers:**
- 2.1 Drone Swarm for Distributed Video Surveillance of Roads and Car Tracking (2024)
- 2.2 Research on Multi-UAV Autonomous Obstacle Avoidance Algorithm Integrating Improved Dynamic Window Approach and ORCA (2025)
- 2.3 Enhanced Multi-Agent Coordination Algorithm for Drone Swarm Patrolling in Durian Orchards (2025)
- 2.4 A Transformer-Based Multimodal Adaptive Fusion System for UAV Obstacle Avoidance Integrating Photoelectric and Nano-Radar Sensors (2026)
- 2.5 Efficient Multi-Agent Deep Reinforcement Learning Algorithm for Multi-UAV Collision Avoidance (2026)
- 2.6 Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review (2024)
- 2.7 Safe Formation Scaling and Motion Planning for Heterogeneous UAV-UGV Teams in Cluttered Environments (2026)
- 2.8 DPAF-SA: A Formation Control Algorithm for Dynamic Allocation and Fusion of Potential Fields for UAV Swarms (2026)
- 3.7 3D UAV Trajectory Planning for IoT Data Collection via Matrix-Based Evolutionary Computation (2024)
- 3.13 Decentralised Multi-UAV Cooperative Searching Multi-Target in Cluttered and GPS-Denied Environments (2022)

---

### Q1. What problem are we studying?

These papers show what swarm coordination algorithms can do today. They cover ORCA and Dynamic Window Approach combinations, deep reinforcement learning for multi-agent collision avoidance, potential-field formation control, formation scaling for mixed aerial-ground teams, evolutionary trajectory planning, and decentralised search in GPS-denied environments. The overall picture is that swarm coordination has become quite capable. But all of it assumes the sensor data coming in is trustworthy. ORCA needs accurate neighbor positions. Potential field methods need accurate obstacle maps. Deep RL is trained and validated in clean simulation environments. The problem these papers frame is that this assumption does not hold in the real world, and none of them tests what happens when it breaks.

### Q2. Why do false positives and false negatives matter in UAV swarms?

These algorithms treat sensor reports as ground truth. A false positive, a phantom obstacle, will cause an ORCA or DWA planner to steer around something that is not there, pulling the drone out of formation and pushing it into the path of neighbors. A false negative, a missed real obstacle, gives the planner nothing to react to, so the drone flies straight into the hazard. In the close-formation settings these papers study, both error types compound: one drone swerving unexpectedly due to a phantom can trigger reactive moves from neighbors, and a single missed obstacle in a tight configuration leaves no room or time to recover once it is finally seen.

### Q3. Why does sensor fusion reliability matter?

Some papers in this group already use fusion for coordination. The transformer-based system in 2.4 fuses photoelectric and nano-radar sensors. The mmWave radar and inertial odometry combination in 2.9 improves navigation in low-light spaces. These fusion inputs improve the quality of the obstacle picture. But the coordination algorithms sitting downstream do not account for what happens when one of the fusion inputs is wrong or degraded. A formation control algorithm that gets a confident but incorrect obstacle map will plan as if that map is real. Fusion reliability is not just a sensor concern. It directly limits how well coordination can work.

### Q4. Why is this important for safety?

These papers optimise for performance: shortest paths, lowest energy, fastest reaction times. Safety is measured as collision avoidance success rate under normal conditions. What is not measured is how that rate falls when the sensor inputs have errors. The reMARL framework reports 98.75% faster reaction times, but on clean simulation inputs. The DRL collision avoidance in 2.5 is validated without injected sensor noise. These results cannot be taken as safety guarantees for real deployment until we know how the algorithms degrade under corrupted perception.

### Q5. Why is simulation suitable for this study?

All algorithms in this group either run in simulation or are directly compatible with simulation environments. ORCA, DWA, potential fields, and MARL can all be implemented in multi-agent simulators. This means we can take the coordination logic from these papers, attach a perception layer with configurable faults, and measure how safety metrics change as error rate increases. Simulation is not a substitute for these algorithms. It is the native environment many of them were built and tested in, which makes it the right place to study what happens when their inputs go wrong.

---

## The Fusion Imperative

**Papers:**
- 2.10 Robust BEV Perception via Dual 4D Radar-Camera Fusion Under Adverse Conditions with Fog-Aware Enhancement (2026)
- 2.13 Radar-Camera Fusion in Perspective View and Bird's Eye View for 3D Object Detection (2025)
- 3.1 Camera-Radar Fusion with Radar Channel Extension and Dual-CBAM-FPN for Object Detection (2024)
- 3.4 Multi-Sensory Data Fusion in Terms of UAV Detection in 3D Space (2022)
- 3.10 Deep Camera-Radar Fusion with an Attention Framework for Autonomous Vehicle Vision in Foggy Weather Conditions (2023)

---

### Q1. What problem are we studying?

These papers explain why multi-modal fusion is necessary and what has been built to achieve it. They cover BEV radar-camera fusion under fog and adverse weather, cross-modal projection for 3D detection, dual-CBAM-FPN feature fusion, multi-sensory complementary filtration in 3D space, and attention-based fusion in degraded visibility. Together they show that fusion is now the accepted answer to individual sensor limitations and that current architectures do meaningfully improve detection under bad conditions. The gap they leave open is that all of them are tested on single targets or ground vehicles. None of them is tested in a swarm setting where many UAVs are producing and consuming fused data at once and where a bad fusion output affects a whole formation, not just one vehicle.

### Q2. Why do false positives and false negatives matter in UAV swarms?

These papers document how fusion architectures handle both error types. CBAM-FPN attention filters background clutter to reduce false positives. Radar features recover camera misses in fog to reduce false negatives. BEV feature denoising suppresses spurious activations from scattering. The important point for our study is that each of these mechanisms works by adjusting confidence. When a degraded sensor is given too high a confidence weight, these filters pass the error through instead of blocking it. A false positive that should have been suppressed reaches the coordination layer at full weight. This group establishes the mechanism by which perception errors survive fusion and become inputs to swarm decisions.

### Q3. Why does sensor fusion reliability matter?

These papers are direct evidence that fusion reliability is what determines the quality of the merged output. Under clean conditions all five architectures perform well. Under degraded conditions, performance depends entirely on whether the algorithm correctly identifies which modality is degraded and reduces its weight. DARE in 2.10 uses Doppler features to spot camera degradation and compensate. The attention framework in 3.10 learns to down-weight camera detections caused by fog scattering. Multi-stage clustering in 3.4 drops signatures that do not match known UAV profiles. All of these are confidence-weighting strategies. When confidence assignment goes wrong, the fusion gain disappears. Our study treats this as the key variable: how does swarm behavior change as fusion confidence mis-weighting increases?

### Q4. Why is this important for safety?

These papers show that the difference between a working fusion system and a mis-weighted one is not just a small accuracy drop. It can mean that a whole category of obstacle goes undetected. A camera-heavy fusion system in fog misses what only radar could see. A radar-heavy system in interference misses what only the camera could see. The swarm then acts on a map that has category-specific blind spots. What ends up in those blind spots determines whether the result is a near-miss or a collision.

### Q5. Why is simulation suitable for this study?

The failure modes in these papers, fog degradation, clutter, calibration offset, can only be tested in a controlled way through simulation. In the real world you cannot hold everything else constant while changing fog density from 0 to 100%. Simulation can do this. These papers also provide the quantitative degradation curves, how recall drops with fog severity, how false positive rate rises with clutter, that can be directly parameterised in a simulation fault model. Simulation lets us use these numbers to create realistic and repeatable fault conditions.

---

## When Uncertainty Leads to Unsafety: Empirical Insights into the Role of Uncertainty in Unmanned Aerial Vehicle Safety

**Authors:** Sajad Khatiri, Fatemeh Mohammadi Amin, Sebastiano Panichella, Paolo Tonella (2025)

---

### Q1. What problem are we studying?

This paper studies the link between uncertainty in a UAV's autonomous control system and actual flight safety violations. It measures uncertainty as control-signal inconsistency: the degree to which the same situation produces different outputs from the system across runs. The paper builds an empirical case from PX4-based camera obstacle avoidance logs, showing that uncertainty is not just a model quality number. It is a behavioral pattern that predicts real unsafe events. This is the most direct evidence in the literature that the problem we are studying, the link between perception error and safety outcome, is real and measurable rather than theoretical.

### Q2. Why do false positives and false negatives matter in UAV swarms?

The paper reports that up to 36% of unsafe flight states produced no detectable uncertainty signal. This is the false negative case at the decision level: the system appeared confident while an unsafe condition was building. Between 26 and 50% of uncertain decisions did not result in unsafe states, which is the false positive case: the system raised an alarm that did not correspond to real danger. Both matter in a swarm. A swarm that generates too many false positive safety signals will over-react, breaking formation unnecessarily. A swarm where over a third of genuine unsafe states go undetected has a blind spot that no coordination algorithm can compensate for.

### Q3. Why does sensor fusion reliability matter?

This paper uses black-box decision uncertainty as a stand-in for underlying perception uncertainty. The connection it draws is that when perception is unreliable, expressed as inconsistent control outputs, unsafe states follow. Sensor fusion sits directly in this chain. If the fusion layer produces inconsistent or low-confidence outputs because one degraded modality is weighted too heavily, the control system receives noisy inputs, produces inconsistent decisions, and, by the numbers in this paper, significantly increases the probability of a safety event.

### Q4. Why is this important for safety?

The quantitative result here is the strongest safety justification for our study. When the system is in an uncertain behavioral state, the probability of an unsafe event reaches up to 74%. Up to 89% of all unsafe states were preceded by a detectable uncertainty signal. These numbers mean uncertainty is a leading indicator of physical danger that appears before the violation happens. This reframes the research question. It is not about how accurate the perception system is. It is about how uncertain the system's behavior is and how that uncertainty connects to safety events. That is exactly the framing our simulation study uses.

### Q5. Why is simulation suitable for this study?

This paper builds its entire evidence base from simulation logs of PX4-Avoidance flight behavior. The runtime safety detector it validates is trained and tested on simulated flight data. This gives our study a direct methodological precedent: simulation-based analysis of the perception-uncertainty-to-safety-violation chain produces findings that are rigorous and worth citing. Our work extends this from a single UAV to a swarm and from black-box decision uncertainty to explicitly injected perception faults with known rates and magnitudes.

---

## Risk-Aware AI Architecture for BVLOS UAV Safety: Integrating Sensor Fusion and SATCOM

**Author:** Nick Barua (2026)

---

### Q1. What problem are we studying?

This paper addresses how a UAV system can stay safe when operating beyond visual line of sight, where ground-based oversight is not possible and the UAV must rely entirely on its own perception. Barua proposes a three-layer Risk-Aware UAV Safety Architecture (RASA) that scores sensor reliability against current environmental conditions and adjusts fusion weights and safety decisions in real time. The problem it defines is that BVLOS safety cannot be achieved by any single sensor or fixed algorithm. It requires an architecture that knows its own perception quality at runtime and can act on it. This is the formal architectural version of the problem our simulation study examines empirically.

### Q2. Why do false positives and false negatives matter in UAV swarms?

RASA handles both error types at the architecture level. For false positives, the AI risk model reduces the weight of sensor modalities currently operating in conditions known to produce spurious detections, so phantom obstacles do not enter the decision loop at high confidence. For false negatives, SATCOM data links maintain situational awareness when local sensors fail, providing a backup channel when onboard sensing misses something. In a swarm context, both mechanisms are needed. An architecture that lets either error type pass unfiltered into the coordination layer will produce unsafe swarm behavior, which our simulation is set up to demonstrate.

### Q3. Why does sensor fusion reliability matter?

Fusion reliability is the central concern of RASA. The whole point of dynamic weight adjustment is that a system using fixed fusion weights will become unreliable in the very conditions where BVLOS operations are hardest: fog, signal interference, and communication problems each degrade specific modalities in specific ways. An architecture that recognises this and adjusts weights in real time is fundamentally different from one that fuses all inputs equally regardless of condition. The question our simulation asks, what happens to swarm behavior when confidence weighting is wrong, is exactly the failure case RASA is designed to prevent. We study the problem; this paper proposes one solution.

### Q4. Why is this important for safety?

RASA is built as a safety architecture, not a performance one. Its target operating condition, BVLOS at scale, is where stakes are highest: no human backup, longest sensor links, and highest cost for any failure. Barua argues that current safety frameworks fall short because they do not account for sensor reliability degrading in real time. Our simulation provides the empirical numbers that argument needs. By comparing swarm safety outcomes with and without RASA-like confidence adjustment, we can quantify the safety cost of not having it.

### Q5. Why is simulation suitable for this study?

Barua explicitly notes that RASA is a conceptual framework without extensive multi-agent simulation data behind it. Our study addresses that gap directly. Simulation is the right tool because it lets us implement RASA-like and non-RASA-like fusion architectures, subject them to the same fault injection scenarios, and measure the difference in swarm safety outcomes. This is not possible to do systematically in real flight, where fault conditions cannot be controlled or repeated exactly.

---

## Enabling Robots to Autonomously Search Dynamic Cluttered Post-Disaster Environments

**Authors:** K. Rado, M. Baglioni, A. Jamshidnejad (2025)

---

### Q1. What problem are we studying?

This paper tackles autonomous navigation in dynamic, cluttered environments where perception is uncertain and obstacles may appear, move, or vanish unpredictably. It combines heuristic motion planning with Tube Model Predictive Control (TMPC), which explicitly keeps bounded perception uncertainty inside the planning loop rather than ignoring it. The problem it solves is how to plan safe paths when you cannot be certain whether a detected obstacle is real or a phantom, or whether a clear path actually has something in it. This is a precise formulation of the false positive and false negative problem applied to navigation, and it is the closest existing work to the planning-level consequence we are trying to measure in a UAV swarm.

### Q2. Why do false positives and false negatives matter in UAV swarms?

Rado et al. handle false positives by building obstacle belts: inflated safety margins around detections that absorb the spatial uncertainty from phantom hits. Rather than trying to decide whether each detection is real, TMPC treats all detections as possibly real and plans wide enough to stay safe even if some are not. For false negatives, the replanning mechanism adjusts trajectory quickly when a previously undetected obstacle appears. In a swarm, the obstacle belt idea transfers directly: a formation that inflates its avoidance margins around all detections trades some efficiency for protection against false positives. Measuring that trade-off under varying false positive rates is part of what our simulation does.

### Q3. Why does sensor fusion reliability matter?

TMPC relies on bounded perception uncertainty: it assumes errors in detection stay within known limits, and plans around those limits. This only works if the fusion system feeding TMPC has calibrated, honest confidence estimates. If the fusion layer sends an overconfident output from a degraded sensor, the obstacle belt shrinks to the wrong size and the safety guarantee breaks. Fusion reliability is therefore a precondition for TMPC's safety properties to hold. Our simulation models exactly this dependency by varying fusion confidence calibration and observing where the guarantees fail.

### Q4. Why is this important for safety?

This paper provides a mathematically grounded safety result: TMPC with bounded uncertainty, validated with collision avoidance rates and mission reachability numbers. The acknowledged limitation is that it was tested on ground robots in search-and-rescue settings, not on high-speed aerial swarms. The safety properties hold as long as real perception error stays within the modelled bounds. When sensor degradation and miscalibrated fusion push error beyond those bounds, the guarantee breaks. Our simulation identifies where and how fast that happens.

### Q5. Why is simulation suitable for this study?

TMPC's error bound is directly parameterisable in simulation. We can set the assumed error bound, inject actual errors at or above that level, and observe whether the safety guarantee holds or breaks. This cannot be done systematically in real deployment where the actual error rate is unknown and cannot be controlled. Simulation gives us the control over error magnitude that makes the safety analysis repeatable and interpretable.

---

## An Uncertainty Quantization-Based Method for Anti-UAV Detection in Infrared Images

**Authors:** C. Wu, W. Tang, Y. Rao, Y. Chen, H. Ding, S. Zhu et al. (2025)

---

### Q1. What problem are we studying?

This paper tackles the problem of unreliable detection confidence in single-modality infrared UAV detection. Its three-stage Bayesian CNN produces not just segmentation outputs but also pixel-wise probabilistic uncertainty maps: a measure of how confident the model is in each part of its detection result. The core problem it solves is that a detection without a confidence measure gives the downstream system no way to know how much to trust it. A high-confidence and a low-confidence detection look identical to a coordination algorithm reading only the detection label. Uncertainty maps make that distinction visible so the downstream system can respond differently depending on how sure the perception layer actually is.

### Q2. Why do false positives and false negatives matter in UAV swarms?

Wu et al. tackle both error types in one framework. Uncertainty maps flag low-confidence background activations in cluttered areas where the model is uncertain, and suppress them, reducing false positives. The same framework uses uncertainty-guided refinement to increase the salience of faint, small, distant targets that plain segmentation misses, reducing false negatives. The dual function matters because fixing only one side creates a new problem. A system that suppresses false positives without recovering false negatives trades phantom obstacles for missed real ones, which is equally dangerous. Uncertainty maps handle both by making confidence the deciding factor rather than the raw detection score.

### Q3. Why does sensor fusion reliability matter?

This paper works on a single infrared sensor, which it acknowledges as its limitation. In a multi-sensor fusion setting, uncertainty maps become more powerful because each modality produces its own confidence estimate. The fusion layer can then weight each modality by its per-pixel confidence rather than treating all inputs as equally reliable. When one sensor's uncertainty map shows high uncertainty in a region, the fusion system defers to the other modality that shows lower uncertainty there. This is how confidence-weighted fusion becomes principled rather than arbitrary, and this paper shows that generating the necessary confidence signal is technically feasible.

### Q4. Why is this important for safety?

In a swarm, an uncertainty map acts as a real-time signal of where the perception system's picture of the world is reliable and where it is not. A swarm that can read this signal can tighten its safety margins in uncertain regions and plan more freely in confident ones. A swarm without this signal must assume the same level of reliability everywhere, which forces a choice between being always over-cautious, wasting energy and breaking formation, or always under-cautious, accepting collision risk in degraded areas. Wu et al. show that generating the uncertainty signal is possible. Our study examines what swarm behavior looks like when this signal is available versus when it is not.

### Q5. Why is simulation suitable for this study?

The uncertainty map framework in this paper is validated entirely in controlled experiments with known ground truth: images where the model's uncertainty output is compared against labelled correct and incorrect detections. The same logic applies to simulation. We know exactly where faults were injected, we can check whether the uncertainty maps flag those regions correctly, and we can measure the effect on swarm decisions downstream. Simulation provides the ground truth labels, real versus phantom obstacle, real versus missed target, that make the uncertainty quantification analysis interpretable at swarm scale.

---

## Drone Swarm Strategy for the Detection and Tracking of Occluded Targets in Complex Environments / An Autonomous Drone Swarm for Detecting and Tracking Anomalies Among Dense Vegetation

**Papers:**
- 3.14 Drone Swarm Strategy for the Detection and Tracking of Occluded Targets in Complex Environments -- Amala Arokia Nathan R.J., Kurmi I., Bimber O. (2023)
- 3.15 An Autonomous Drone Swarm for Detecting and Tracking Anomalies Among Dense Vegetation -- Amala Arokia Nathan R.J., Strand S., Mehrwald D., Shutin D., Bimber O. (2025)

---

### Q1. What problem are we studying?

These two papers study how a UAV swarm can keep detecting and tracking targets when individual drones cannot see them due to occlusion, the most common natural cause of false negatives in visual sensing. In complex environments (3.14) and dense vegetation (3.15), no single UAV has a clear view. Targets are hidden from individual cameras. The response in both papers is coordinated repositioning: the swarm spreads itself spatially to get collective line-of-sight coverage that no individual member could achieve alone. The problem is not just false negatives at the sensor level. It is how the swarm should organise itself to recover from them systematically, which is exactly the swarm-level behavioral response to false negatives we want to characterise in our study.

### Q2. Why do false positives and false negatives matter in UAV swarms?

Both papers focus on false negatives as their central problem. In 3.14, a target hidden in a complex environment is missed by individual UAVs. The swarm responds with coordinated re-observation and consensus validation: a detection is only confirmed when multiple UAVs see it from independent angles. In 3.15, the same logic applies to vegetation-dense environments, where alternate viewing angles are deliberately assigned to swarm members to cover the blind spots of primary observers. The shared finding across both papers is that false negatives at the individual level can be recovered at the swarm level, but only if the swarm has enough redundancy and the coordination protocol explicitly plans for missed detections. Without that, the missed obstacle stays invisible to the whole group.

### Q3. Why does sensor fusion reliability matter?

The consensus validation mechanism in both papers is effectively a form of sensor fusion across swarm members. Multiple independent observations of the same region are merged to produce a collective detection with higher confidence. This is structurally the same as multi-modal hardware sensor fusion, with swarm members acting as the modalities. The reliability concern is the same too. Just as a miscalibrated hardware sensor introduces low-quality data into a fusion pipeline, a swarm member with a degraded or blocked sensor introduces low-quality data into the consensus. If the consensus algorithm treats all members equally regardless of their individual observation quality, one degraded member can weaken the collective confidence enough to cause a real obstacle to go undetected. The solution that follows from these papers is confidence-weighted fusion applied at the swarm coordination level.

### Q4. Why is this important for safety?

Both papers show that safety in an occlusion-heavy environment depends on the swarm's coverage redundancy. In 3.14, enough redundancy means reliable tracking even with complex occlusion. Not enough means detection gaps appear. In 3.15, dense vegetation coverage requires explicit assignment of alternate viewing angles, which is an acknowledgment that without deliberate coordination, false negatives from occlusion will persist and affect mission safety. The safety implication is clear: false negatives from occlusion are unavoidable for any individual sensor, and the swarm is only as safe as its coordination protocol's ability to recover from them.

### Q5. Why is simulation suitable for this study?

Both papers validate their cooperative strategies through simulation or controlled empirical trials. Our study extends that methodology in the direction these papers do not cover: injecting known false negatives with controlled rates and spatial patterns, and measuring how well the swarm's consensus and redistribution protocols recover from them at different fault severities. The occlusion scenarios in 3.14 and 3.15 represent one cause of false negatives. Our simulation generalises to any perception failure that produces missed detections, which lets us characterise the recovery range of swarm-level consensus mechanisms across a broader set of fault conditions.

---

## Impact Study of Faulty Sensors on Flocking-Based Cooperative Control of Nonholonomic Robots

**Author:** Iftekhar L. (2025)

---

### Q1. What problem are we studying?

This paper studies what happens to collective behavior when the sensors in the coordination loop are faulty. It injects specific sensor fault conditions into a flocking algorithm for nonholonomic robots and measures how swarm cohesion degrades as a result. The problem it quantifies is that sensor faults do not just slow down individual agents. They degrade collective behavior in ways that cannot be predicted from single-agent analysis alone. Swarm cohesion is an emergent property, and when individual sensors go wrong, what breaks is the collective outcome. This is the closest existing study to what we are doing, applied to ground robots rather than UAVs.

### Q2. Why do false positives and false negatives matter in UAV swarms?

The sensor fault models in this paper correspond directly to false positives and false negatives. A sensor reporting a spurious obstacle is a false positive at the hardware level. A sensor failing to report a real one is a false negative. The paper shows that flocking cohesion degrades measurably as fault rate increases, and that once fault rate passes a threshold, recovery is incomplete. This is the quantitative version of the false positive and false negative argument: at low fault rates, the collective redundancy of the swarm absorbs the individual errors. At high rates, it cannot, and safe coordination breaks down. Our study uses this to ask where that threshold sits for UAV swarms.

### Q3. Why does sensor fusion reliability matter?

When a faulty sensor's output is fused with healthy outputs at full weight, the faulty readings are blended into the collective perception at the same level as correct ones. Iftekhar's results show that this spreading effect degrades cohesion more than the raw fault rate alone would suggest. The fusion layer amplifies rather than absorbs the fault. A fusion system that identifies and down-weights the faulty sensor would contain this spreading. The paper's findings are therefore an argument for fault-aware fusion that stops one sensor's error from contaminating the collective, not just for fault-tolerant coordination algorithms.

### Q4. Why is this important for safety?

The value of this paper is that it measures rather than argues. Flocking success rate drops, cohesion metrics worsen, and recovery is incomplete at high fault rates. For dependability research, this is exactly the kind of evidence needed: a controlled demonstration that the perception-to-behavior fault propagation chain produces real, measurable safety degradation at the swarm level. Our study extends this methodology from nonholonomic ground robots to UAV swarms and from general sensor faults to specifically injected false positives and false negatives with controlled rates and magnitudes.

### Q5. Why is simulation suitable for this study?

This paper establishes the methodological template our study follows: inject faults in a simulated swarm, measure collective behavioral metrics, and analyse degradation as fault severity increases. Simulation is the only method that allows fault injection at controlled and repeatable rates. Iftekhar's results confirm that simulation-based fault injection produces swarm safety data that is meaningful and interpretable. Our study follows the same template at UAV scale, using false positive and false negative rates as the primary variables.

---

## Confidence-Weighted Cooperative Merging of Observations for Safe Navigation of UAV Swarms

**Author:** Konnikov (2026)

---

### Q1. What problem are we studying?

This paper addresses how a UAV swarm should combine observations from multiple members when those members have different sensor quality. Some will have degraded hardware, poor line-of-sight, or bad environmental conditions. Treating their observations at the same weight as high-quality members puts low-quality data directly into the collective navigation decision. The confidence-weighted merging approach assigns each observation a reliability score and weights it accordingly. The problem it solves, maintaining collective navigation safety when individual members have unreliable sensing, is the swarm-scale version of the sensor fusion reliability problem at the centre of our study.

### Q2. Why do false positives and false negatives matter in UAV swarms?

Confidence-weighted merging handles both error types at the coordination level. Low-confidence observations from a degraded swarm member are down-weighted in the merge, so that member's phantom detections contribute less to the collective obstacle map, reducing the impact of false positives. For false negatives, redundant coverage from multiple members means that an obstacle missed by one member with poor visibility can still be detected by others with better viewing angles, and their higher-confidence observations dominate the result. The weighting mechanism acts as a collective filter that is more robust than any single sensor.

### Q3. Why does sensor fusion reliability matter?

This paper is a direct answer to the question. When fusion weights are uniform, collective navigation quality is limited by the worst-performing swarm member. When weights reflect actual confidence, collective quality approaches that of the best-performing members. How accurately the confidence weights reflect real observation quality determines how much of that benefit is realised. Our study treats weight miscalibration as the primary independent variable: we inject errors and ask how much confidence mis-weighting the swarm can tolerate before collective navigation falls below safe thresholds.

### Q4. Why is this important for safety?

Swarm navigation safety is the stated metric of this paper. The architecture is evaluated on safety and coordination robustness, not just detection accuracy. This makes it the most direct existing work linking sensor observation quality to swarm safety outcomes in a formal way. The limitation Konnikov identifies, that swarm members have heterogeneous sensor quality, is precisely the condition our simulation studies. Confidence-weighted merging is a proposed solution. Our study characterises its failure envelope: how much mis-weighting it can absorb before swarm safety degrades.

### Q5. Why is simulation suitable for this study?

The confidence-weighted merging algorithm works on simulated observations as naturally as real ones. The mathematics of weighting incoming data does not change based on whether the data came from real sensors or simulated ones. This means we can implement the architecture in simulation, inject known confidence errors, and measure the resulting safety degradation with full ground-truth knowledge of which observations were correct, which were false positives, and which were false negatives. Simulation provides the labels that make the confidence-weighting analysis interpretable.
