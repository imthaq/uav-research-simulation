# Radar-Like Multimodal UAV Swarm Simulation

This repository contains the final research implementation of a highly realistic, multimodal UAV swarm simulation. It is designed to evaluate decentralized swarm decision-making under severe perception uncertainty, including sensor noise, clutter, false alarms, radar dropouts, and communication outages.

## Features

- **Strict Ground-Truth Isolation:** UAV runtime controllers never have access to ground truth; decisions are made exclusively on noisy, fused sensor data.
- **Multimodal Perception Modeling:** Simulates Radar, Vision, and LiDAR characteristics including range-dependent noise, Probability of Detection (P_D), Probability of False Alarm (P_FA), and Poisson clutter.
- **Advanced Tracking & Fusion:** Implements Kalman filtering with nearest-neighbor data association, track coasting, and dynamic trust-weighted multimodal fusion.
- **Decentralized Communication:** Simulates packet loss, latency, and complete network outages between swarm agents.
- **Comprehensive Evaluation:** Automated Monte Carlo experiment framework for generating rigorous statistical evidence.

## Installation

Ensure you have Python 3.8+ installed.

```bash
# Clone the repository
git clone https://github.com/yourusername/uav-research.git
cd uav-research

# Install dependencies
pip install -r simulation_prototype/requirements.txt
```

## Quick Start: Interactive Demo

To launch the interactive GUI, tweak sensor parameters in real-time, and watch a single simulation trial unfold (2D or 3D):

```bash
python simulation_prototype/run_interactive_demo.py
```

## Reproducibility: Automated Demo

To run a small, fast, representative slice of the core experiments and generate a subset of the CSV logs (proving the pipeline works on your machine):

```bash
python simulation_prototype/run_final_demo.py
```
Results will be saved in `simulation_prototype/results/demo/`.

## Running the Full Experiment Matrix

To re-run the massive batch of final experiments (hundreds of trials across all scenarios) that generated the statistics used in the final report:

```bash
python simulation_prototype/run_final_simulations.py
```
*Warning: This process is computationally intensive and may take significant time to complete.*

## Project Structure

- `simulation_prototype/`: The core python simulation codebase, models, and execution scripts.
  - `validation/`: Isolated validation tests for radar, tracking, fusion, and trust models.
  - `results/final_release/`: The aggregated CSV metrics and statistical JSON analysis.
  - `plots/final_release/`: High-quality plots comparing fusion modes, latency, and collision risks.
- `media/`: Pre-rendered 3D videos of major scenarios (baseline, heavy clutter, communication outage, etc.).
- `final_acceptance/`: Contains the final technical report (`Final_Report.docx`), the presentation (`Radar_Swarm_Simulation_Final_Presentation.pptx`), and the demonstration script.
- `notes/`: A comprehensive collection of research literature reviews, paper classifications, methodology drafts, and simulation architecture documentation. Includes the core `literature_matrix.csv` and `literature_summary.md`.
- `tables/`: Planning documentation and expected results schemas.

## Final Release Artifacts

The formal research findings are documented in the `final_acceptance` directory. Please refer to the **Final Report** for detailed methodology, statistical significance testing, and architectural conclusions. The **Demonstration Script** (`final_research_demo_script.md`) can be used to guide live presentations of this system.

---
*This research simulation is frozen and validated against strict completion gates. Ground-truth leakage has been fully audited and removed.*