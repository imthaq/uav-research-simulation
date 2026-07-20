# Environment Setup & Quick Start Guide

This guide provides a concise overview of the environment setup, dependency requirements, sanity checks, and primary execution commands for the UAV Swarm Simulation and Sensor Fusion framework.

---

## 1. Prerequisites & System Requirements

* **Python:** Python 3.8+ (64-bit recommended)
* **Core Python Packages:** `numpy`, `matplotlib`, `pillow`
* **External Tool (Optional):** `ffmpeg` (required for exporting `.mp4` video animations via `simulation_visualizer.py`)

---

## 2. Quick Environment Setup

1. **Clone / Extract Project Repository**
2. **Install Dependencies:**
   ```bash
   pip install numpy matplotlib pillow
   ```
3. **Verify FFmpeg Installation (for video exports):**
   ```bash
   ffmpeg -version
   ```

---

## 3. Environment Sanity Check (Quick Demo)

Run a fast, lightweight verification test to ensure all imports, simulation models, and directory structures are working correctly:

```bash
python run_final_demo.py
```

* **Description:** Executes a small representative slice (`baseline`, `naive_fusion`, `high_dropout`, `communication_outage`) across a few trials.
* **Output:** Writes test logs and summary files to `results/demo/`.

---

## 4. Running the Full Experiment Suite

To execute the full simulation matrix across all scenarios defined in `simulation_config.json`:

```bash
python run_final_experiments.py
```

* **Customizing Runtime Budget / Trials:**
  ```bash
  python run_final_experiments.py --time-budget-seconds 600 --core-trials 10
  ```
* **Output:** Generates comprehensive run-level CSVs, step logs, and scenario summaries in the `results/` directory.

---

## 5. Generating Demonstration Videos

To generate the full advanced demonstration video suite covering Kalman tracking, clutter stress tests, sensor dropout recovery, faulty sensor dynamic trust weighting, and communication outages:

```bash
python simulation_visualizer.py --advanced-demos
```

To generate a side-by-side comparison video of centralized vs. distributed fusion architectures:

```bash
python simulation_visualizer.py --fusion-comparison
```

All rendered `.mp4` files are saved to the `media/` directory.
