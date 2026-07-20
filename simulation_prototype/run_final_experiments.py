"""
run_final_experiments.py

Task 17: one command that runs the full experiment matrix - every
scenario in simulation_config.json, using run_final_simulations.py's own
two-tier trial plan (Task 9): every scenario at --core-trials, plus the
key fusion-comparison scenarios bumped to --comparison-trials whenever
the runtime budget allows.

This is a thin pass-through, not a reimplementation: run_final_simulations.py
already *is* the full-matrix runner (see its own module docstring for the
full trial plan and output list). This file only exists to be the
project's single documented "run everything" command, named to sit
alongside run_final_demo.py's "run a small representative slice".

All flags (--core-trials, --comparison-trials, --time-budget-seconds,
--base-seed, --seed-mode, --output-dir, --skip-step-logs, ...) are
run_final_simulations.py's own - see `python run_final_experiments.py --help`.

Usage:
    python run_final_experiments.py
    python run_final_experiments.py --time-budget-seconds 600
"""

import sys

from run_final_simulations import main

if __name__ == "__main__":
    sys.exit(main())
