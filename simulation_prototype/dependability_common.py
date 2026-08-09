import copy
import json
import os
import traceback


def clone_scenario(config, base_scenario, new_name, overrides, description=None):
    """Deep-copies config['scenarios'][base_scenario] under new_name with
    `overrides` applied on top, and inserts it into (a copy of) config.
    Returns the new config; does not mutate the original.

    Used to synthesize the scenario configs this project's
    simulation_config.json doesn't yet define (Covariance-Intersection
    fusion mode, ghost/aliasing radar conditions, ...) without ever
    editing the shared config file in place.
    """
    new_config = copy.deepcopy(config)
    scn = copy.deepcopy(new_config["scenarios"][base_scenario])
    scn.update(overrides)
    if description is not None:
        scn["description"] = description
    new_config["scenarios"][new_name] = scn
    return new_config


def run_trials(label, trial_fn, seeds, on_progress=None):
    """Runs trial_fn(seed) once per seed, catching exceptions per-trial.

    Returns (results, failures, seeds_used):
      results  - list of {"seed": s, "value": <trial_fn return>} for
                 trials that succeeded
      failures - list of {"seed": s, "error": <traceback string>} for
                 trials that raised
      seeds_used - the full seed list attempted (for the seeds record)
    """
    results, failures = [], []
    for i, seed in enumerate(seeds):
        try:
            value = trial_fn(seed)
            results.append({"seed": seed, "value": value})
        except Exception:
            err = traceback.format_exc(limit=6)
            failures.append({"seed": seed, "error": err})
            print(f"  [FAIL] {label} seed={seed}: {err.strip().splitlines()[-1]}")
        if on_progress:
            on_progress(i + 1, len(seeds))
    return results, failures, list(seeds)


def seed_range(base_seed, n):
    return [base_seed + i for i in range(n)]


class DependabilityWriter:
    """Owns the output directory layout for one comparison and writes the
    five required artifact kinds: raw logs, run summaries, seeds,
    configurations, failed-run records - plus an aggregated-results file.
    """

    def __init__(self, root_dir, comparison_name):
        self.comparison_name = comparison_name
        self.dir = os.path.join(root_dir, comparison_name)
        os.makedirs(self.dir, exist_ok=True)

    def _path(self, name):
        return os.path.join(self.dir, name)

    def write_json(self, name, obj):
        path = self._path(name)
        with open(path, "w") as f:
            json.dump(obj, f, indent=2, default=str)
        return path

    def write_raw_log(self, name, rows):
        """Raw per-trial rows, saved as JSON (row shapes vary too much
        across comparisons - track rows, fused rows, calibration pairs,
        mission-sim rows - for one fixed CSV schema)."""
        return self.write_json(name, rows)

    def write_configuration(self, config, scenario_names):
        """Saves only the scenario sub-configs this comparison actually
        used (plus the shared sim/world/etc. blocks), not the whole
        project config, so it's obvious at a glance what conditions
        produced this comparison's numbers."""
        snapshot = {
            "sim": config.get("sim"),
            "scenarios_used": {
                name: config["scenarios"][name]
                for name in scenario_names if name in config["scenarios"]
            },
        }
        return self.write_json("configuration_used.json", snapshot)

    def write_seeds(self, seeds_by_arm):
        return self.write_json("seeds.json", seeds_by_arm)

    def write_failed_runs(self, failures_by_arm):
        total = sum(len(v) for v in failures_by_arm.values())
        return self.write_json("failed_runs.json", {
            "total_failed": total,
            "by_arm": failures_by_arm,
        })

    def write_run_summary(self, summary_rows):
        return self.write_json("run_summary.json", summary_rows)

    def write_aggregated_results(self, aggregated):
        return self.write_json("aggregated_results.json", aggregated)
