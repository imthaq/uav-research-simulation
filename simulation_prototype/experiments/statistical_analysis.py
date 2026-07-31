"""
statistical_analysis.py

Performs statistical analysis on simulation results:
- Means, stdevs, confidence intervals per scenario/fusion_mode
- Effect size (Cohen's d) between fusion modes
- Correlations between perception parameters and outcomes
- ANOVA/Kruskal-Wallis across fusion modes
- Paired comparisons (naive vs trust-weighted)
- Significance tests for mission success, collision risk, response time
"""

import argparse
import csv
import json
import os
import sys
import warnings
from collections import defaultdict
from scipy import stats
import numpy as np

warnings.filterwarnings("ignore", category=RuntimeWarning)

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_results_csv(path):
    """Load results_summary.csv from run_experiments output."""
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric columns
            for k in row:
                if k in ['collision_risk_count', 'unnecessary_avoidance_count', 
                         'missed_response_count', 'fusion_recovery_count', 'total_near_misses']:
                    try:
                        row[k] = int(row[k])
                    except (ValueError, TypeError):
                        pass
                elif k in ['avg_response_time_s', 'avg_formation_error', 'avg_confidence_error',
                          'false_positive_rate', 'false_negative_rate', 'noise_level', 
                          'dropout_probability', 'confidence_error_level',
                          'expected_calibration_error', 'maximum_calibration_error',
                          'brier_score', 'negative_log_likelihood',
                          'overconfidence_rate', 'underconfidence_rate']:
                    try:
                        row[k] = float(row[k])
                    except (ValueError, TypeError):
                        pass
                elif k in ['latency_steps', 'calibration_n_samples']:
                    try:
                        row[k] = int(row[k])
                    except (ValueError, TypeError):
                        pass
                elif k == 'mission_success':
                    row[k] = row[k].lower() in ['yes', 'true', '1']
            rows.append(row)
    return rows


def group_by_scenario(rows):
    """Group rows by scenario name."""
    groups = defaultdict(list)
    for row in rows:
        groups[row['scenario']].append(row)
    return groups


def group_by_fusion_mode(rows):
    """Group rows by fusion_mode."""
    groups = defaultdict(list)
    for row in rows:
        groups[row['fusion_mode']].append(row)
    return groups


def cohens_d(x, y):
    """Cohen's d effect size between two groups."""
    n1, n2 = len(x), len(y)
    if n1 < 2 or n2 < 2:
        return None
    var1, var2 = np.var(x, ddof=1), np.var(y, ddof=1)
    pooled_var = ((n1 - 1) * var1 + (n2 - 1) * var2) / (n1 + n2 - 2)
    return (np.mean(x) - np.mean(y)) / np.sqrt(pooled_var) if pooled_var > 0 else None


def compute_scenario_stats(scenario_rows):
    """Compute mean, stdev, 95% CI for each metric in a scenario."""
    metrics = {}
    metric_names = ['collision_risk_count', 'mission_success', 'avg_response_time_s', 
                    'avg_formation_error', 'missed_response_count', 'fusion_recovery_count',
                    'expected_calibration_error', 'maximum_calibration_error', 'brier_score',
                    'negative_log_likelihood', 'overconfidence_rate', 'underconfidence_rate']
    
    for metric in metric_names:
        values = [row.get(metric) for row in scenario_rows if row.get(metric) not in (None, '')]
        if not values:
            continue
        
        if metric == 'mission_success':
            values = [1 if v else 0 for v in values]
        
        values = [float(v) for v in values]
        n = len(values)
        mean = np.mean(values)
        stdev = np.std(values, ddof=1) if n > 1 else 0
        se = stdev / np.sqrt(n) if n > 0 else 0
        ci = 1.96 * se  # 95% CI
        
        metrics[metric] = {
            'mean': mean,
            'stdev': stdev,
            'n': n,
            'ci95_half_width': ci,
            'values': values
        }
    
    return metrics


def fusion_mode_comparison(rows):
    """Compare metrics across fusion modes: means, stdevs, effect sizes, ANOVA."""
    fusion_groups = group_by_fusion_mode(rows)
    
    results = {}
    metric_names = ['collision_risk_count', 'avg_response_time_s', 'avg_formation_error',
                     'expected_calibration_error', 'brier_score']
    
    for metric in metric_names:
        mode_values = {}
        for mode, mode_rows in fusion_groups.items():
            values = [float(row.get(metric)) for row in mode_rows 
                     if row.get(metric) not in (None, '')]
            if values:
                mode_values[mode] = values
        
        if len(mode_values) < 2:
            continue
        
        # ANOVA
        f_stat, p_val = stats.f_oneway(*mode_values.values())
        
        results[metric] = {
            'modes': {mode: {
                'mean': np.mean(vals),
                'stdev': np.std(vals, ddof=1) if len(vals) > 1 else 0,
                'n': len(vals)
            } for mode, vals in mode_values.items()},
            'anova_f': f_stat,
            'anova_p': p_val,
            'significant': p_val < 0.05
        }
        
        # Pairwise Cohen's d for significant results
        if p_val < 0.05:
            modes = list(mode_values.keys())
            for i in range(len(modes)):
                for j in range(i + 1, len(modes)):
                    d = cohens_d(mode_values[modes[i]], mode_values[modes[j]])
                    if d is not None and 'pairwise_d' not in results[metric]:
                        results[metric]['pairwise_d'] = {}
                    if d is not None:
                        results[metric]['pairwise_d'][f"{modes[i]}_vs_{modes[j]}"] = d
    
    return results


def mission_success_by_mode(rows):
    """Compare mission success rate across fusion modes with chi-square."""
    fusion_groups = group_by_fusion_mode(rows)
    
    contingency_data = {}
    for mode, mode_rows in fusion_groups.items():
        success = sum(1 for row in mode_rows if row.get('mission_success'))
        total = len(mode_rows)
        contingency_data[mode] = (success, total - success)
    
    # Create contingency table and run chi-square
    if len(contingency_data) >= 2:
        modes = list(contingency_data.keys())
        table = np.array([contingency_data[m] for m in modes])
        chi2, p_val, dof, expected = stats.chi2_contingency(table)
        
        return {
            'modes': {mode: {'success': contingency_data[mode][0], 
                            'total': sum(contingency_data[mode])}
                     for mode in modes},
            'chi2': chi2,
            'p_value': p_val,
            'significant': p_val < 0.05
        }
    return None


def paired_comparison_naive_vs_trust(rows):
    """Compare naive_fusion vs trust_weighted_fusion in paired scenarios."""
    by_scenario = group_by_scenario(rows)
    
    paired_data = {'collision_risk_count': [], 'avg_response_time_s': [], 'avg_formation_error': [],
                   'expected_calibration_error': [], 'brier_score': []}
    
    for scenario, scenario_rows in by_scenario.items():
        naive_rows = [r for r in scenario_rows if r.get('fusion_mode') == 'naive_fusion']
        trust_rows = [r for r in scenario_rows if r.get('fusion_mode') == 'trust_weighted_fusion']
        
        if not naive_rows or not trust_rows:
            continue
        
        # Match by run_number for true pairing
        for nr in naive_rows:
            matching_tr = next((tr for tr in trust_rows if tr.get('run_number') == nr.get('run_number')), None)
            if matching_tr:
                for metric in paired_data.keys():
                    nv = nr.get(metric)
                    tv = matching_tr.get(metric)
                    if nv is not None and tv is not None:
                        paired_data[metric].append((float(nv), float(tv)))
    
    results = {}
    for metric, pairs in paired_data.items():
        if len(pairs) >= 3:
            naive_vals, trust_vals = zip(*pairs)
            t_stat, p_val = stats.ttest_rel(naive_vals, trust_vals)
            results[metric] = {
                'n_pairs': len(pairs),
                'naive_mean': np.mean(naive_vals),
                'trust_mean': np.mean(trust_vals),
                'mean_diff': np.mean(naive_vals) - np.mean(trust_vals),
                't_statistic': t_stat,
                'p_value': p_val,
                'significant': p_val < 0.05,
                'd': cohens_d(list(naive_vals), list(trust_vals))
            }
    
    return results if results else None


def perception_parameter_correlation(rows):
    """Correlate perception error parameters with swarm outcomes."""
    metrics = ['collision_risk_count', 'missed_response_count', 'avg_formation_error']
    params = ['false_positive_rate', 'false_negative_rate', 'noise_level', 
              'latency_steps', 'dropout_probability', 'confidence_error_level']
    
    results = {}
    for metric in metrics:
        results[metric] = {}
        for param in params:
            metric_vals = []
            param_vals = []
            
            for row in rows:
                m = row.get(metric)
                p = row.get(param)
                if m not in (None, '') and p not in (None, ''):
                    try:
                        metric_vals.append(float(m))
                        param_vals.append(float(p))
                    except (ValueError, TypeError):
                        pass
            
            if len(metric_vals) >= 3:
                # Spearman correlation (less assumption-heavy)
                rho, p_val = stats.spearmanr(param_vals, metric_vals)
                results[metric][param] = {
                    'correlation': rho,
                    'p_value': p_val,
                    'significant': p_val < 0.05
                }
    
    return results


def main():
    parser = argparse.ArgumentParser(description="Statistical analysis of simulation results")
    parser.add_argument("--input", default=os.path.join(_ROOT_DIR, "results", "results_summary.csv"), 
                       help="Input CSV from run_experiments.py")
    parser.add_argument("--output", default=os.path.join(_ROOT_DIR, "results", "statistical_analysis.json"),
                       help="Output JSON file")
    args = parser.parse_args()
    
    try:
        rows = load_results_csv(args.input)
    except FileNotFoundError:
        sys.exit(f"Results file not found: {args.input}")
    
    if not rows:
        sys.exit("No data found in results file")
    
    # Compute all analyses
    by_scenario = group_by_scenario(rows)
    scenario_stats = {s: compute_scenario_stats(sr) for s, sr in by_scenario.items()}
    
    analyses = {
        'per_scenario': scenario_stats,
        'fusion_mode_comparison': fusion_mode_comparison(rows),
        'mission_success_by_mode': mission_success_by_mode(rows),
        'paired_naive_vs_trust': paired_comparison_naive_vs_trust(rows),
        'perception_parameter_correlation': perception_parameter_correlation(rows)
    }
    
    # Convert numpy types to Python types for JSON serialization
    def convert(obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        if isinstance(obj, np.bool_):
            return bool(obj)
        if isinstance(obj, dict):
            return {k: convert(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [convert(item) for item in obj]
        return obj
    
    analyses = convert(analyses)
    
    # Write JSON
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(analyses, f, indent=2)
    
    # Print summary to stdout
    print(f"\n=== Statistical Analysis Summary ===\n")
    for scenario, stats_dict in scenario_stats.items():
        print(f"[{scenario}]")
        if 'collision_risk_count' in stats_dict:
            s = stats_dict['collision_risk_count']
            print(f"  collision_risk: mean={s['mean']:.2f}, stdev={s['stdev']:.2f}, ci95±={s['ci95_half_width']:.2f}")
        if 'avg_response_time_s' in stats_dict:
            s = stats_dict['avg_response_time_s']
            print(f"  response_time: mean={s['mean']:.3f}s, stdev={s['stdev']:.3f}s")
        if 'mission_success' in stats_dict:
            s = stats_dict['mission_success']
            print(f"  mission_success: {s['mean']*100:.0f}%")
        if 'expected_calibration_error' in stats_dict:
            s = stats_dict['expected_calibration_error']
            print(f"  confidence_calibration: ECE mean={s['mean']:.4f}, stdev={s['stdev']:.4f}")
        if 'brier_score' in stats_dict:
            s = stats_dict['brier_score']
            print(f"  brier_score: mean={s['mean']:.4f}, stdev={s['stdev']:.4f}")
    
    if analyses['fusion_mode_comparison']:
        print(f"\n=== Fusion Mode Comparison (ANOVA) ===")
        for metric, result in analyses['fusion_mode_comparison'].items():
            sig = "***" if result['significant'] else "ns"
            print(f"  {metric}: F={result['anova_f']:.2f}, p={result['anova_p']:.4f} {sig}")
    
    if analyses['mission_success_by_mode']:
        result = analyses['mission_success_by_mode']
        sig = "***" if result['significant'] else "ns"
        print(f"\n=== Mission Success Rates (χ²) ===")
        for mode, data in result['modes'].items():
            print(f"  {mode}: {data['success']}/{data['total']} ({data['success']/data['total']*100:.0f}%)")
        print(f"  χ²={result['chi2']:.2f}, p={result['p_value']:.4f} {sig}")
    
    if analyses['paired_naive_vs_trust']:
        print(f"\n=== Paired: Naive vs Trust-Weighted ===")
        for metric, result in analyses['paired_naive_vs_trust'].items():
            sig = "***" if result['significant'] else "ns"
            print(f"  {metric}: naive_mean={result['naive_mean']:.2f}, trust_mean={result['trust_mean']:.2f}, "
                  f"diff={result['mean_diff']:.2f}, p={result['p_value']:.4f} {sig}")
    
    print(f"\nResults written to: {args.output}")


if __name__ == "__main__":
    sys.exit(main())