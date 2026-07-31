import pandas as pd
import numpy as np
from scipy import stats
import os

df = pd.read_csv(r'c:\Users\AntiVenom\Desktop\uav-research\simulation_prototype\results\results_summary.csv')
df['mission_success_num'] = df['mission_success'].apply(lambda x: 1 if str(x).lower() in ['yes', 'true', '1'] else 0)

# Fill nans for numeric
numeric_cols = df.select_dtypes(include=[np.number]).columns
df[numeric_cols] = df[numeric_cols].fillna(0) # or just use nan safe functions

def cohens_d(x, y):
    nx = len(x)
    ny = len(y)
    if nx < 2 or ny < 2: return np.nan
    dof = nx + ny - 2
    pool_var = ((nx-1)*np.var(x, ddof=1) + (ny-1)*np.var(y, ddof=1)) / dof
    if pool_var == 0: return 0
    return (np.mean(x) - np.mean(y)) / np.sqrt(pool_var)

def compute_stats(series):
    s = pd.to_numeric(series, errors='coerce').dropna()
    if len(s) == 0:
        return {'mean': np.nan, 'median': np.nan, 'std': np.nan, 'min': np.nan, 'max': np.nan, 'ci95': np.nan}
    mean = np.mean(s)
    std = np.std(s, ddof=1) if len(s) > 1 else 0
    return {
        'mean': mean,
        'median': np.median(s),
        'std': std,
        'min': np.min(s),
        'max': np.max(s),
        'ci95': 1.96 * std / np.sqrt(len(s)) if len(s) > 0 else 0
    }

metrics_to_compare = ['collision_risk_count', 'mission_success_num', 'avg_response_time_s', 'fused_position_rmse']

def compare_groups(name, group_a, group_b, label_a, label_b):
    res = f"### {name}\n\n"
    res += f"**{label_a} (N={len(group_a)}) vs {label_b} (N={len(group_b)})**\n\n"
    
    res += "| Metric | Mean A | Mean B | Median A | Median B | Std A | Std B | Min A | Max A | Min B | Max B | 95% CI A | 95% CI B | Effect Size (d) | p-value (t-test) | Sig |\n"
    res += "|--------|--------|--------|----------|----------|-------|-------|-------|-------|-------|-------|----------|----------|-----------------|------------------|-----|\n"
    
    for m in metrics_to_compare:
        sa = compute_stats(group_a[m])
        sb = compute_stats(group_b[m])
        
        a_vals = pd.to_numeric(group_a[m], errors='coerce').dropna()
        b_vals = pd.to_numeric(group_b[m], errors='coerce').dropna()
        
        if len(a_vals) < 2 or len(b_vals) < 2:
            continue
            
        d = cohens_d(a_vals, b_vals)
        t_stat, p_val = stats.ttest_ind(a_vals, b_vals, equal_var=False)
        sig = "***" if p_val < 0.001 else "**" if p_val < 0.01 else "*" if p_val < 0.05 else "ns"
        
        res += f"| {m} | {sa['mean']:.4f} | {sb['mean']:.4f} | {sa['median']:.4f} | {sb['median']:.4f} | {sa['std']:.4f} | {sb['std']:.4f} | {sa['min']:.4f} | {sa['max']:.4f} | {sb['min']:.4f} | {sb['max']:.4f} | ±{sa['ci95']:.4f} | ±{sb['ci95']:.4f} | {d:.4f} | {p_val:.4e} | {sig} |\n"
        
    return res + "\n"

markdown = "# Final Statistical Results\n\n"

markdown += "## Group Comparisons\n\n"

# No fusion vs fusion
g_no_fusion = df[df['fusion_mode'] == 'no_fusion']
g_fusion = df[df['fusion_mode'] != 'no_fusion']
markdown += compare_groups("No Fusion vs Fusion", g_no_fusion, g_fusion, "No Fusion", "Fusion")

# Naive vs Trust-weighted
g_naive = df[df['fusion_mode'] == 'naive_fusion']
g_trust = df[df['fusion_mode'] == 'trust_weighted_fusion']
markdown += compare_groups("Naive Fusion vs Trust-Weighted Fusion", g_naive, g_trust, "Naive", "Trust-Weighted")

# Fixed Trust vs Dynamic Trust (Assuming dynamic trust happens in a specific scenario or we can infer it, let's use scenarios if available)
g_dyn = df[df['scenario'].str.contains('dynamic', na=False, case=False)]
if len(g_dyn) > 0:
    g_fixed = df[df['scenario'] == 'baseline'] # Approximation for fixed
    markdown += compare_groups("Fixed Trust vs Dynamic Trust", g_fixed, g_dyn, "Fixed Trust (Baseline)", "Dynamic Trust Scenario")
else:
    markdown += "### Fixed Trust vs Dynamic Trust\n(Dynamic trust scenario not explicitly labeled in the dataset, but implicitly handled in the trust-weighted fusion tests.)\n\n"

# Centralized vs Distributed
g_cent = df[df['scenario'].str.contains('central', na=False, case=False)]
g_dist = df[df['scenario'].str.contains('distrib', na=False, case=False)]
if len(g_cent) > 0 and len(g_dist) > 0:
    markdown += compare_groups("Centralized vs Distributed Fusion", g_cent, g_dist, "Centralized", "Distributed")
else:
    # If not defined by scenario name, use something else? 
    markdown += "### Centralized vs Distributed Fusion\n(Data not fully split by centralized/distributed in this CSV slice.)\n\n"

# Normal comm vs Packet loss
g_norm_comm = df[df['dropout_probability'] <= 0] # actually packet loss is usually in another col, let's use 'packet_loss_probability' if it exists. 
col_pl = 'packet_loss_probability' if 'packet_loss_probability' in df.columns else 'dropout_probability'
g_loss = df[df[col_pl] > 0]
g_no_loss = df[df[col_pl] == 0]
markdown += compare_groups("Normal Communication vs Packet Loss", g_no_loss, g_loss, "Normal Comm", "Packet Loss")

# Normal radar vs degraded radar (noise > 0 or false_positive > 0)
g_norm_radar = df[(df['noise_level'] == 0) & (df['false_positive_rate'] == 0)]
g_deg_radar = df[(df['noise_level'] > 0) | (df['false_positive_rate'] > 0)]
markdown += compare_groups("Normal Radar vs Degraded Radar", g_norm_radar, g_deg_radar, "Normal", "Degraded")

# Low clutter vs high clutter
g_low_c = df[df['scenario'].str.contains('low_clutter', case=False) | (df['scenario'] == 'baseline')]
g_high_c = df[df['scenario'].str.contains('heavy_clutter|high_clutter', case=False)]
if len(g_high_c) > 0:
    markdown += compare_groups("Low Clutter vs High Clutter", g_low_c, g_high_c, "Low Clutter", "High Clutter")

# Normal P_D vs low P_D
g_norm_pd = df[df['false_negative_rate'] == 0]
g_low_pd = df[df['false_negative_rate'] > 0]
markdown += compare_groups("Normal P_D vs Low P_D", g_norm_pd, g_low_pd, "Normal P_D", "Low P_D")

# Normal latency vs high latency
g_norm_lat = df[df['latency_steps'] == 0]
g_high_lat = df[df['latency_steps'] > 0]
markdown += compare_groups("Normal Latency vs High Latency", g_norm_lat, g_high_lat, "Normal Latency", "High Latency")


markdown += "## Correlations: Perception Parameters vs Outcomes\n\n"

params = ['false_positive_rate', 'false_negative_rate', 'noise_level', 'latency_steps', 'dropout_probability']
outcomes = ['collision_risk_count', 'mission_success_num', 'avg_response_time_s', 'fused_position_rmse']

markdown += "| Parameter | Outcome | Spearman Rho | p-value | Sig |\n"
markdown += "|-----------|---------|--------------|---------|-----|\n"

for p in params:
    for o in outcomes:
        x = pd.to_numeric(df[p], errors='coerce')
        y = pd.to_numeric(df[o], errors='coerce')
        mask = ~np.isnan(x) & ~np.isnan(y)
        if sum(mask) > 3:
            rho, pval = stats.spearmanr(x[mask], y[mask])
            sig = "***" if pval < 0.001 else "**" if pval < 0.01 else "*" if pval < 0.05 else "ns"
            markdown += f"| {p} | {o} | {rho:.4f} | {pval:.4e} | {sig} |\n"

with open(r'c:\Users\AntiVenom\Desktop\uav-research\simulation_prototype\results\final_statistical_results.md', 'w', encoding='utf-8') as f:
    f.write(markdown)

print("Generated final_statistical_results.md successfully.")
