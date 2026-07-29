import os
import shutil
import glob

def create_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def copy_file(src, dst_dir):
    if os.path.exists(src):
        print(f"Copying {src} -> {dst_dir}")
        shutil.copy(src, dst_dir)
    else:
        print(f"Warning: {src} not found")

def main():
    root = os.path.dirname(os.path.abspath(__file__))
    release_dir = os.path.join(root, "results", "final_release")
    
    # Create structure
    create_dir(release_dir)
    create_dir(os.path.join(release_dir, "plots"))
    create_dir(os.path.join(release_dir, "videos"))
    create_dir(os.path.join(release_dir, "config"))

    # 1. Config files
    copy_file(os.path.join(root, "simulation_config.json"), os.path.join(release_dir, "config"))
    copy_file(os.path.join(root, "results", "final", "run_metadata.json"), os.path.join(release_dir, "config"))
    
    # 2. Results & Indices (which have means, medians, std, CI)
    final_src_dir = os.path.join(root, "results", "final")
    copy_file(os.path.join(final_src_dir, "raw_run_index.csv"), release_dir)
    copy_file(os.path.join(final_src_dir, "aggregated_metrics.csv"), release_dir)
    copy_file(os.path.join(final_src_dir, "scenario_summary.csv"), release_dir)
    copy_file(os.path.join(final_src_dir, "statistical_comparisons.csv"), release_dir)
    copy_file(os.path.join(final_src_dir, "failed_run_report.csv"), release_dir)
    copy_file(os.path.join(final_src_dir, "README.md"), release_dir)

    # 3. Selected plots
    plots_final = glob.glob(os.path.join(root, "plots", "final", "*.png"))
    for p in plots_final:
        copy_file(p, os.path.join(release_dir, "plots"))
    
    plots_depend = glob.glob(os.path.join(root, "plots", "dependability", "*.png"))
    for p in plots_depend:
        copy_file(p, os.path.join(release_dir, "plots"))
        
    # 4. Selected videos
    # Only copy a diverse representative selection or all if there aren't too many
    videos = glob.glob(os.path.join(root, "media", "**", "*.mp4"), recursive=True)
    # Filter to avoid too many redundant ones, we will pick 10 representative videos
    target_videos = [
        "baseline_video.mp4",
        "dynamic_trust_adaptation.mp4",
        "centralized_vs_distributed_fusion.mp4",
        "fusion_comparison_video.mp4",
        "clutter_stress_test.mp4",
        "communication_outage_scenario.mp4",
        "target_crossing_example.mp4",
        "calibrated_vs_overconfident_radar.mp4",
        "combined_fault_recovery.mp4",
        "swarm_size_comparison.mp4"
    ]
    
    for v in videos:
        if os.path.basename(v) in target_videos:
            copy_file(v, os.path.join(release_dir, "videos"))

    print(f"\nFinal release assembled at {release_dir}")

if __name__ == "__main__":
    main()
