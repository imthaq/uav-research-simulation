import os
import sys
import json
import copy
import tkinter as tk
from tkinter import ttk, messagebox

_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from simulation_visualizer import _run_full_stack, SimulationVisualizer
try:
    from simulation_visualizer_3d import SimulationVisualizer3D
    HAS_3D = True
except ImportError:
    HAS_3D = False

class InteractiveDemoApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UAV Swarm Simulation - Interactive Demo")
        self.root.geometry("800x600")

        # Load default config
        self.config_path = os.path.join(_ROOT_DIR, "simulation_config.json")
        with open(self.config_path, 'r') as f:
            self.base_config = json.load(f)

        self.create_widgets()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # -----------------------------------------------------
        # Left Panel (Parameters)
        # -----------------------------------------------------
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        row = 0
        ttk.Label(left_frame, text="Scenario Settings", font=("Arial", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        row += 1
        ttk.Label(left_frame, text="Scenario:").grid(row=row, column=0, sticky=tk.W)
        self.var_scenario = tk.StringVar(value="baseline")
        scenarios = list(self.base_config.get("scenarios", {}).keys())
        ttk.Combobox(left_frame, textvariable=self.var_scenario, values=scenarios, state="readonly").grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Random Seed:").grid(row=row, column=0, sticky=tk.W)
        self.var_seed = tk.IntVar(value=self.base_config["sim"]["seed"])
        ttk.Entry(left_frame, textvariable=self.var_seed).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Radar Settings", font=("Arial", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=(15,5))

        row += 1
        ttk.Label(left_frame, text="P_D (Detection Prob):").grid(row=row, column=0, sticky=tk.W)
        self.var_pd = tk.DoubleVar(value=1.0)
        ttk.Scale(left_frame, from_=0.0, to=1.0, variable=self.var_pd, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="P_FA (False Alarm Prob):").grid(row=row, column=0, sticky=tk.W)
        self.var_pfa = tk.DoubleVar(value=0.0)
        ttk.Scale(left_frame, from_=0.0, to=1.0, variable=self.var_pfa, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Clutter Level:").grid(row=row, column=0, sticky=tk.W)
        self.var_clutter = tk.DoubleVar(value=0.0)
        ttk.Scale(left_frame, from_=0.0, to=10.0, variable=self.var_clutter, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Noise Level (Std):").grid(row=row, column=0, sticky=tk.W)
        self.var_noise = tk.DoubleVar(value=0.3)
        ttk.Scale(left_frame, from_=0.0, to=5.0, variable=self.var_noise, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Latency (Steps):").grid(row=row, column=0, sticky=tk.W)
        self.var_latency = tk.IntVar(value=0)
        ttk.Scale(left_frame, from_=0, to=20, variable=self.var_latency, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Dropout Prob:").grid(row=row, column=0, sticky=tk.W)
        self.var_dropout = tk.DoubleVar(value=0.0)
        ttk.Scale(left_frame, from_=0.0, to=1.0, variable=self.var_dropout, orient=tk.HORIZONTAL).grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Fusion & Comm Settings", font=("Arial", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=(15,5))

        row += 1
        ttk.Label(left_frame, text="Fusion Mode:").grid(row=row, column=0, sticky=tk.W)
        self.var_fusion = tk.StringVar(value="naive_fusion")
        ttk.Combobox(left_frame, textvariable=self.var_fusion, values=["no_fusion", "naive_fusion", "confidence_weighted_fusion", "trust_weighted_fusion", "covariance_weighted_fusion"], state="readonly").grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Trust Mode:").grid(row=row, column=0, sticky=tk.W)
        self.var_trust = tk.StringVar(value="fixed")
        ttk.Combobox(left_frame, textvariable=self.var_trust, values=["fixed", "dynamic"], state="readonly").grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Architecture:").grid(row=row, column=0, sticky=tk.W)
        self.var_arch = tk.StringVar(value="centralized")
        ttk.Combobox(left_frame, textvariable=self.var_arch, values=["centralized", "distributed"], state="readonly").grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Comm Condition:").grid(row=row, column=0, sticky=tk.W)
        self.var_comm = tk.StringVar(value="perfect")
        ttk.Combobox(left_frame, textvariable=self.var_comm, values=["perfect", "low_packet_loss", "high_packet_loss", "outage"], state="readonly").grid(row=row, column=1, sticky=tk.EW)

        row += 1
        ttk.Label(left_frame, text="Display Settings", font=("Arial", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=(15,5))

        row += 1
        ttk.Label(left_frame, text="Visualization:").grid(row=row, column=0, sticky=tk.W)
        self.var_3d = tk.BooleanVar(value=HAS_3D)
        view_frame = ttk.Frame(left_frame)
        view_frame.grid(row=row, column=1, sticky=tk.W)
        ttk.Radiobutton(view_frame, text="2D", variable=self.var_3d, value=False).pack(side=tk.LEFT)
        rb3d = ttk.Radiobutton(view_frame, text="3D", variable=self.var_3d, value=True)
        rb3d.pack(side=tk.LEFT)
        if not HAS_3D: rb3d.state(['disabled'])

        row += 1
        run_btn = ttk.Button(left_frame, text="Run Simulation", command=self.on_run)
        run_btn.grid(row=row, column=0, columnspan=2, pady=20)

        # -----------------------------------------------------
        # Right Panel (Output / Summary)
        # -----------------------------------------------------
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=20)

        ttk.Label(right_frame, text="Final Summary", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=5)
        self.txt_summary = tk.Text(right_frame, height=20, width=50, state=tk.DISABLED)
        self.txt_summary.pack(fill=tk.BOTH, expand=True)

    def on_run(self):
        cfg = copy.deepcopy(self.base_config)
        scenario = self.var_scenario.get()
        if scenario not in cfg["scenarios"]:
            cfg["scenarios"][scenario] = {}
            
        s_cfg = cfg["scenarios"][scenario]
        
        # Apply overrides
        cfg["sim"]["seed"] = self.var_seed.get()
        
        
        cfg["radar"]["radar_detection_probability"] = self.var_pd.get()
        cfg["radar"]["radar_false_alarm_probability"] = self.var_pfa.get()
        cfg["radar"]["radar_clutter_density"] = self.var_clutter.get()
        cfg["radar"]["radar_range_noise_std"] = self.var_noise.get()
        cfg["radar"]["radar_latency_steps"] = self.var_latency.get()
        cfg["radar"]["radar_dropout_probability"] = self.var_dropout.get()
        
        # Communication
        comm = self.var_comm.get()
        if comm == "perfect":
            s_cfg["packet_loss_probability"] = 0.0
            s_cfg["communication_outage_probability"] = 0.0
        elif comm == "low_packet_loss":
            s_cfg["packet_loss_probability"] = 0.1
        elif comm == "high_packet_loss":
            s_cfg["packet_loss_probability"] = 0.5
        elif comm == "outage":
            s_cfg["communication_outage_probability"] = 0.05
            
        # Hide GUI to let Matplotlib take focus
        self.root.withdraw()
        
        # We run the simulation in a background thread to keep Tkinter responsive,
        # but matplotlib's interactive window MUST be in the main thread (on Mac/Windows).
        # However, _run_full_stack doesn't use matplotlib. It just builds data.
        # So we can just block the main thread since _run_full_stack is fast (~1s).
        
        arch = self.var_arch.get()
        fm = self.var_fusion.get()
        tm = self.var_trust.get()
        use_3d = self.var_3d.get()
        
        # Re-show the main window after a short delay so matplotlib can close properly
        def simulation_routine():
            try:
                sim_data, radar_data, fused_data, radar_model = _run_full_stack(
                    cfg, scenario, architecture=arch, seed=cfg["sim"]["seed"],
                    fusion_mode_override=fm, use_adaptive_trust=(tm == "dynamic")
                )
                
                if use_3d and HAS_3D:
                    viz = SimulationVisualizer3D(sim_data, radar_data=radar_data, fused_data=fused_data)
                else:
                    viz = SimulationVisualizer(sim_data)
                    viz.radar_data = radar_data
                    viz.fused_data = fused_data
                    
                viz.add_legend()
                import matplotlib.pyplot as plt
                plt.show(block=False)
                interval = 1.0 / 10.0
                for step in range(sim_data.steps):
                    viz.render_step(step)
                    plt.pause(interval)
                plt.pause(1.0)
                plt.close(viz.fig)
                
                # Fetch metrics
                t = sim_data.steps - 1
                metrics = radar_model.sim._metrics(t)
                
                self.show_summary(metrics, cfg)
            except Exception as e:
                import traceback
                traceback.print_exc()
                messagebox.showerror("Error", str(e))
            finally:
                self.root.deiconify()

        # Execute
        simulation_routine()
        
    def show_summary(self, metrics, cfg):
        self.txt_summary.config(state=tk.NORMAL)
        self.txt_summary.delete(1.0, tk.END)
        
        out = "=== SIMULATION FINISHED ===\n\n"
        out += f"Mission Success: {metrics.get('mission_success')}\n"
        out += f"Collision Count: {metrics.get('collision_count')}\n"
        out += f"Near Misses: {metrics.get('total_near_misses')}\n"
        out += f"Completion Time (s): {metrics.get('mission_completion_time_s')}\n"
        out += f"Formation Error RMSE: {metrics.get('formation_error_rmse')}\n"
        
        out += "\n--- SELECTED CONFIGURATION ---\n"
        out += f"Scenario: {self.var_scenario.get()}\n"
        out += f"Seed: {cfg['sim']['seed']}\n"
        out += f"Architecture: {self.var_arch.get()}\n"
        out += f"Fusion: {self.var_fusion.get()}\n"
        out += f"Trust: {self.var_trust.get()}\n"
        
        out += "\n--- STATUS ---\n"
        out += "Sensors: OK\n"
        out += f"Communication: {self.var_comm.get()}\n"
        
        out += "\nNote: Visualizer window has been closed.\n"
        out += f"Files available at: {os.path.join(_ROOT_DIR, 'results')}\n"
        
        self.txt_summary.insert(tk.END, out)
        self.txt_summary.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = InteractiveDemoApp(root)
    root.mainloop()
