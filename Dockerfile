# --- UAV Radar-Swarm Simulation: Docker image ---
# Default entrypoint is the interactive tkinter GUI (run_interactive_demo.py).
# This REQUIRES a display to be forwarded into the container at `docker run` time
# (e.g. WSLg on Windows 11, or an X server) — it will fail to open a window otherwise.
# For headless/reproducible batch runs, override the command at runtime, e.g.:
#   docker run --rm -e MPLBACKEND=Agg -v $(pwd)/results:/app/simulation_prototype/results \
#       uav-sim python run_final_demo.py

FROM python:3.11-slim

# ffmpeg: optional, only needed if you call simulation_visualizer.py to render .mp4s.
# Safe to keep; the code checks `shutil.which("ffmpeg")` and skips video export if absent.
# python3-tk: required for run_interactive_demo.py (provides libtk8.6.so for tkinter).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ffmpeg \
        python3-tk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install deps first so this layer is cached across code changes
COPY simulation_prototype/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

ENV PYTHONUNBUFFERED=1
# NOTE: no MPLBACKEND override here — leaving it unset lets matplotlib pick TkAgg
# automatically now that tkinter/Tk is installed, which the GUI needs to render.
# Set MPLBACKEND=Agg explicitly (see batch example above) for headless/batch runs.

# Copy only the simulation code (media/, __pycache__, notes, final_acceptance are
# excluded via .dockerignore — see below)
COPY simulation_prototype/ ./simulation_prototype/

WORKDIR /app/simulation_prototype

# Default: launch the interactive GUI. Needs a display forwarded in, e.g. on WSL2/WSLg:
#   docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v /mnt/wslg:/mnt/wslg -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
#       -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR uav-sim
# (run from inside a WSL terminal, not cmd.exe/PowerShell)
CMD ["python", "run_interactive_demo.py"]