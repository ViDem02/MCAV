# MCAV Project Specification — MuJoCo-only Modular Simulation + Control (URDF-only models)

## 0. Objectives

We want a modular research environment that supports:

- **Iterative complexity**
  1) Unicycle: path-following + Lyapunov analysis  
  2) More complex vehicles (eventually quadrotor / quadricopter)  
  3) Multi-vehicle formation control  
  4) (Stretch) Cooperative vehicle target tracking

- **Symbolic computations** (SymPy) to derive models / Lyapunov candidates and verify convergence conditions.
- **Numerical simulations** (**MuJoCo**) for validation, stress tests, and batch experiments.
- **Strong plotting pipeline** and an exportable **LaTeX report**.
- **Parameter flexibility** (easy tuning) and **optimization** (e.g., Optuna).

Constraints:
- **MuJoCo only** (no PyBullet or other engines).
- **URDF-only for robot models**: all robots must be authored and maintained as **URDF** (single source of truth).
- **ROS not required** (URDF is used purely as a model format, not implying ROS tooling).
- Must work well on **ARM mac (Apple Silicon, e.g., M3)**.
- Must support:
  - notebooks + importable Python modules
  - modular simulation wrappers (with/without video)
  - per-step structured logging
  - tqdm for any time-consuming process
  - strict separation of **simulation FPS** and **video FPS**

> Note: MuJoCo natively uses MJCF, but this project will treat **URDF as the authoritative model format** and will **load URDF into MuJoCo** (potentially via a conversion step/tooling if required). No hand-authored MJCF should be maintained as a source model.

---

## 1. Tooling & environment management

### 1.1 Package manager
Use **`uv`** for speed + clean ARM-native environments.

```bash
uv init mcav_project
cd mcav_project
uv add mujoco numpy scipy sympy matplotlib plotly pandas tqdm ipykernel jupyterlab pydantic pyyaml imageio[ffmpeg] optuna
uv add --dev pytest black ruff
```

---

## 2. Repository structure (URDF-only models)

```
mcav_project/
│
├── PEDRO_MATERIAL/              # theory slides/reference (read-only)
│
├── config/                      # all tunable parameters (YAML)
│   ├── sim_default.yaml
│   ├── unicycle.yaml
│   └── quadrotor.yaml
│
├── models/
│   ├── urdf/                    # URDF is the only source of truth
│   │   ├── unicycle.urdf
│   │   └── quadrotor.urdf
│   └── assets/                  # meshes/textures referenced by URDF
│
├── src/
│   ├── simulation/
│   │   ├── __init__.py
│   │   ├── base_sim.py          # abstract base wrapper
│   │   ├── headless_sim.py      # no rendering (fast)
│   │   ├── video_sim.py         # offscreen render -> video
│   │   ├── viewer_sim.py        # interactive live viewer
│   │   └── urdf_loader.py       # URDF -> MuJoCo model loading utilities
│   │
│   ├── vehicles/
│   │   ├── __init__.py
│   │   ├── unicycle.py
│   │   └── quadrotor.py
│   │
│   ├── control/
│   │   ├── __init__.py
│   │   ├── lyapunov.py
│   │   ├── path_following.py
│   │   ├── formation.py
│   │   └── tracking.py
│   │
│   ├── symbolic/
│   │   ├── __init__.py
│   │   └── kinematics.py
│   │
│   ├── logging/
│   │   ├── __init__.py
│   │   └── sim_logger.py        # JSON records -> DataFrame -> file
│   │
│   └── utils/
│       ├── __init__.py
│       ├── config.py
│       └── plotting.py
│
├── notebooks/
│   ├── 01_unicycle_path_following/
│   │   ├── 01_symbolic_derivation.ipynb
│   │   └── 02_simulation.ipynb
│   ├── 02_lyapunov_analysis/
│   └── 03_quadrotor_formation/
│
├── data/                        # auto-generated simulation logs (JSON)
├── videos/                      # auto-generated recordings (mp4)
│
├── report/                      # LaTeX report source
│   ├── main.tex
│   └── figures/                 # plots exported from notebooks
│
├── pyproject.toml
└── README.md
```

---

## 3. Core design: simulation wrappers

### 3.1 Wrapper concept
We will provide **multiple wrappers** around MuJoCo:

1. **HeadlessSim**: no rendering; fastest; for batch runs and optimization.
2. **VideoSim**: offscreen rendering; records video to file.
3. **ViewerSim**: interactive viewer; for debugging.

All wrappers share the same core idea:

- `BaseSim` owns: setup, stepping, timing, logging, teardown.
- You override a single function: **`_loop_step(t, step)`**.

### 3.2 Critical FPS requirement
- Simulation runs at physics timestep `dt` (e.g. 0.005 => 200 Hz).
- Video capture runs at independent `video_fps` (e.g. 30 FPS).
- Capture every:
  `capture_every = max(1, int(1.0 / (dt * video_fps)))`

---

## 4. Logging subsystem (per-step JSON + export)

### 4.1 Requirements
- At every simulation step:
  - accumulate fields into a JSON-like dict
  - **commit** it as a record (stored in memory)
- Provide:
  - JSON save/load
  - JSON -> `pandas.DataFrame` (table)

### 4.2 Reference implementation

```python
# src/logging/sim_logger.py
import json
from pathlib import Path
import pandas as pd

class SimLogger:
    def __init__(self):
        self._current: dict = {}
        self._records: list[dict] = []

    def log(self, data: dict):
        """Accumulate fields during a single simulation step."""
        self._current.update(data)

    def commit(self, t: float):
        """Seal the current step record. Called once per step by the wrapper."""
        self._current["t"] = t
        self._records.append(dict(self._current))
        self._current = {}

    def to_dataframe(self) -> pd.DataFrame:
        return pd.json_normalize(self._records)

    def save(self, path: str):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self._records, f, indent=2)

    @staticmethod
    def load(path: str) -> pd.DataFrame:
        with open(path) as f:
            return pd.json_normalize(json.load(f))
```

---

## 5. URDF-only loading into MuJoCo

### 5.1 Requirement
The simulation must start from a **URDF path** (e.g. `models/urdf/unicycle.urdf`), and MuJoCo must be initialized from that.

We will centralize this in `src/simulation/urdf_loader.py` so all wrappers use the same loading logic.

### 5.2 Expected API (design)
```python
# src/simulation/urdf_loader.py
# Goal: one function that returns (model, data) from a URDF path + optional assets root.
#
# def load_mujoco_model_from_urdf(urdf_path: str, *, assets_dir: str | None = None) -> mujoco.MjModel:
#     ...
```

Implementation details depend on the chosen URDF->MuJoCo loading strategy, but **the rest of the project must not care**: wrappers only call the loader and get a ready `MjModel`.

---

## 6. Base simulation wrapper (MuJoCo)

### 6.1 Responsibilities
- Load model from **URDF path** via the URDF loader
- Initialize `MjModel` / `MjData`
- Run stepping loop with `tqdm`
- Call `_loop_step(t, step)` every step (user-defined logic)
- Step physics via `mujoco.mj_step(...)`
- Commit log every step

```python
# src/simulation/base_sim.py
from abc import ABC
import mujoco
from tqdm import tqdm
from src.logging.sim_logger import SimLogger
from src.simulation.urdf_loader import load_mujoco_model_from_urdf

class BaseSim(ABC):
    def __init__(self, config: dict):
        self.cfg = config
        self.dt = float(config["dt"])
        self.t_end = float(config["t_end"])
        self.logger = SimLogger()

        # URDF-only constraint
        self.urdf_path = config["urdf_path"]
        self.model = load_mujoco_model_from_urdf(self.urdf_path, assets_dir=config.get("assets_dir"))
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

    # --- override this ---
    def _loop_step(self, t: float, step: int):
        """
        Apply controls, read state, call self.logger.log(dict).
        Do NOT call mj_step here — wrapper does it.
        """
        pass

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
        self._teardown()

    def _teardown(self):
        pass

    def to_dataframe(self):
        return self.logger.to_dataframe()

    def save(self, path: str):
        self.logger.save(path)
```

---

## 7. Specific wrappers

### 7.1 Headless wrapper (fast)
```python
# src/simulation/headless_sim.py
from .base_sim import BaseSim

class HeadlessSim(BaseSim):
    """Pure physics, no rendering. Use for optimization and batch runs."""
    pass
```

### 7.2 Video wrapper (offscreen render -> mp4)
Key properties:
- decoupled video FPS
- capture every N steps
- uses `imageio[ffmpeg]`
- `tqdm` progress bar remains on the simulation loop

```python
# src/simulation/video_sim.py
import pathlib
import mujoco
import imageio
from .base_sim import BaseSim

class VideoSim(BaseSim):
    """
    Offscreen rendering via mujoco.Renderer.
    Video FPS and simulation FPS are independent.
    """
    def __init__(self, config: dict):
        super().__init__(config)
        self.video_fps = int(config.get("video_fps", 30))
        self.video_path = config.get("video_path", "videos/output.mp4")
        self.width = int(config.get("video_w", 1280))
        self.height = int(config.get("video_h", 720))

        self.capture_every = max(1, int(1.0 / (self.dt * self.video_fps)))
        self._frames = []

        self._renderer = mujoco.Renderer(self.model, self.height, self.width)

    def run(self):
        n_steps = int(self.t_end / self.dt)
        from tqdm import tqdm
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
            if i % self.capture_every == 0:
                self._capture_frame()
        self._write_video()
        self._teardown()

    def _capture_frame(self):
        self._renderer.update_scene(self.data)
        frame = self._renderer.render()   # HxWx3 uint8 RGB
        self._frames.append(frame)

    def _write_video(self):
        pathlib.Path(self.video_path).parent.mkdir(parents=True, exist_ok=True)
        imageio.mimwrite(self.video_path, self._frames, fps=self.video_fps)

    def _teardown(self):
        self._renderer.close()
```

### 7.3 Interactive viewer wrapper (debugging)
```python
# src/simulation/viewer_sim.py
import mujoco
import mujoco.viewer
from tqdm import tqdm
from .base_sim import BaseSim

class ViewerSim(BaseSim):
    """Real-time interactive viewer. Not suitable for batch runs."""
    def run(self):
        n_steps = int(self.t_end / self.dt)
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
                t = i * self.dt
                self._loop_step(t, i)
                mujoco.mj_step(self.model, self.data)
                self.logger.commit(t)
                viewer.sync()
```

---

## 8. Config system (YAML + optional validation)

Example:

```yaml
# config/unicycle.yaml
urdf_path: "models/urdf/unicycle.urdf"
assets_dir: "models/assets"

dt: 0.005
t_end: 20.0

video_fps: 30
video_path: "videos/unicycle_run.mp4"
video_w: 1280
video_h: 720

control:
  k1: 1.0
  k2: 2.0
  lookahead: 0.5
```

Loader:

```python
# src/utils/config.py
import yaml

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)
```

---

## Running Notebooks

Quick steps to run the notebooks included in `notebooks/`.

- **Activate the project virtual environment** (example):

```bash
source .venv/bin/activate
# or (if your venv is elsewhere)
source mcav_project/.venv/bin/activate
```

- **Start an interactive session** (recommended):

```bash
jupyter lab
# or
jupyter notebook
```

Open the notebook you want from the `notebooks/` folder and run cells via the UI.

- **Run a notebook headless (non-interactive)** — executes all cells and writes an executed copy:

```bash
jupyter nbconvert --to notebook --execute notebooks/<NOTEBOOK>.ipynb --output executed/<NOTEBOOK>.ipynb
```

Replace `<NOTEBOOK>` with the filename (without path). Example:

```bash
jupyter nbconvert --to notebook --execute notebooks/quadcopter_kinematic.ipynb --output executed/quadcopter_kinematic.ipynb
```

Notebooks included in this repo (root `notebooks/`):

- `quadcopter_kinematic.ipynb`
- `quadcopter_lyapunov_circle.ipynb`
- `unicycle_circle.ipynb`
- `unicycle_fall_north.ipynb`
- `unicycle_lyapunov_circle_fixed.ipynb`
- `unicycle_lyapunov_circle.ipynb`

Notes:
- Ensure dependencies are installed in the active environment (see "Tooling & environment management" above).
- To ensure the notebook kernel matches the venv, you can (once) register the kernel:

```bash
python -m ipykernel install --user --name mcav_project --display-name "mcav_project"
```

The headless `nbconvert` command is useful for CI or automated runs; interactive `jupyter lab` is preferred for exploration and plotting.


## 9. Symbolic layer (SymPy)

Keep derivations in `.py` modules so notebooks can import them (avoid re-deriving).

```python
# src/symbolic/kinematics.py
from sympy import symbols, Matrix, cos, sin

def unicycle_model():
    x, y, theta, v, omega = symbols("x y θ v ω")
    f = Matrix([v*cos(theta), v*sin(theta), omega])
    return {"state": (x, y, theta), "input": (v, omega), "f": f}
```

---

## 10. Plotting + LaTeX report pipeline

- Notebooks produce plots
- Save plots to `report/figures/*.pdf`
- LaTeX includes them

```python
# src/utils/plotting.py
import matplotlib.pyplot as plt

def set_style():
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "figure.dpi": 150,
    })

def save_for_report(fig, name: str):
    fig.savefig(f"report/figures/{name}.pdf", bbox_inches="tight")
```

---

## 11. Optimization

Use **Optuna** for parameter tuning:
- run `HeadlessSim` in the objective function
- define a scalar metric (e.g., mean tracking error, settling time, energy)
- enable progress bars

---

## 12. Iterative roadmap

1. **Unicycle**
   - symbolic model + Lyapunov candidate
   - path-following simulation
   - logging + plots + initial report section

2. **Lyapunov refinements**
   - robustness experiments
   - parameter sweeps + Optuna tuning

3. **Quadrotor**
   - URDF model + stabilization + trajectory tracking
   - logging/plots/report update

4. **Formation + cooperative tracking**
   - N vehicles, formation constraints
   - scalable experiments and reproducible report figures