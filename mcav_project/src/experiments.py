"""
src/experiments.py

Headless experiments behind every number and figure of the report.

load()                config/<name>.yaml with paths resolved
simulate_quad()       closed-loop quadrotor in MuJoCo (circle or path following)
roa_monte_carlo()     random initial positions/attitudes → convergence statistics
render_quad()         replay a simulate_quad() run in MuJoCo's renderer → mp4
"""

from __future__ import annotations

from pathlib import Path
from typing import cast

import mujoco
import numpy as np
import pandas as pd
import yaml
from joblib import Parallel, delayed
from scipy.spatial.transform import Rotation

from src.control.cascade_controller import GeometricController, circle_reference
from src.control.path_following_controller import CirclePath, PathFollowingController

ROOT = Path(__file__).resolve().parents[1]


def load(name: str) -> dict:
    cfg = yaml.safe_load((ROOT / "config" / f"{name}.yaml").read_text())
    if "model_path" in cfg:
        cfg["model_path"] = str(ROOT / cfg["model_path"])
    return cfg


def load_model(cfg: dict) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(cfg["model_path"])
    model.opt.timestep = cfg["dt"]
    return model


# ──────────────────────────────────────────────────────────────────────────────
# Quadrotor
# ──────────────────────────────────────────────────────────────────────────────

def simulate_quad(
    cfg: dict,
    gains: dict | None = None,
    feedforward: bool = True,
    reference: str = "circle",              # "circle" | "path"
    t_end: float | None = None,
    pos0: np.ndarray | None = None,
    R0: Rotation | None = None,
    stop_on_ground: bool = False,
) -> pd.DataFrame:

    
    """One closed-loop run; one row per control step."""
    model = load_model(cfg)
    data = mujoco.MjData(model)
    ic = cfg["initial_state"]
    data.qpos[:3] = ic["pos"] if pos0 is None else pos0
    data.qpos[3:7] = (R0 or Rotation.from_euler("z", ic["yaw"])).as_quat(scalar_first=True)
    ctrl = GeometricController(model, gains or cfg["controller"], feedforward)

    traj = cfg["trajectory"]
    pfc = None
    if reference == "path":
        pfc = PathFollowingController.from_config(cfg, CirclePath(traj["radius"], traj["z_height"]))
        pfc.reset(data.qpos[:3])

    rows = []
    for i in range(int((t_end or cfg["t_end"]) / cfg["dt"])):
        t = i * cfg["dt"]
        if pfc is None:
            ref = circle_reference(t, traj["radius"], traj["omega"], traj["z_height"])
        else:
            ref, _, _ = pfc.step(data.qpos[:3], data.qvel[:3], cfg["dt"])
        data.ctrl[:], log = ctrl.compute(data, ref)
        if not np.all(np.isfinite(data.ctrl)) or (stop_on_ground and data.qpos[2] < 0.05):
            break
        rows.append({"t": t, "x": data.qpos[0], "y": data.qpos[1], "z": data.qpos[2],
                     **dict(zip(("qw", "qx", "qy", "qz"), data.qpos[3:7])),
                     "e_norm": float(np.linalg.norm(data.qpos[:3] - ref.pos)), **log})
        mujoco.mj_step(model, data)
    return pd.DataFrame(rows)


def _roa_rollout(cfg, gains, offset, rotvec, mc, f_max):
    traj = cfg["trajectory"]
    p0 = circle_reference(0.0, traj["radius"], traj["omega"], traj["z_height"]).pos
    R0 = Rotation.from_rotvec(rotvec) * Rotation.from_euler("z", cfg["initial_state"]["yaw"])
    df = simulate_quad(cfg, gains, True, "circle", mc["t_end"], p0 + offset, R0, stop_on_ground=True)
    f = df[["f1", "f2", "f3", "f4"]].to_numpy()
    finished = df["t"].iloc[-1] >= mc["t_end"] - 0.1
    return {
        "offset": float(np.linalg.norm(offset)),
        "angle_deg": float(np.degrees(np.linalg.norm(rotvec))),
        "V2_0": float(df["V2"].iloc[1]),                 # after Ω_d is initialised
        "converged": bool(finished and df["e_norm"].iloc[-1] < mc["tol"] and df["Psi"].iloc[-1] < 1e-4),
        "hit_ground": not finished,
        "feasible": bool(f.min() >= 0.0 and f.max() <= f_max),
    }


def roa_monte_carlo(cfg: dict, gains: dict | None = None, n: int | None = None, n_jobs: int = -1) -> pd.DataFrame:
    """
    Initial position offset uniform in a ball around p_d(0); initial attitude =
    nominal attitude perturbed by a uniform angle about a uniformly random axis;
    zero velocities.  "converged": ‖e_p‖ < tol and Ψ < 1e-4 at t_end.
    "feasible": every motor force stayed inside the motor range of the MJCF.
    """
    mc = cfg["monte_carlo"]
    n = n or mc["n"]
    rng = np.random.default_rng(mc["seed"])
    unit = lambda v: v / np.linalg.norm(v, axis=1, keepdims=True)
    offsets = unit(rng.normal(size=(n, 3))) * mc["max_offset"] * rng.uniform(size=(n, 1)) ** (1 / 3)
    rotvecs = unit(rng.normal(size=(n, 3))) * np.radians(mc["max_angle_deg"]) * rng.uniform(size=(n, 1))
    f_max = float(load_model(cfg).actuator_ctrlrange[:, 1].min())
    rows = cast(list[dict], Parallel(n_jobs=n_jobs)(        # joblib's return type is untyped
        delayed(_roa_rollout)(cfg, gains, offsets[i], rotvecs[i], mc, f_max) for i in range(n)))
    return pd.DataFrame(rows)


def _add_segment(scene: mujoco.MjvScene, a, b, rgba, width: float = 0.015) -> None:
    if scene.ngeom < scene.maxgeom:
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, float))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a, float), np.asarray(b, float))
        scene.ngeom += 1


def render_quad(
    cfg: dict,
    df: pd.DataFrame,
    path: str | Path,
    fps: int = 30,
    width: int = 960,
    height: int = 540,
    t_end: float | None = None,
    distance: float = 3.5,
) -> Path:
    """
    Replay the logged states of simulate_quad() (no new simulation) with a camera
    tracking the vehicle; reference circle in grey, flown path in blue.
    """
    import imageio.v3 as iio

    model = load_model(cfg)
    data = mujoco.MjData(model)
    tr = cfg["trajectory"]
    th = np.linspace(0, 2 * np.pi, 121)
    circle = np.column_stack([tr["radius"] * np.cos(th), tr["radius"] * np.sin(th), np.full_like(th, tr["z_height"])])
    t = df["t"].to_numpy()
    idx = np.searchsorted(t, np.arange(0.0, min(t_end or t[-1], t[-1]), 1.0 / fps))
    pos, quat = df[["x", "y", "z"]].to_numpy(), df[["qw", "qx", "qy", "qz"]].to_numpy()

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = model.body("quad").id
    cam.distance, cam.azimuth, cam.elevation = distance, -60.0, -20.0

    frames = []
    with mujoco.Renderer(model, height, width, max_geom=len(idx) + 500) as renderer:
        for k, i in enumerate(idx):
            data.qpos[:3], data.qpos[3:7] = pos[i], quat[i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, cam)
            for a, b in zip(circle[:-1], circle[1:]):
                _add_segment(renderer.scene, a, b, (0.4, 0.4, 0.4, 0.6))
            for a, b in zip(pos[idx[:k]], pos[idx[1:k + 1]]):
                _add_segment(renderer.scene, a, b, (0.12, 0.37, 0.66, 1.0))
            frames.append(renderer.render())

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, np.stack(frames), fps=fps, codec="libx264", macro_block_size=2)
    return path
