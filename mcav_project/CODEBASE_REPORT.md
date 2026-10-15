# mcav_project

## Directory Tree

```text
mcav_project/
├── config/
│   ├── quadcopter.yaml
│   └── unicycle.yaml
├── models/
│   └── quadcopter/
│       └── quadcopter.xml
├── results/
│   ├── monte_carlo.csv
│   ├── numbers.tex
│   └── summary.json
├── src/
│   ├── control/
│   │   ├── __init__.py
│   │   ├── cascade_controller.py
│   │   └── path_following_controller.py
│   ├── design/
│   │   ├── __init__.py
│   │   └── lmi_gains.py
│   ├── symbolic/
│   │   ├── lyapunov.py
│   │   ├── quadcopter_certified_lyapunov.py
│   │   └── quadcopter_path_following.py
│   └── experiments.py
├── tests/
│   ├── conftest.py
│   ├── test_closed_loop.py
│   ├── test_lmi.py
│   └── test_symbolic.py
├── .gitignore
├── main.py
├── MUJOCO_LOG.TXT
└── pyproject.toml
```

## File Contents

### `.gitignore`

```gitignore
pedro_material
__pycache__/
.ipynb_checkpoints/
.venv/
results/
```

### `config/quadcopter.yaml`

```yaml
# config/quadcopter.yaml
# Vehicle physics (mass, inertia, motor geometry and limits) live in the MJCF
# model only; this file holds the mission, the gains and the design choices.

model_path: "models/quadcopter/quadcopter.xml"
dt: 0.002            # physics and control period [s] (500 Hz)
t_end: 40.0          # mission length [s]

trajectory:          # p_d(t) = [R cos ωt, R sin ωt, z_d]
  radius:   3.0
  omega:    0.5
  z_height: 3.0

initial_state:       # at rest, level, heading tangent to the circle at t = 0
  pos: [0.0, 0.0, 0.5]
  yaw: 1.5707963267948966

# Hand-tuned gains (used when the LMI design is not run).
# Attitude: roll/pitch critically damped at 10 rad/s, kR = J_xy ωn², kw = 2 ζ ωn J_xy.
controller:
  kx: 1.0
  ky: 1.0
  kz: 1.5
  kv: 4.0
  kR: 0.35
  kw: 0.07
  saturate: false    # analysis assumes unbounded, signed motor forces

# LMI gain synthesis (src/design/lmi_gains.py)
design:
  torque_reserve_per_motor: 1.5   # N of each motor's range reserved for torques
  torque_split: [0.3, 0.3, 0.4]   # share of the reserve for |τx|/(4d), |τy|/(4d), |τz|/(4k_m)
  pd_torque_fraction: 0.8         # share of each torque budget for the PD part (rest: gyro + feedforward)
  accel_budget_xy: 3.0            # m/s² horizontal correction budget
  min_position_envelope: 0.5      # m     envelope certified in addition to the mission start
  min_velocity_envelope: 0.5      # m/s
  min_attitude_envelope_deg: 10.0
  min_rate_envelope: 0.5          # rad/s
  alpha_max_position: 2.0         # 1/s  cap: keeps the position loop slower than the attitude loop
  alpha_max_attitude: 20.0        # 1/s  cap: far below the 500 Hz control rate

monte_carlo:
  n: 500
  max_offset: 2.0                 # m    initial position offset (uniform in a ball)
  max_angle_deg: 175.0            #      initial attitude error (uniform angle, random axis)
  t_end: 20.0                     # s
  tol: 0.05                       # m    convergence threshold on ‖e_p‖
  seed: 7

path_following:
  v_d:     0.5       # nominal path rate [rad/s] — same as trajectory.omega
  k_gamma: 0.5       # along-track correction gain [rad/(s·m)]
```

### `config/unicycle.yaml`

```yaml
# config/unicycle.yaml — kinematic unicycle, Lyapunov path following on a circle
t_end: 15.0

control:
  v: 1.0          # constant forward speed [m/s]
  R: 1.5          # circle radius [m]
  ky: 1.0
  ktheta: 2.0

initial_state:    # outside the certified set V < R²/2 (see src/symbolic/lyapunov.py)
  x: 3.0
  y: 0.0
  theta: 2.356194490192345   # 3π/4
```

### `main.py`

```python
"""
main.py — reproduce every result of the report with one command.

    uv run python main.py            # full run  (~2 min on a laptop)
    uv run python main.py --quick    # smaller Monte Carlo (~30 s)

Steps
  1. symbolic verification of the stability analysis (SymPy)
  2. LMI synthesis of actuator-aware gains (cvxpy / Clarabel)
  3. nominal mission: full geometric law vs PD-only law, LMI gains vs paper gains
  4. Monte Carlo over initial positions and attitudes
  5. unicycle path following
Outputs go to results/: summary.json, numbers.tex (LaTeX macros used by the
report) and figures/*.pdf|png.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.design.lmi_gains import design_cascade_gains
from src.experiments import load, load_model, roa_monte_carlo, simulate_quad, simulate_unicycle
from src.symbolic.lyapunov import derive_circular_path_following
from src.symbolic.quadcopter_certified_lyapunov import run_all_checks
from src.symbolic.quadcopter_path_following import derive_path_following

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results"
FIG = OUT / "figures"
C_REF, C_PD, C_ALT, C_OK, C_BAD = "#1f5fa8", "#d1495b", "#8a8a8a", "#2a9d8f", "#d1495b"


def _save(fig, name):
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"{name}.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_lmi_certificate(d: dict, cfg: dict) -> None:
    """Certified ellipse of the x position loop, with a linear-model trajectory and its input."""
    from scipy.linalg import expm
    K = np.array([[-d["kv"] * d["k"], -d["kv"]]])
    A_cl = np.array([[0.0, 1.0], [0.0, 0.0]]) + np.array([[0.0], [1.0]]) @ K
    W, ubar = d["W"], d["ubar"]
    p0 = np.asarray(cfg["initial_state"]["pos"])[0] - cfg["trajectory"]["radius"]     # e_p,x(0)
    env = cfg["design"]
    a, b = max(abs(p0), env["min_position_envelope"]), env["min_velocity_envelope"]
    th = np.linspace(0, 2 * np.pi, 300)
    ell = np.linalg.cholesky(W) @ np.vstack([np.cos(th), np.sin(th)])            # boundary of xᵀW⁻¹x = 1
    t = np.linspace(0, 10, 400)
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.9))
    ax[0].plot(ell[0], ell[1], color=C_REF, lw=1.4, label=r"certified set $\mathcal{E}$")
    ax[0].plot([-a, a, a, -a, -a], [-b, -b, b, b, -b], color=C_ALT, ls="--", lw=1, label="design box")
    for x0, c in [(np.array([p0, 0.0]), C_PD), (np.array([-a, -b]), "#e9a03b")]:
        x = np.array([expm(A_cl * ti) @ x0 for ti in t])
        ax[0].plot(x[:, 0], x[:, 1], color=c, lw=1.2)
        ax[1].plot(t, (K @ x.T).ravel(), color=c, lw=1.2,
                   label=fr"start $({x0[0]:.1f},\,{x0[1]:.1f})$")
    ax[0].set_xlabel(r"$e_{p,x}$ [m]"); ax[0].set_ylabel(r"$e_{v,x}$ [m/s]"); ax[0].legend(fontsize=7)
    ax[0].set_title("Position loop, x axis")
    ax[1].axhline(ubar, color=C_ALT, ls=":"); ax[1].axhline(-ubar, color=C_ALT, ls=":")
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel(r"$u = Kx$ [m/s$^2$]"); ax[1].legend(fontsize=7)
    ax[1].set_title(r"Input stays within $\pm\bar u$")
    _save(fig, "lmi_certificate")


def _settling_time(df, tol=0.05):
    """First time after which ‖e_p‖ stays below tol."""
    above = df["e_norm"].to_numpy() >= tol
    if not above.any():
        return 0.0
    last = np.nonzero(above)[0][-1]
    return float(df["t"].iloc[min(last + 1, len(df) - 1)])


def _mission_metrics(df, f_max):
    f = df[["f1", "f2", "f3", "f4"]].to_numpy()
    ss = df["t"] >= df["t"].iloc[-1] - 10.0
    return {
        "settling_time_5cm_s": _settling_time(df),
        "final_error_m": float(df["e_norm"].iloc[-1]),
        "steady_rms_error_m": float(np.sqrt((df.loc[ss, "e_norm"] ** 2).mean())),
        "motor_force_min_N": float(f.min()),
        "motor_force_max_N": float(f.max()),
        "within_motor_limits": bool(f.min() >= 0 and f.max() <= f_max),
        "max_V2_increase": float(np.diff(df["V2"].to_numpy()[2:]).max()),
    }


def main(quick: bool) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                         "axes.spines.top": False, "axes.spines.right": False})
    summary: dict = {}
    t0 = time.time()

    # ── 1. symbolic verification ───────────────────────────────────────────
    checks = {**run_all_checks(),
              **{f"unicycle.{k}": v for k, v in derive_circular_path_following()["checks"].items()},
              **{f"path_following.{k}": v for k, v in derive_path_following()["checks"].items()}}
    print(f"[1/5] symbolic checks: {sum(checks.values())}/{len(checks)} passed")
    if not all(checks.values()):
        raise SystemExit(f"FAILED: {[k for k, v in checks.items() if not v]}")
    summary["symbolic_checks"] = {"passed": sum(checks.values()), "total": len(checks)}

    # ── 2. LMI gain synthesis ──────────────────────────────────────────────
    cfg = load("quadcopter")
    model = load_model(cfg)
    f_max = float(model.actuator_ctrlrange[:, 1].min())
    paper_gains = dict(cfg["controller"], kR=8.0, kw=2.5)        # Lee et al. 2010, 4.3 kg vehicle
    t = time.time()
    design = design_cascade_gains(cfg, model)
    gains = design["controller"]
    att, pos = design["attitude"], design["position"]
    level = float(np.sort(att["G"])[:2].sum())                    # min_{i≠j}(g_i + g_j)
    summary["lmi"] = {
        "solve_time_s": time.time() - t,
        "gains": design["controller"],
        "alpha_position": {a: pos[a]["alpha"] for a in "xyz"},
        "alpha_attitude": att["alpha"].tolist(),
        "torque_budget_Nm": design["budgets"]["tau_max"].tolist(),
        "accel_budget_xy": design["budgets"]["ubar_xy"],
        "accel_budget_z": design["budgets"]["ubar_z"],
        "attitude_level_bound": level,
        # largest roll/pitch error (zero rate error) inside {V₂ < level}: Ψ = ½(g_y + g_z)(1 − cos θ)
        "certified_roll_deg": float(np.degrees(np.arccos(max(-1.0, 1 - level / (0.5 * (att["G"][1] + att["G"][2])))))),
    }
    _plot_lmi_certificate(pos["x"], cfg)
    print(f"[2/5] LMI design in {summary['lmi']['solve_time_s']:.1f} s: "
          f"alpha_pos={[round(pos[a]['alpha'], 2) for a in 'xyz']}, alpha_att={np.round(att['alpha'], 2).tolist()}")

    # ── 3. nominal mission ─────────────────────────────────────────────────
    runs = {
        "lmi_full": simulate_quad(cfg, gains),
        "lmi_pd": simulate_quad(cfg, gains, feedforward=False),
        "paper_full": simulate_quad(cfg, paper_gains, t_end=20.0),
    }
    summary["mission"] = {k: _mission_metrics(v, f_max) for k, v in runs.items()}
    m = summary["mission"]
    print(f"[3/5] mission: settle {m['lmi_full']['settling_time_5cm_s']:.1f} s, "
          f"forces [{m['lmi_full']['motor_force_min_N']:.2f}, {m['lmi_full']['motor_force_max_N']:.2f}] N "
          f"(paper gains: [{m['paper_full']['motor_force_min_N']:.0f}, {m['paper_full']['motor_force_max_N']:.0f}] N)")

    df, pdd = runs["lmi_full"], runs["lmi_pd"]
    tr = cfg["trajectory"]
    th = np.linspace(0, 2 * np.pi, 400)

    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.0))
    ax[0].plot(tr["radius"] * np.cos(th), tr["radius"] * np.sin(th), "--", color=C_ALT, lw=1, label="reference")
    ax[0].plot(df["x"], df["y"], color=C_REF, lw=1.4, label="vehicle")
    ax[0].plot(df["x"].iloc[0], df["y"].iloc[0], "o", color="k", ms=4, label="start")
    ax[0].set_aspect("equal"); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
    ax[0].set_title("Top view"); ax[0].legend(fontsize=7, loc="lower left")
    ax[1].semilogy(df["t"], df["e_norm"], color=C_REF, lw=1.4, label="full geometric law")
    ax[1].semilogy(pdd["t"], pdd["e_norm"], color=C_PD, lw=1.2, label="PD only (no feedforward)")
    ax[1].axhline(0.05, color=C_ALT, ls=":", lw=1)
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel(r"$\|e_p\|$ [m]"); ax[1].set_ylim(1e-10, 10)
    ax[1].set_title("Position error"); ax[1].legend(fontsize=7)
    _save(fig, "mission_tracking")

    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8), sharey=False)
    for a, key, title in [(ax[0], "paper_full", "Gains from the literature"), (ax[1], "lmi_full", "LMI-synthesised gains")]:
        r = runs[key]; r = r[r["t"] <= 2.0]
        a.axhspan(0, f_max, color=C_OK, alpha=0.10, lw=0, label="motor range")
        for c, lab in zip(["f1", "f2", "f3", "f4"], ["FR", "FL", "BR", "BL"]):
            a.plot(r["t"], r[c], lw=1, label=lab)
        a.set_title(title); a.set_xlabel("t [s]")
    ax[0].set_ylabel("motor force [N]"); ax[1].legend(fontsize=7, ncol=3, loc="upper right")
    ax[1].set_ylim(-0.5, f_max + 0.5)
    _save(fig, "motor_forces")

    fig, ax = plt.subplots(figsize=(3.6, 2.6))
    ax.semilogy(df["t"], df["V2"], color=C_REF, lw=1.4)
    ax.set_xlabel("t [s]"); ax.set_ylabel(r"$V_2$"); ax.set_title("Attitude Lyapunov function")
    ax.set_xlim(0, 10); ax.set_ylim(1e-12, None)
    _save(fig, "lyapunov_attitude")

    # ── 4. Monte Carlo ─────────────────────────────────────────────────────
    n = 100 if quick else cfg["monte_carlo"]["n"]
    t = time.time()
    mc = roa_monte_carlo(cfg, gains, n=n)
    mc.to_csv(OUT / "monte_carlo.csv", index=False)
    inside = mc["V2_0"] < level
    summary["monte_carlo"] = {
        "n": n, "time_s": time.time() - t,
        "converged": float(mc["converged"].mean()),
        "feasible": float(mc["feasible"].mean()),
        "hit_ground": float(mc["hit_ground"].mean()),
        "n_inside_certified_set": int(inside.sum()),
        "converged_inside_certified_set": float(mc.loc[inside, "converged"].mean()) if inside.any() else None,
        "failures_inside_all_ground": bool(mc.loc[inside & ~mc["converged"], "hit_ground"].all()),
        "feasible_small_perturbation": float(mc.loc[(mc["angle_deg"] < 20) & (mc["offset"] < 1.0), "feasible"].mean()),
    }
    s = summary["monte_carlo"]
    print(f"[4/5] Monte Carlo n={n}: converged {s['converged']:.1%}, within motor limits {s['feasible']:.1%}, "
          f"ground contact {s['hit_ground']:.1%}")

    fig, ax = plt.subplots(figsize=(4.6, 3.0))
    groups = [(mc["converged"] & mc["feasible"], C_OK, "converged, within motor limits"),
              (mc["converged"] & ~mc["feasible"], "#e9c46a", "converged, needs > motor limits"),
              (~mc["converged"], C_BAD, "ground contact")]
    for mask, c, lab in groups:
        ax.scatter(mc.loc[mask, "angle_deg"], mc.loc[mask, "offset"], s=9, color=c, label=lab, lw=0)
    ax.set_xlabel("initial attitude error [deg]"); ax.set_ylabel("initial position offset [m]")
    ax.set_title(f"Monte Carlo, {n} initial conditions")
    ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3, frameon=False)
    _save(fig, "monte_carlo")

    # ── 5. unicycle ────────────────────────────────────────────────────────
    ucfg = load("unicycle")
    u = simulate_unicycle(ucfg)
    Rc = ucfg["control"]["R"]
    summary["unicycle"] = {"final_cross_track_m": float(abs(u["e"].iloc[-1])),
                           "V_monotone": bool(np.diff(u["V"]).max() <= 1e-9)}
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8))
    ax[0].plot(Rc * np.cos(th), Rc * np.sin(th), "--", color=C_ALT, lw=1)
    ax[0].plot(u["x"], u["y"], color=C_REF, lw=1.4); ax[0].set_aspect("equal")
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].set_title("Unicycle path")
    ax[1].semilogy(u["t"], u["V"], color=C_REF, lw=1.4)
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("V"); ax[1].set_title("Lyapunov function")
    _save(fig, "unicycle")
    print(f"[5/5] unicycle: final cross-track error {summary['unicycle']['final_cross_track_m']:.1e} m")

    # ── outputs ────────────────────────────────────────────────────────────
    summary["runtime_s"] = time.time() - t0
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    _write_macros(summary)
    print(f"done in {summary['runtime_s']:.0f} s → {OUT.relative_to(ROOT)}/")


def _sci(x: float) -> str:
    mant, exp = f"{x:.0e}".split("e")
    return rf"{mant}\times10^{{{int(exp)}}}"


def _write_macros(s: dict) -> None:
    """LaTeX macros so that the report never contains a hand-copied number."""
    m, mc, lmi = s["mission"], s["monte_carlo"], s["lmi"]
    g = lmi["gains"]
    vals = {
        "NChecks": f"{s['symbolic_checks']['total']}",
        "LmiTime": f"{lmi['solve_time_s']:.1f}",
        "AlphaPosMin": f"{min(lmi['alpha_position'].values()):.2f}",
        "AlphaPosMax": f"{max(lmi['alpha_position'].values()):.2f}",
        "AlphaRollPitch": f"{lmi['alpha_attitude'][0]:.1f}",
        "AlphaYaw": f"{lmi['alpha_attitude'][2]:.1f}",
        "TauBudgetXY": f"{lmi['torque_budget_Nm'][0]:.3f}",
        "TauBudgetZ": f"{lmi['torque_budget_Nm'][2]:.4f}",
        "AccelBudgetXY": f"{lmi['accel_budget_xy']:.1f}",
        "AccelBudgetZ": f"{lmi['accel_budget_z']:.2f}",
        "LevelBound": f"{lmi['attitude_level_bound']:.3f}",
        "GainKvX": f"{g['kv'][0]:.2f}", "GainKvZ": f"{g['kv'][2]:.2f}",
        "SettleTime": f"{m['lmi_full']['settling_time_5cm_s']:.1f}",
        "FinalErr": _sci(m['lmi_full']['final_error_m']),
        "CertRollDeg": f"{lmi['certified_roll_deg']:.0f}",
        "PdRms": f"{1e3 * m['lmi_pd']['steady_rms_error_m']:.1f}",
        "FMin": f"{m['lmi_full']['motor_force_min_N']:.2f}",
        "FMax": f"{m['lmi_full']['motor_force_max_N']:.2f}",
        "PaperFMin": f"{m['paper_full']['motor_force_min_N']:.0f}".replace("-", "$-$"),
        "PaperFMax": f"{m['paper_full']['motor_force_max_N']:.0f}",
        "PaperSettle": f"{m['paper_full']['settling_time_5cm_s']:.1f}",
        "McN": f"{mc['n']}",
        "McTime": f"{mc['time_s']:.0f}",
        "McConverged": f"{100 * mc['converged']:.1f}",
        "McFeasible": f"{100 * mc['feasible']:.1f}",
        "McGround": f"{100 * mc['hit_ground']:.1f}",
        "McInside": f"{mc['n_inside_certified_set']}",
        "McInsideConv": f"{100 * (mc['converged_inside_certified_set'] or 0):.0f}",
        "McSmallFeasible": f"{100 * mc['feasible_small_perturbation']:.0f}",
        "Runtime": f"{s['runtime_s']:.0f}",
    }
    lines = ["% generated by main.py — do not edit"] + [rf"\newcommand{{\R{k}}}{{{v}}}" for k, v in vals.items()]
    (OUT / "numbers.tex").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="smaller Monte Carlo")
    main(ap.parse_args().quick)
```

### `models/quadcopter/quadcopter.xml`

```xml
<!--
  models/quadcopter/quadcopter.xml
  
  Quadrotor model for full-dynamics cascade control.
  Identical geometry to the kinematic model, with:
    - Explicit inertia tensor on the body
    - Full sensor suite (pos, quat, linvel, angvel, accel, gyro)
    - Motor actuators mapping ctrl[i] (Newtons) → thrust + reaction torque
  
  Motor order:  ctrl[0]=FR, ctrl[1]=FL, ctrl[2]=BR, ctrl[3]=BL
  
  Mass budget:
    body box    0.50 kg
    4 arms      0.20 kg  (4 × 0.05)
    4 props     0.10 kg  (4 × 0.025)
    ─────────────────────
    total       0.80 kg
  
  Hover thrust per motor = 0.80 × 9.81 / 4 ≈ 1.96 N
-->

<mujoco model="quadcopter_dynamic">

  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>

  <!-- rendering only (src/experiments.py: render_quad); no effect on the dynamics -->
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.4 0.4 0.4"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.75 0.85 0.95" rgb2="1 1 1" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.72 0.74 0.76" rgb2="0.82 0.84 0.86" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="30 30"/>
  </asset>

  <default>
    <geom rgba="0.8 0.8 0.8 1" contype="1" conaffinity="1"/>
    <joint armature="0.005" damping="0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/>
    <light pos="3 3 3" dir="-1 -1 -1" directional="true" diffuse="0.3 0.3 0.3"/>
    <geom name="floor" type="plane" size="30 30 0.1" material="grid"/>

    <body name="quad" pos="0 0 0.5">
      <freejoint name="root"/>

      <!--
        Central frame.
        Inertia tensor for a solid box (lx=0.12, ly=0.12, lz=0.05, m=0.5):
          Ixx = m(ly²+lz²)/12 = 0.5*(0.0144+0.0025)/12 ≈ 0.000704
          Iyy = m(lx²+lz²)/12 ≈ 0.000704
          Izz = m(lx²+ly²)/12 = 0.5*0.0288/12 ≈ 0.00120
        Arms add extra inertia; values below are slightly inflated to be realistic.
      -->
      <inertial pos="0 0 0" mass="0.80"
                diaginertia="0.0035 0.0035 0.0060"/>

      <geom name="body" type="box"
            size="0.06 0.06 0.025"
            mass="0.50"
            rgba="0.15 0.15 0.70 1"/>

      <!-- ── arms (X-configuration, 45°) ── -->
      <geom name="arm_fr" type="capsule" fromto="0 0 0  0.13 -0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_fl" type="capsule" fromto="0 0 0  0.13  0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_br" type="capsule" fromto="0 0 0 -0.13 -0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_bl" type="capsule" fromto="0 0 0 -0.13  0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>

      <!-- ── rotor disks (visual only) ── -->
      <geom name="prop_fr" type="cylinder" pos=" 0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 0.8"/>
      <geom name="prop_fl" type="cylinder" pos=" 0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 0.8"/>
      <geom name="prop_br" type="cylinder" pos="-0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.9 0.1 0.1 0.8"/>
      <geom name="prop_bl" type="cylinder" pos="-0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.9 0.1 0.1 0.8"/>

      <!--
        Thrust sites.
        Gear = [fx fy fz tx ty tz] in site local frame.
          Force component  (0 0 1):    ctrl[i] N upward thrust
          Torque component (0 0 ±km): reaction yaw torque
            km = 0.012  (torque/thrust ratio, typical for small quads)
          CW  motors (FR, BL): negative z-torque → gear "0 0 1 0 0 -0.012"
          CCW motors (FL, BR): positive z-torque → gear "0 0 1 0 0  0.012"
      -->
      <site name="s_fr" pos=" 0.13 -0.13 0" size="0.008" rgba="0 1 0 1"/>  <!-- FR  CW  -->
      <site name="s_fl" pos=" 0.13  0.13 0" size="0.008" rgba="0 1 0 1"/>  <!-- FL  CCW -->
      <site name="s_br" pos="-0.13 -0.13 0" size="0.008" rgba="1 0 0 1"/>  <!-- BR  CCW -->
      <site name="s_bl" pos="-0.13  0.13 0" size="0.008" rgba="1 0 0 1"/>  <!-- BL  CW  -->

      <!-- IMU at centre of mass -->
      <site name="imu" pos="0 0 0" size="0.005" rgba="1 1 0 1"/>

    </body>
  </worldbody>

  <!--
    Actuators.
    ctrl[i] is the thrust of motor i in Newtons.
    MuJoCo applies the force at the site AND the torque from the gear vector.
    The cascade controller's allocation matrix maps [T, τx, τy, τz] → ctrl[0..3].

    ctrlrange is the motor spec (0–8 N per motor); it is read by the gain
    design and by the feasibility checks.  ctrllimited="false" keeps MuJoCo
    from clipping: the stability analysis assumes unbounded, signed motor
    forces, so the simulated plant must not clip them either.
  -->
  <actuator>
    <motor name="m_fr" site="s_fr" gear="0 0 1 0 0 -0.012" ctrlrange="0 8" ctrllimited="false"/>
    <motor name="m_fl" site="s_fl" gear="0 0 1 0 0  0.012" ctrlrange="0 8" ctrllimited="false"/>
    <motor name="m_br" site="s_br" gear="0 0 1 0 0  0.012" ctrlrange="0 8" ctrllimited="false"/>
    <motor name="m_bl" site="s_bl" gear="0 0 1 0 0 -0.012" ctrlrange="0 8" ctrllimited="false"/>
  </actuator>

  <sensor>
    <!-- position and orientation -->
    <framepos    name="pos"    objtype="site" objname="imu"/>
    <framequat   name="quat"   objtype="site" objname="imu"/>
    <!-- velocities -->
    <framelinvel name="linvel" objtype="site" objname="imu"/>
    <frameangvel name="angvel" objtype="site" objname="imu"/>
    <!-- raw IMU (for realism / future use) -->
    <gyro        name="gyro"   site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>

</mujoco>
```

### `MUJOCO_LOG.TXT`

```text
Mon Oct 12 16:30:29 2026
WARNING: Nan, Inf or huge value in CTRL at ACTUATOR 0. The simulation is unstable. Time = 0.8080.

Mon Oct 12 16:30:29 2026
WARNING: Nan, Inf or huge value in QACC at DOF 3. The simulation is unstable. Time = 0.8080.

Mon Oct 12 16:30:29 2026
WARNING: Nan, Inf or huge value in CTRL at ACTUATOR 0. The simulation is unstable. Time = 0.5960.

Mon Oct 12 16:30:29 2026
WARNING: Nan, Inf or huge value in QACC at DOF 3. The simulation is unstable. Time = 0.5960.

Mon Oct 12 16:30:29 2026
WARNING: Nan, Inf or huge value in QACC at DOF 3. The simulation is unstable. Time = 1.1900.
```

### `pyproject.toml`

```toml
[project]
name = "mcav-project"
version = "0.1.0"
description = "Geometric cascade control of a quadrotor in MuJoCo, with symbolically verified stability analysis"
requires-python = ">=3.11,<3.14"
dependencies = [
    "cvxpy>=1.9.3",
    "imageio[ffmpeg]>=2.38.1",
    "ipykernel>=7.2.0",
    "joblib>=1.5.3",
    "jupyterlab>=4.5.7",
    "matplotlib>=3.10.9",
    "mujoco>=3.9.0",
    "numpy>=2.4.6",
    "pandas>=3.0.3",
    "pyyaml>=6.0.3",
    "scipy>=1.17.1",
    "sympy>=1.14.0",
]
[dependency-groups]
dev = [
    "black>=26.5.1",
    "pybind11-stubgen>=3.0.0",
    "pytest>=9.0.3",
    "ruff>=0.15.14",
]
[tool.pyright]
stubPath = "typings"
```

### `results/monte_carlo.csv`

```csv
offset,angle_deg,V2_0,converged,hit_ground,feasible
1.765391856987295,170.86721639145446,0.8168327899206725,False,True,False
1.5345364678253204,115.55683541803245,0.3995005666703369,True,False,True
1.1460711998005746,167.3330914220253,0.8594229973005151,False,True,False
1.9770218719806678,120.43926796446316,0.579557836900725,True,False,True
1.5511274480671222,121.93761903114938,0.4036947091391576,True,False,True
1.790354556997683,122.68582444748041,0.7033432340394535,True,False,True
1.623433596279821,21.99279174212765,0.0948514774964159,True,False,True
0.8642556668924182,44.209695695793364,0.06061518970441651,True,False,True
1.237157908510749,11.513828324560862,0.017030606628606077,True,False,True
0.7959182009505364,163.18922018733156,0.8722840140049574,False,True,False
1.1513458847061553,174.040456128853,0.8885212335829368,False,True,True
1.1699710568790522,105.88591230075836,0.12575900083129954,True,False,True
1.4916399542697378,27.29864034459153,0.05911234619909985,True,False,True
1.370395451679846,21.76295570697524,0.014473322179973103,True,False,True
1.4261128670296452,78.07453099556591,0.27372473131946656,True,False,True
1.8696085572055128,68.39180477854033,0.2769430998556031,True,False,True
0.5549195233202836,34.23998653643697,0.019350555548784378,True,False,True
0.9387079288998075,15.613714176210102,0.006620350975997785,True,False,True
1.8626524090708845,113.29301917012901,0.5403206303622178,True,False,True
1.6225606966929835,138.65226537991316,0.708383345661744,True,False,True
1.5199000844564061,11.039999100747494,0.0613990439418423,True,False,True
1.28836525385228,54.16494791157861,0.028983563306676143,True,False,True
1.6309798151452894,52.57474971119424,0.0444981022463839,True,False,True
1.2102154739245876,94.881985816735,0.3400255400833528,True,False,True
1.9007278557016187,105.79174476176046,0.5121358542759749,True,False,True
1.8750493676730036,71.8346347207372,0.20477820863396876,True,False,True
1.3706131597323084,151.54010224984293,0.4013953910474357,True,False,True
0.580876512638975,68.3882617052402,0.2774803101683457,True,False,True
1.9221184335606316,53.08851597453224,0.14669358293596604,True,False,True
1.3368871017304078,134.17677641512208,0.7102832641292625,True,False,True
1.7385654367280414,147.33803771752267,0.7841045303157251,True,False,True
1.9005763903071835,153.5447490547817,0.8917883295397797,True,False,True
1.8553674422035686,29.624434149881925,0.04905208528967047,True,False,True
1.903779018940441,34.46758821494651,0.029371856409637083,True,False,True
1.5758180856893182,80.95559168749709,0.4152663397568628,True,False,True
1.888793025474673,169.32434534250646,0.9192428172512127,False,True,False
1.5451807970994307,108.56279079352217,0.7067614947462083,True,False,True
1.2827183819024486,109.08803780997164,0.11018737817105083,True,False,True
1.5669974122559922,57.16793151774143,0.2814683668432417,True,False,True
1.389574354335003,173.3358556702516,0.5885208717178803,True,False,True
1.830864309769248,31.956073569515233,0.041802215327560725,True,False,True
1.8808483040375537,97.87532515118305,0.5228248535010538,True,False,True
1.041377144201308,118.10719156111159,0.610961040916697,True,False,True
1.0956895802331732,141.74774285913372,0.278908169602271,True,False,True
1.4532689596862436,142.3570020150361,0.8138159019924216,True,False,True
1.5475564657153502,67.64087183947325,0.07717073132992783,True,False,True
1.9704464459379951,47.42493009646627,0.04494478914127076,True,False,True
1.9603470852684912,171.45856573441662,0.23937227008518516,True,False,True
1.9777811254650048,96.7634791001409,0.42537285687410115,True,False,True
1.8006215404148604,49.364046033092315,0.14444094963675075,True,False,True
1.7563380787172145,24.200245075524073,0.011403603621598651,True,False,True
1.2932847438518282,140.5578117279611,0.5462345053409942,True,False,True
1.4392364951711778,148.49089995138186,0.5488194397856125,True,False,True
0.2779201880539708,99.71436462419716,0.5297476228151715,True,False,True
1.881787369286967,67.19855672915024,0.19268130791839352,True,False,True
1.3022997921779673,115.45056051999423,0.5511556633914897,True,False,True
0.85918468725197,171.51865944323742,0.21909880575349214,True,False,True
1.3616691494581115,101.20620063835831,0.3727661213304588,True,False,True
1.3724830902415874,137.74293319974728,0.6904410787521844,True,False,True
1.7958634524430148,154.359971518167,0.41529950071746696,True,False,True
0.3574025793568845,132.44937707184326,0.6789635011453832,True,False,True
1.22540911617591,85.14683205931648,0.3723106578949636,True,False,True
1.9284503640538235,31.20532537212933,0.014331758733073704,True,False,True
1.8981523086025183,121.89560472240396,0.20103772510988166,True,False,True
1.7136445704225278,145.51179472817165,0.7100985574565386,True,False,True
1.647483049729321,168.50038228660253,0.6352214884657952,True,False,True
1.2121316264861044,26.64869922556441,0.06027305934096549,True,False,True
1.9130022167278753,19.89294219360802,0.005949616486571434,True,False,True
1.8257384254577542,111.34158394406646,0.3262373812533165,True,False,True
1.6009709295516799,144.74346598720626,0.23938267107120592,True,False,True
1.0880655843362401,13.366626597887565,0.012642223673174493,True,False,True
1.1997021596922424,107.45558228338663,0.4851234889881892,True,False,True
1.4477329922310187,46.616441360774964,0.10296175466945703,True,False,True
1.3836528924620435,174.86171159121972,0.5196835348049783,True,False,True
0.7875405077742279,63.24125633795162,0.27568415673236274,True,False,True
1.4814016007862778,14.34263990956763,0.04519078541712101,True,False,True
1.531161236011492,136.96995668556642,0.7008690431289045,True,False,True
1.3193627817507123,52.625068962371785,0.21919948087248378,True,False,True
1.4540785939711538,73.34421546715733,0.11275048228323402,True,False,True
1.3853960867872994,22.845101561183277,0.005736782663619003,True,False,True
1.4445990366188504,57.90352188887469,0.034348524991592785,True,False,True
1.704514848228725,23.327924594859546,0.051242193129566335,True,False,True
1.5845711718287405,45.037023109113605,0.04022288185193388,True,False,True
1.1648890705420174,101.27621041128056,0.6479385167203376,True,False,True
1.852961145264506,52.31873629178805,0.16274682902602544,True,False,True
1.6012349853554508,116.80781248784366,0.7046949168799294,True,False,True
1.5420569136865754,157.69777233050587,0.29820165301170487,True,False,True
1.7870199802116826,100.87203546670683,0.5604008841592003,True,False,True
1.818845585971,5.018923235539765,0.0017730516777860065,True,False,True
1.8938374669699691,167.96533459111566,0.6230871783588101,True,False,True
1.8652954594581903,34.61920494258245,0.17404330524940465,True,False,True
1.749210183977581,60.576749023518246,0.32926443151885415,True,False,True
1.793498639877283,92.13965811996238,0.22945323444516516,True,False,True
1.0792286896669216,108.29208771615944,0.08511691171293054,True,False,True
1.7751762668421138,59.864139298141694,0.1270874553545228,True,False,True
1.267365636673077,77.08180935051924,0.3063626154484199,True,False,True
1.27294910902201,109.69380410287982,0.38471839093861643,True,False,True
0.9868457544155697,79.99708032451474,0.22915310411400316,True,False,True
1.4698070319619403,49.589005033082785,0.08983365551056749,True,False,True
0.5417745054476425,71.87977011819542,0.15556978832041915,True,False,True
1.7386571182340773,67.3331830777007,0.15428486178819706,True,False,True
1.8155870007905766,51.09208199849691,0.2786156003452564,True,False,True
1.8375246702417256,105.12127640858718,0.4053915784013207,True,False,True
1.0101943573267291,89.64546623184536,0.32045883651553875,True,False,True
1.7245226394435382,72.01747166840056,0.1867888254824181,True,False,True
1.8857986325280092,124.14988780265948,0.08194167358134528,True,False,True
0.6601997546417689,142.19721667179925,0.8156786363863752,True,False,True
1.1565348951097134,113.54809166680538,0.6306902379926049,True,False,True
1.2891014783690329,59.70748856975789,0.029556131350804832,True,False,True
1.546105607091086,163.91395824724123,0.5182968374657109,True,False,True
1.1126155041859045,18.237456486365364,0.045401551286379724,True,False,True
1.6275156485510358,140.96951511909722,0.30053636300678116,True,False,True
1.4012936924186337,73.72341193393976,0.3473336107971349,True,False,True
1.737006753488071,69.42806807341398,0.336842865588731,True,False,True
1.6198816716209121,147.55816353115867,0.5107099306989737,True,False,True
1.7641448048779373,114.6770273975547,0.16755038554430374,True,False,True
1.2537441647475944,170.42788101622818,0.9037374388053052,False,True,True
1.7817655598221773,153.65383758047037,0.836067089321881,False,True,False
1.804627145180476,20.347144338749793,0.012257722655982102,True,False,True
1.7941932325257828,88.0520545338327,0.24402383418384327,True,False,True
0.8181281083442264,134.3315559672099,0.2393135699481806,True,False,True
1.9634358275218589,14.447602018078392,0.08133470529128942,True,False,True
1.6648985325861343,94.93375331298296,0.3497348240092512,True,False,True
1.2963512315001984,173.73954124791607,0.737817583737757,True,False,True
1.5143936259380646,41.82568821149064,0.06818721289309584,True,False,True
1.6058138008046405,20.438702427896608,0.004041441031847172,True,False,True
1.6059876360859455,50.648782867199316,0.2160027862594741,True,False,True
1.2564821187719186,60.25907679883224,0.1548862520500434,True,False,True
1.28457394985999,50.01056236370047,0.04951505925676288,True,False,True
1.427468329039688,75.6154667331336,0.31330792771892796,True,False,True
1.8417601463654982,145.84657687248327,0.2909118620883327,True,False,True
1.5182297604846355,90.54198314349613,0.49440640027861277,True,False,True
0.6760982426048588,103.1199230522725,0.42846204986127484,True,False,True
1.5351235588044814,19.0812966845019,0.008332883716796325,True,False,True
1.172257267911937,11.340840970507372,0.0019058395179540719,True,False,True
0.46307205291461123,166.42442712623114,0.7743096717598573,True,False,True
1.9845679225994486,60.74172152888644,0.16966829220246615,True,False,True
1.3294583567699347,165.78302638278785,0.8959999216014718,False,True,True
1.6261298232304413,58.73484735372673,0.26312433334092983,True,False,True
1.4203817906451406,111.92583193812837,0.5461903085662502,True,False,True
0.7531740155880101,115.76736661193353,0.4975991532118602,True,False,True
1.9316107432064578,15.748423654643508,0.09928818364393449,True,False,True
1.9561505881868675,12.414113169258064,0.046103759531239255,True,False,True
1.1425381364720169,77.30162376733688,0.33692574291068317,True,False,True
1.4953896448840518,22.274535524507545,0.005572134564181841,True,False,True
1.4405048987862945,51.770155831623086,0.02028385541525976,True,False,True
1.1018525090932385,34.27468241241739,0.021233352184010658,True,False,True
1.7427767658997395,54.764686378189374,0.09361573710072496,True,False,True
1.842601912166949,79.78891790331744,0.4032300669144294,True,False,True
1.8249192824265352,54.48307172768847,0.10667767498372219,True,False,True
1.7546289041336602,58.70213704802168,0.06791017143865646,True,False,True
0.5191545344890393,85.04463406593764,0.17183994379418285,True,False,True
1.2246697552435135,119.47754067754366,0.09795626596927626,True,False,True
1.6027049376341387,7.350062623676772,0.017954067235766768,True,False,True
1.7782068334829564,174.56410258749133,0.8260931460712604,True,False,True
1.4567484114072606,75.39097853236203,0.4293742725256137,True,False,True
1.5317860409534287,161.5301019191189,0.7141494888814897,True,False,True
1.8876159618331938,1.267492902501935,0.003294517597845029,True,False,True
0.5732468913238451,10.06014288245387,0.0050908007237526086,True,False,True
1.2904049893680698,17.349800661402057,0.008189851369110726,True,False,True
1.3237225169314044,159.72947455121164,0.8346034702483084,False,True,True
1.9951767920889376,72.10389198146207,0.344860186647484,True,False,True
1.2209937436893443,148.11747540905387,0.10214975015894286,True,False,True
0.7223440211488136,121.55997617809392,0.5884938954295079,True,False,True
1.4935595945649631,15.367109691303058,0.03077867251862751,True,False,True
1.732059528422651,63.297517957691944,0.2924668825180136,True,False,True
1.070668002010807,58.00606060891648,0.187858472713684,True,False,True
1.8868490330320078,79.28688045896494,0.33699691126717607,True,False,True
1.162657992681998,131.9175373924269,0.6857063454647769,True,False,True
1.7794193010804713,14.632833093386466,0.017767333603261534,True,False,True
1.089841475483832,162.1127687541349,0.846196769735107,False,True,False
1.9178195482896858,83.40459676037416,0.2577536923975088,True,False,True
1.1721754490294405,64.01250587787146,0.08710028722675499,True,False,True
1.8019393808021862,122.0972941211015,0.3419754428241371,True,False,True
1.8860991265708407,48.38755030094104,0.08161731708383833,True,False,True
1.2681762967245815,124.6824869512711,0.7675669125071221,True,False,True
0.9055656919035523,72.78893052467504,0.40944535516742714,True,False,True
1.6737763492135933,83.67739642651665,0.13201639807518123,True,False,True
1.586364355290393,33.093873800211206,0.13038341357710007,True,False,True
1.5652068640890349,35.557312308417615,0.05872429679623793,True,False,True
1.8420820802464895,150.74397380829328,0.8230609839251173,False,True,True
1.547359664254414,100.85647817605332,0.6056394063841555,True,False,True
1.5842532474496003,73.89876176993734,0.4234333362697028,True,False,True
1.573807147770448,135.6191586223523,0.6529638516809663,True,False,True
1.537810041249537,159.61839491219374,0.881433772937697,False,True,True
1.4806132935439609,27.800188028846783,0.03254363651302608,True,False,True
1.0878086107524145,174.64995832397156,0.6416368684350833,True,False,True
1.3318530370625687,2.474923756359729,0.004962410400231888,True,False,True
1.2844564168711803,107.44378698896607,0.16064823785032303,True,False,True
1.256856757766747,46.13784581101032,0.014635849634981768,True,False,True
1.843252547162121,44.12332988232357,0.10124770425424062,True,False,True
0.969868856229547,86.56444996603442,0.08623363688584418,True,False,True
1.1800791723784727,105.73920757494386,0.5964298988349046,True,False,True
1.6148755518496771,76.44971715180105,0.31349941062647574,True,False,True
1.3572688296665516,124.94601914195431,0.6429364901722338,True,False,True
1.2431739557391601,166.3006866817121,0.5057787908300452,True,False,True
1.4401736188818741,29.55272277836355,0.10981499362997399,True,False,True
1.5895372152771543,41.57745386299008,0.02638953159920348,True,False,True
1.753834407136183,106.67372040004472,0.5344428263094858,True,False,True
1.9813072345210683,71.79541689344906,0.24893517808393512,True,False,True
1.8964949690046593,55.19264116371494,0.12667226245912944,True,False,True
1.509559569849079,88.28709238835022,0.5682275506337466,True,False,True
1.5344751029152086,121.94624967359486,0.3994404611001605,True,False,True
1.4926614326904444,159.01706363200432,0.19423231994968315,True,False,True
1.2410495780305588,164.6521275284982,0.6732171477790985,True,False,True
1.9673122889053125,143.41844251538086,0.754449316831698,True,False,True
0.8189625434510388,60.65664673618486,0.11126195252883271,True,False,True
1.1791726357654853,154.67737620492392,0.8345647964490807,True,False,True
1.7430581099226967,124.23980262555092,0.6220822655063221,True,False,True
1.6770265064796344,30.536335425875656,0.03380843466917268,True,False,True
1.306316049114505,166.81670828389488,0.6405208902077234,True,False,True
1.957054175234495,43.28494554453428,0.10839493352165261,True,False,True
1.4851330638977203,119.37487161950052,0.10794649301410962,True,False,True
1.6833470535215567,121.5425857719174,0.6185189487076602,True,False,True
0.8996750906519825,158.9110872947471,0.4133085883947518,True,False,True
1.924440564584958,116.63059359935406,0.5092667115934781,True,False,True
0.9375205097279472,172.04866511812116,0.7010143671156892,True,False,True
1.4662653709096305,10.452756467448376,0.019705743357216525,True,False,True
1.9206061630143942,6.555396623877174,0.009962242487705233,True,False,True
1.962635639239029,34.2333583577266,0.04421366224637048,True,False,True
1.9338178865436766,100.67116845449937,0.5918340544018461,True,False,True
1.8777407163639734,74.18484293190023,0.31155823954982714,True,False,True
1.2312192381197997,14.439547289904386,0.007858085764017819,True,False,True
0.6339935342480217,35.35160334573926,0.009095894043470899,True,False,True
1.5346704463404757,101.36834600783529,0.3300759014738348,True,False,True
0.7965668953360885,70.43300407942948,0.22340285241323116,True,False,True
1.9733355664295125,133.6015858251197,0.4170410991498391,True,False,True
0.40309996788954644,81.71880204173871,0.482489155080277,True,False,True
1.375746331095924,153.60397598778488,0.3945928190005001,True,False,True
1.1759900304377502,132.78219450284863,0.4730635589128927,True,False,True
0.4784548476351933,86.52726137518286,0.12149477818761431,True,False,True
1.5864247927020008,62.64764498028787,0.16039927429877313,True,False,True
0.8473497336503074,37.42225613705047,0.1760635817190147,True,False,True
1.9309352539459166,51.11342966562716,0.10030171238379072,True,False,True
1.7652836424002645,62.13860443278,0.17215715039700882,True,False,True
1.4358118165877063,63.21635299202078,0.20023405454779036,True,False,True
1.8882481077710656,31.327686348440675,0.08797928836491607,True,False,True
1.8542014169005203,82.81199900772386,0.45835461303852254,True,False,True
1.377098282368318,19.570733593180332,0.01160947282920779,True,False,True
1.8924748839328187,152.001215252119,0.8503992346822171,False,True,False
1.95177855669227,96.3106434275109,0.5877832502676544,True,False,True
1.705304525790938,16.641415352494704,0.0022690472277782057,True,False,True
1.4118490833265565,57.36867396622094,0.17160145274317237,True,False,True
1.9313978066781423,44.666459586068086,0.028330776318349005,True,False,True
1.961161641926568,27.327130990211916,0.09688081708941632,True,False,True
1.4058043535858389,84.11186588558827,0.18903730079447187,True,False,True
0.8837987381876427,32.07258194238521,0.03042591044434422,True,False,True
1.9076772393616395,26.54281338393932,0.04489067193427823,True,False,True
1.745522922019967,122.90032007718973,0.6335002191431944,True,False,True
1.228922888539743,75.1763616959968,0.491636336250014,True,False,True
1.6552836693744888,117.60981866505172,0.3701790360228367,True,False,True
1.3509457210491171,82.24531784175132,0.1789131234678198,True,False,True
1.9446054729486149,88.19652521579918,0.514396807250814,True,False,True
1.398794494367239,54.371188658890546,0.12218848951564862,True,False,True
1.1622666978893712,74.60589129124011,0.1863276817999709,True,False,True
0.7018139287548446,84.15723008406553,0.31305359799012783,True,False,True
1.399342667950049,59.920349521968646,0.1794747258689758,True,False,True
1.9929399999132493,144.58718001070946,0.8208637989697639,False,True,True
1.2043173141250907,43.098441296336944,0.19750412522584468,True,False,True
1.5417053215370997,38.594554121727576,0.13355166475602734,True,False,True
1.9812287854822472,60.24375691113695,0.35256465371383355,True,False,True
0.37730645953943115,137.4068994871969,0.7707369089359076,True,False,True
1.3314810552824832,148.32427672497354,0.7801024558202573,True,False,True
1.8728809425320638,98.87860444141408,0.34953064854253063,True,False,True
0.9149174322939908,63.12008898313511,0.283732289956196,True,False,True
1.9510070636955477,113.54168444347334,0.6094700621118999,True,False,True
1.882993147333642,144.63251011200376,0.11152518337065508,True,False,True
1.0459902296647527,42.50527822732462,0.1146918287237579,True,False,True
1.6796616244725004,116.82065642134096,0.31959175205940005,True,False,True
1.6440848509618802,124.22784111423938,0.7221844125781501,True,False,True
1.7704590037028687,68.29343465735028,0.2955905297878844,True,False,True
1.3489654218296985,25.34233398252328,0.10273971382532265,True,False,True
1.4538915496734746,106.30239306928496,0.3979805228816653,True,False,True
1.2442742404477023,36.77961014729355,0.10343486734541892,True,False,True
1.326705236972704,64.40235541522726,0.07274066251922104,True,False,True
1.1202396221101159,95.29391750081015,0.1080344704400671,True,False,True
1.9341225783783447,168.47251917968657,0.44430390431359573,True,False,True
1.9219425847755418,25.64154461889177,0.03197412938178693,True,False,True
1.4227791829621423,12.896353861517133,0.012713655049569858,True,False,True
1.7700543357416645,94.30076319148016,0.24150861352937605,True,False,True
1.5770424945431913,165.04003217547816,0.1786058178123709,True,False,True
1.309644604331191,8.085036950318356,0.02788004642924532,True,False,True
1.4089282642478074,112.3706112050678,0.3895914723108891,True,False,True
1.9764953065974469,4.774663746844499,0.007332981032862958,True,False,True
1.3979036115160897,96.43346536672841,0.5234400031466485,True,False,True
1.666630890546961,20.85369917297351,0.037444282100666165,True,False,True
1.9627107354422975,115.34626530711631,0.264475301605388,True,False,True
1.335276860714436,63.17250918911063,0.09592260012481081,True,False,True
1.863525922578904,114.54973503615965,0.11347002993777767,True,False,True
0.9699905465748576,45.19353297109299,0.2298449269125375,True,False,True
1.803557178001932,13.110046115069267,0.006142924821074518,True,False,True
1.8617947821880036,31.829603570606807,0.0669780277514979,True,False,True
1.8851614674892094,47.93521957746641,0.11559933591597465,True,False,True
1.9832284259769464,28.834349072096707,0.039000772046276645,True,False,True
1.785010282263863,34.72596678258689,0.1054241750623984,True,False,True
1.9846904900509723,95.23159553690266,0.5077234213488329,True,False,True
1.3058565503244568,76.58732988224048,0.10174177161788264,True,False,True
1.6098978905302845,144.42758524612344,0.36796703799387526,True,False,True
1.196423028404766,87.08524810444972,0.41143347605027014,True,False,True
0.8742627087849049,94.41400119265256,0.4915581517303106,True,False,True
1.5199542397091779,124.27226152876766,0.3201776814464985,True,False,True
1.7922966026467018,149.34744984802848,0.6733795539249399,True,False,True
1.8398100499945889,102.32277014543236,0.5370341840654899,True,False,True
0.5141399256713517,6.443876818547696,0.01337920566240727,True,False,True
1.9607126904438108,80.36229276456898,0.29493771281601444,True,False,True
1.255256302150361,5.647219286253177,0.015705246587580032,True,False,True
1.3271606810905952,105.85389258986307,0.39620511229476185,True,False,True
1.8813781850710853,11.383339045566949,0.007880156897045527,True,False,True
1.9268324693829186,94.43158676603215,0.2609374681390729,True,False,True
1.8061001808855077,26.23619998645234,0.05081947970932705,True,False,True
1.3860056243734657,149.71672040198627,0.6599841652669729,True,False,True
1.54706205573036,16.502175348153276,0.011772058863400606,True,False,True
1.8830289796164812,115.3986256152551,0.7067061745789084,True,False,True
1.419202809816848,123.16625938092795,0.7015077422071081,True,False,True
1.8607312084585108,48.157828886572226,0.16992550641230086,True,False,True
1.8426124332829363,103.73540146199551,0.44942756639433007,True,False,True
1.5535294678361973,57.735729147629904,0.2381409543953871,True,False,True
1.9285118672051722,45.31644516266707,0.013962044338720558,True,False,True
1.454352050570995,45.4394689358045,0.13213007987459124,True,False,True
1.817052201933427,2.7730416096468677,0.0019209489581533338,True,False,True
1.7233222461602227,117.76379799848813,0.6226317863498035,True,False,True
1.3838284312304647,12.571044712569394,0.0023329252653866046,True,False,True
1.620348399616001,34.47684451050685,0.1272509246307015,True,False,True
1.6925066983131654,15.215968216851703,0.0668066437077372,True,False,True
1.625453272239228,140.74895661489538,0.574399535721865,True,False,True
0.9603675555992666,92.65952332244524,0.4088128080833657,True,False,True
1.994284073727486,81.78734644975042,0.20733007371098336,True,False,True
1.9774838573496565,102.08124991013334,0.47622469802875383,True,False,True
0.6692810556592896,141.44759917854984,0.10881155266481474,True,False,True
1.9621410757959357,109.21497037422202,0.4888412384587459,True,False,True
1.6200486150864564,89.8234382151017,0.23607702639644262,True,False,True
1.0018751598317315,54.80409329930096,0.19565243106151917,True,False,True
1.872413259138335,90.4307947468235,0.27072355994006464,True,False,True
1.5130302884540259,172.76659130556843,0.10554260074037558,True,False,True
1.7868929928842494,99.72818847508421,0.4720955713740534,True,False,True
1.8774609742885597,83.50893170649096,0.3115427181579153,True,False,True
1.6266773504992433,155.45099432897126,0.6047854427597864,True,False,True
0.7351585227873343,9.543675904562406,0.006452258577165201,True,False,True
1.8738586478326986,11.072936862225683,0.047559857071515145,True,False,True
1.7707522946925955,138.14129192752708,0.7621831143631621,True,False,True
1.9382046273695765,164.87576989711852,0.40344313554953537,True,False,True
1.2928795573860412,8.90575156332338,0.04165998133926222,True,False,True
1.8450924856264221,4.2094111370221485,0.028408993323787653,True,False,True
1.9115584461815782,1.7072440918274536,0.010309011398472191,True,False,True
1.4162076973637716,100.96936172702686,0.6398445895195126,True,False,True
1.4860188417739237,24.110439739746756,0.0684615733097771,True,False,True
1.3876290600717645,98.23465221932885,0.17813391477451326,True,False,True
0.3578378175769214,116.16397691097302,0.5917437716693822,True,False,True
1.2281447241321597,2.720708823069767,0.003572648399819207,True,False,True
1.3842261318361582,152.2703124159625,0.09602946392884862,True,False,True
0.9536261921710336,44.20020403640166,0.11225328326708106,True,False,True
1.8295980851851732,96.94643587733233,0.6115341407249997,True,False,True
1.5059939329574525,61.897051842555676,0.19450342166274687,True,False,True
1.656535364344376,60.07172408213765,0.02815303907748883,True,False,True
1.3155630278232677,70.58727833657471,0.16889419589768018,True,False,True
1.4232160214173741,0.6494243043735666,0.007944341476132849,True,False,True
1.8198494816517676,75.03709776212257,0.36004996653791255,True,False,True
1.5547952439835588,46.37794724708353,0.06878659886866632,True,False,True
1.6835996339159105,62.54635054767477,0.4036653579172953,True,False,True
1.5009540775222667,28.507360561169865,0.0557436764440995,True,False,True
1.6455994139028556,58.60226015744196,0.14172487456337016,True,False,True
1.929087621953889,122.40088176816644,0.30045436856054364,True,False,True
1.288500888101827,172.16146240158747,0.15933259132875988,True,False,True
1.6073226482002365,34.497564234678585,0.0907238827033246,True,False,True
1.7649397278969436,151.67013117088499,0.8905399227067138,False,True,False
0.8854941696706922,107.58240010927226,0.5347486225088495,True,False,True
1.974344016852185,17.675762537905847,0.07207734936391795,True,False,True
1.5860853185524821,2.5714800742282993,0.01704648991956359,True,False,True
1.2520388619374934,154.704161002244,0.7851494613681256,False,True,True
1.8255058345067667,40.96068167217454,0.05264747581301804,True,False,True
1.8686114767891542,18.59252066294912,0.10359274894075485,True,False,True
1.7150857822876222,114.72348516429903,0.7347435986775959,True,False,True
1.9424695040986086,123.65566187964924,0.45151729364061716,True,False,True
0.8926502441172937,156.63667698373072,0.6948950633083896,True,False,True
1.6160861560822053,87.5556707754068,0.10338103325907533,True,False,True
1.8113603195296584,54.59359354124232,0.19144746849037705,True,False,True
1.942932563755805,55.69433209242713,0.13122813979803855,True,False,True
1.724845523467505,35.83286876359168,0.17211106783384114,True,False,True
1.6890290882426182,111.16627708204926,0.08134402506205675,True,False,True
1.4590122962269232,164.89765915149093,0.3177433580523088,True,False,True
1.6598287252691855,14.819546542491963,0.0041643912013176135,True,False,True
1.6433926369564427,67.66141334901023,0.15589441503749937,True,False,True
1.9345658592741601,61.568609439325066,0.3347372474401607,True,False,True
1.8773685883746494,14.035239413816242,0.06868612608113854,True,False,True
1.8067201393652468,104.09365295497156,0.2580809105744876,True,False,True
1.9478414653069396,172.02632149305495,0.7875011949247325,True,False,True
0.808910889397991,48.418829893267805,0.07431551960988649,True,False,True
1.8328277876788284,127.24459785109484,0.2954521227975622,True,False,True
1.5479241328075766,157.6013346199766,0.2295919777325995,True,False,True
1.8185769962531606,47.36374369434225,0.26942212150466305,True,False,True
1.3331551222058176,160.82237632936776,0.7836004130747364,True,False,True
1.5520550236100834,130.96513189484116,0.3776832186478236,True,False,True
1.8754672167585082,112.98780907012338,0.3028112929315754,True,False,True
0.6553341484676961,33.974040907225145,0.04802679526589718,True,False,True
1.3255331651102347,48.54518468408434,0.22807692670391613,True,False,True
1.5750953981168272,17.771885870172536,0.02988921160539184,True,False,True
1.8956518580364092,156.7514681640877,0.8622271087652409,True,False,True
1.2927137863982092,141.56681597485894,0.7644354590292787,True,False,True
1.8743169290178046,15.153201552963575,0.022614929418704497,True,False,True
1.2506888034611403,92.92759268125455,0.534540089172831,True,False,True
1.343537023036949,37.29237528025499,0.1610972362806886,True,False,True
1.6424256107664217,133.98412513172264,0.582450616932489,True,False,True
1.735940864513353,100.29946489506263,0.0725926327369362,True,False,True
1.9166187550718836,165.44224081794164,0.8515916049732282,False,True,False
1.916180972021893,127.24922665285047,0.6274313621735802,True,False,True
1.260192714559803,17.620232190171397,0.047942263540793895,True,False,True
1.813561897805461,65.2366501362329,0.21610741569452072,True,False,True
1.7190413093593622,100.84793729730171,0.14402671706315207,True,False,True
1.4563963580336043,159.57171612912785,0.2736366562398687,True,False,True
1.8379134077237642,64.91581620965299,0.24516837607329964,True,False,True
1.9292030480395466,32.8812279978406,0.053490723527981815,True,False,True
1.5650706943057888,49.579566529878406,0.0931977114559974,True,False,True
1.9424289164189832,43.428825340122685,0.14451282479015895,True,False,True
1.0069404019654506,27.03597946795943,0.013686975910864816,True,False,True
1.0995936520619134,60.18281432610197,0.1715415019308415,True,False,True
1.8285178826015123,43.57542023629444,0.08570345025612659,True,False,True
1.3470743204701583,89.33144112823486,0.25157804723657307,True,False,True
1.6730379403774187,156.49481965177165,0.39541466903395883,True,False,True
1.5842954279089467,167.0068182818693,0.5972991795655682,True,False,True
1.0526113433794602,27.898331425124635,0.07698478053152019,True,False,True
1.8205333221118392,99.65051836752507,0.5095450408553958,True,False,True
1.810174919456343,117.92393435560311,0.5099914272458922,True,False,True
1.0512086990610958,26.690161820968747,0.025747173784897614,True,False,True
1.276127032835724,93.06490029868982,0.04882507498828238,True,False,True
1.857087736391477,35.00936892091312,0.07796575748367794,True,False,True
1.939686697526705,63.71259231903689,0.301735296265917,True,False,True
1.909346466660761,56.68964911021243,0.1163455819488456,True,False,True
1.5893690349224463,142.61628963013706,0.5661161667425829,True,False,True
1.4324338776591334,40.96948957397704,0.07985987163533907,True,False,True
1.5272094155757538,94.51736908914023,0.46285614658377355,True,False,True
1.1216398059557098,102.47852781517271,0.37243396116197136,True,False,True
1.5831447751579988,57.72757511946802,0.11822440742359577,True,False,True
1.2269593045320668,162.6506905069843,0.8582498317423853,False,True,True
1.0901593721854916,58.12387745243949,0.19094142485712112,True,False,True
1.4494223450888781,57.08339742904779,0.060941171710330456,True,False,True
0.7997306734960662,121.20636699638577,0.622599072721036,True,False,True
1.5553864777725612,85.35580964121021,0.31034310754531524,True,False,True
1.7725350248314893,126.83955091521675,0.6120769163761473,True,False,True
1.7337269263977404,26.753571837970853,0.03164452065365232,True,False,True
1.983367894381239,25.91487606472083,0.06164026494778989,True,False,True
1.9207654404653463,128.80902343074746,0.32100485790894645,True,False,True
1.7179204624268332,67.99493839080637,0.06273304726271914,True,False,True
1.929277044869747,149.99830153592097,0.7947088597796242,False,True,True
1.5262882270998037,40.86181916307275,0.019061301605518464,True,False,True
1.8557641488184804,42.04185280571157,0.16294524490771917,True,False,True
1.3153672394090943,151.06902917486502,0.45944682316005414,True,False,True
1.6151711490410947,63.29795674951804,0.07518809293010185,True,False,True
1.9864875587379767,19.21592080342122,0.04177297463179766,True,False,True
1.6857397158632008,38.55251463748768,0.20536026013800557,True,False,True
1.5945557654296993,134.67315851774052,0.6832014673612161,True,False,True
1.8467643207484525,46.4538055159385,0.28113740141405735,True,False,True
1.3570021848897782,97.26208633232166,0.5160197303518148,True,False,True
1.8803470168957686,146.05733584321825,0.7780302572190191,True,False,True
0.529392874271382,106.82398812928454,0.6019727823567126,True,False,True
1.2398757074713658,131.72082677233686,0.640186469611638,True,False,True
1.123457024147025,170.9643523082714,0.4827160606647634,True,False,True
1.272149935205315,52.36193743771143,0.08267312125711587,True,False,True
1.9630076627364097,153.3370502305692,0.39000163226912665,True,False,True
0.5760575874322067,96.43584890706863,0.19589118629038252,True,False,True
1.336996439119552,35.24768623209427,0.16287541110543266,True,False,True
1.6625159139223613,23.098680778469273,0.06859365572720966,True,False,True
1.7784166909531591,57.38166204569751,0.19525880920980326,True,False,True
1.836129162858668,24.090258012710983,0.03206443755831463,True,False,True
1.4527953252645494,135.05201014581107,0.2799703649092811,True,False,True
1.2076800801892855,97.55147275094399,0.4108833716497928,True,False,True
1.8962827240276252,7.740913732720837,0.012572029622114882,True,False,True
0.9921848708941637,153.7021307223387,0.31530517778755995,True,False,True
0.8042354126839072,104.3005074178697,0.45995744936705973,True,False,True
1.2281842721120189,77.5027841431159,0.12186818721034563,True,False,True
1.579516898710591,24.664157935799324,0.050444395768357395,True,False,True
1.9606708138305364,82.22378337332518,0.3549275922589971,True,False,True
1.7998782206971644,160.53216152898344,0.4132037123194599,True,False,True
1.6524028339046097,102.27374452951491,0.2040445222664465,True,False,True
1.6095461345036988,80.36277266090237,0.45191965975955706,True,False,True
1.7702240638717257,106.41666947425449,0.23523374091582336,True,False,True
1.7113124914409696,96.34807444289498,0.3915832941436192,True,False,True
1.3908399892288978,109.13577994352258,0.10061875379114885,True,False,True
0.9052534630128677,34.820176380150535,0.0405006685449186,True,False,True
1.1714016740808624,26.017346724969276,0.0062418403085732965,True,False,True
1.8431612184860304,2.191173641954891,0.019560248394512793,True,False,True
1.7185541057819134,145.57192793055896,0.9001104741960259,False,True,False
1.854449553366957,0.5668761483746565,0.035301499931406674,True,False,True
0.7086861755292181,44.82406273133355,0.19408112878245057,True,False,True
1.9315648240700365,102.4342530908865,0.27262835850760164,True,False,True
1.4514868897312012,87.46506414025612,0.2611576167909807,True,False,True
1.9495146288119514,157.75801242756125,0.8834532956567929,False,True,True
1.6817129482654611,37.31085231288089,0.011562672228459627,True,False,True
1.6359695009768258,19.948725976743745,0.02179136280764666,True,False,True
1.8705160055892271,28.072771645850345,0.015718280241646516,True,False,True
1.7103692293599282,87.12187784242198,0.46765192993500254,True,False,True
1.7540396000584273,50.96595857605124,0.14366173187904005,True,False,True
1.0803141473661666,79.69437783897914,0.30584761676349037,True,False,True
1.6910319171020332,120.37686684748911,0.28515296727111705,True,False,True
1.4181307708000508,56.2413779054571,0.09828580108165225,True,False,True
0.8456301228909056,116.39948625379698,0.4581492371796781,True,False,True
1.0213822589101862,3.438071253814484,0.019638924424343368,True,False,True
1.5657286626330633,95.58063912380142,0.3173056689580719,True,False,True
1.2805868827605027,158.78191877470243,0.19238732641226575,True,False,True
1.8294121522199949,163.47766836516502,0.8816949810603452,False,True,False
0.9930139297432826,100.06310819905214,0.49376261507129,True,False,True
```

### `results/numbers.tex`

```latex
% generated by main.py — do not edit
\newcommand{\RNChecks}{21}
\newcommand{\RLmiTime}{0.7}
\newcommand{\RAlphaPosMin}{0.53}
\newcommand{\RAlphaPosMax}{0.78}
\newcommand{\RAlphaRollPitch}{9.1}
\newcommand{\RAlphaYaw}{2.1}
\newcommand{\RTauBudgetXY}{0.234}
\newcommand{\RTauBudgetZ}{0.0288}
\newcommand{\RAccelBudgetXY}{3.0}
\newcommand{\RAccelBudgetZ}{2.31}
\newcommand{\RLevelBound}{0.092}
\newcommand{\RGainKvX}{1.13}
\newcommand{\RGainKvZ}{1.06}
\newcommand{\RSettleTime}{8.7}
\newcommand{\RFinalErr}{2\times10^{-9}}
\newcommand{\RCertRollDeg}{37}
\newcommand{\RPdRms}{1.3}
\newcommand{\RFMin}{1.72}
\newcommand{\RFMax}{2.65}
\newcommand{\RPaperFMin}{$-$36}
\newcommand{\RPaperFMax}{46}
\newcommand{\RPaperSettle}{3.3}
\newcommand{\RMcN}{500}
\newcommand{\RMcTime}{105}
\newcommand{\RMcConverged}{95.6}
\newcommand{\RMcFeasible}{97.8}
\newcommand{\RMcGround}{4.4}
\newcommand{\RMcInside}{138}
\newcommand{\RMcInsideConv}{100}
\newcommand{\RMcSmallFeasible}{100}
\newcommand{\RRuntime}{111}
```

### `results/summary.json`

```json
{
  "symbolic_checks": {
    "passed": 21,
    "total": 21
  },
  "lmi": {
    "solve_time_s": 0.6934282779693604,
    "gains": {
      "kx": 0.45265588085333397,
      "ky": 0.7517456215653308,
      "kz": 0.43244853904579333,
      "kv": [
        1.132835582734081,
        1.5563876317010883,
        1.0641397212205823
      ],
      "G": [
        0.046128526848874474,
        0.046128526848874474,
        0.8777258742526066
      ],
      "kw": [
        0.06399482131924981,
        0.06399482131924981,
        0.024874169018146767
      ]
    },
    "alpha_position": {
      "x": 0.5662500000000001,
      "y": 0.7781250000000001,
      "z": 0.531875
    },
    "alpha_attitude": [
      9.14,
      9.14,
      2.0725000000000002
    ],
    "torque_budget_Nm": [
      0.23399999999999999,
      0.23399999999999999,
      0.028800000000000006
    ],
    "accel_budget_xy": 3.0,
    "accel_budget_z": 2.3100000000000005,
    "attitude_level_bound": 0.09225705369774895,
    "certified_roll_deg": 36.84334846629313
  },
  "mission": {
    "lmi_full": {
      "settling_time_5cm_s": 8.72,
      "final_error_m": 2.202626020949773e-09,
      "steady_rms_error_m": 1.3284051423277646e-07,
      "motor_force_min_N": 1.7183112637465672,
      "motor_force_max_N": 2.645144041871075,
      "within_motor_limits": true,
      "max_V2_increase": 5.933056993593227e-14
    },
    "lmi_pd": {
      "settling_time_5cm_s": 8.662,
      "final_error_m": 0.001664447853377759,
      "steady_rms_error_m": 0.0012577376538188516,
      "motor_force_min_N": 1.718190621586002,
      "motor_force_max_N": 2.6448897488506993,
      "within_motor_limits": true,
      "max_V2_increase": 1.5771631906357105e-11
    },
    "paper_full": {
      "settling_time_5cm_s": 3.2600000000000002,
      "final_error_m": 6.899730177035242e-14,
      "steady_rms_error_m": 3.116396147453779e-08,
      "motor_force_min_N": -36.44969879593722,
      "motor_force_max_N": 45.765779479099834,
      "within_motor_limits": false,
      "max_V2_increase": 2.930988785011723e-14
    }
  },
  "monte_carlo": {
    "n": 500,
    "time_s": 104.84709095954895,
    "converged": 0.956,
    "feasible": 0.978,
    "hit_ground": 0.044,
    "n_inside_certified_set": 138,
    "converged_inside_certified_set": 1.0,
    "failures_inside_all_ground": true,
    "feasible_small_perturbation": 1.0
  },
  "unicycle": {
    "final_cross_track_m": 2.940926925321108e-06,
    "V_monotone": true
  },
  "runtime_s": 110.94870829582214
}
```

### `src/control/__init__.py`

```python

```

### `src/control/cascade_controller.py`

```python
"""
src/control/cascade_controller.py

Geometric cascade controller for a quadrotor — Lee, Leok, McClamroch (2010),
with the weighted attitude error of Lee (2012) so that every axis can have
its own gain.

  e_p, e_v : position / velocity error           (world frame)
  R_e      : R_dᵀR,  attitude error rotation
  e_R      : ½ vee(G R_e − R_eᵀG),  G = diag(g) ≻ 0   (body frame)
  Ψ        : ½ tr(G (I − R_e))
  e_Ω      : Ω − R_eᵀΩ_d                           (body frame)

  a_des = K_v(ṗ_d − K e_p − ṗ) + p̈_d            PD on position + feedforward
  F_des = m(a_des + g e₃),  T = ‖F_des‖,  b₃d = F_des / T
  R_d   = [b₂d × b₃d, b₂d, b₃d],  b₂d ∥ b₃d × [cos ψ_d, sin ψ_d, 0]
  τ     = −e_R − K_Ω e_Ω  [+ Ω×JΩ − J(Ω̂ R_eᵀΩ_d − R_eᵀΩ̇_d)   if feedforward]

Only with the bracketed terms is the attitude error system autonomous, which
the stability analysis (src/symbolic/quadcopter_certified_lyapunov.py)
requires.  Ω_d and Ω̇_d come from backward differences of R_d at the control
rate (exact on the steady circle, O(dt) in transients).

Mass, inertia, allocation and motor limits are read from the MuJoCo model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

G_ACC = 9.81
E3 = np.array([0.0, 0.0, 1.0])


def vee(S: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix → axial vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


@dataclass
class QuadReference:
    pos:   np.ndarray                                           # p_d
    vel:   np.ndarray                                           # ṗ_d
    yaw:   float                                                # ψ_d
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # p̈_d


def circle_reference(t: float, radius: float, omega: float, z_d: float) -> QuadReference:
    """p_d(t) = [R cos ωt, R sin ωt, z_d] with exact derivatives, yaw tangent to the circle."""
    c, s = np.cos(omega * t), np.sin(omega * t)
    return QuadReference(
        pos   = np.array([radius * c, radius * s, z_d]),
        vel   = np.array([-radius * omega * s, radius * omega * c, 0.0]),
        yaw   = omega * t + np.pi / 2.0,
        accel = np.array([-radius * omega**2 * c, -radius * omega**2 * s, 0.0]),
    )


def desired_attitude(F_des: np.ndarray, yaw: float) -> np.ndarray:
    """R_d with third axis along F_des and first axis as close as possible to heading yaw."""
    z_d = F_des / np.linalg.norm(F_des)
    y_d = np.cross(z_d, [np.cos(yaw), np.sin(yaw), 0.0])
    y_d /= np.linalg.norm(y_d)
    return np.column_stack([np.cross(y_d, z_d), y_d, z_d])


def vehicle_parameters(model: mujoco.MjModel, body: str = "quad") -> dict:
    """Mass, principal inertia, allocation matrix A ([T, τ] = A f) and motor range, from the MJCF."""
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    if not np.allclose(model.body_iquat[b], [1.0, 0.0, 0.0, 0.0]):
        raise ValueError("Principal inertia axes must coincide with the body frame.")
    data = mujoco.MjData(model)                      # qpos0: level pose, so body = world axes
    mujoco.mj_forward(model, data)
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz,
                            data.moment_rowadr, data.moment_colind)
    return {
        "mass": float(model.body_subtreemass[b]),
        "J": model.body_inertia[b].copy(),
        "A": moment[:, 2:6].T,                       # free-joint dofs: [Fz, τx, τy, τz]
        "f_range": model.actuator_ctrlrange.copy(),
    }


class GeometricController:
    """
    gains: kx, ky, kz, kv (scalar or per axis), G or kR (scalar ⇒ G = kR·I),
           kw (scalar or per axis), saturate.
    feedforward=False gives the PD-only law (no Ω×JΩ, no desired-frame terms).
    """

    def __init__(self, model: mujoco.MjModel, gains: dict, feedforward: bool = True):
        p = vehicle_parameters(model)
        self.m, self.J, self.f_range = p["mass"], p["J"], p["f_range"]
        self.A_inv = np.linalg.inv(p["A"])
        self.dt = model.opt.timestep
        self.feedforward = feedforward

        per_axis = lambda v: np.broadcast_to(np.asarray(v, float), (3,)).copy()
        self.K  = np.array([gains["kx"], gains["ky"], gains["kz"]], float)
        self.kv = per_axis(gains["kv"])
        self.G  = per_axis(gains["G"] if "G" in gains else gains["kR"])
        self.kw = per_axis(gains["kw"])
        self.saturate = bool(gains.get("saturate", False))
        self._R_d_prev = self._W_d_prev = None

    def _desired_rates(self, R_d):
        """Ω_d = log(R_d,prevᵀ R_d)/dt,  Ω̇_d = (Ω_d − Ω_d,prev)/dt  (zero until history exists)."""
        W_d, dW_d = np.zeros(3), np.zeros(3)
        if self._R_d_prev is not None:
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (self._R_d_prev.T @ R_d).ravel())
            mujoco.mju_quat2Vel(W_d, q, self.dt)
            if self._W_d_prev is not None:
                dW_d = (W_d - self._W_d_prev) / self.dt
            self._W_d_prev = W_d
        self._R_d_prev = R_d
        return W_d, dW_d

    def compute(self, data: mujoco.MjData, ref: QuadReference) -> tuple[np.ndarray, dict]:
        """Motor forces for the current MuJoCo state.  Call exactly once per period dt."""
        pos, vel, W = data.qpos[:3], data.qvel[:3], data.qvel[3:6]   # Ω in body frame
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, data.qpos[3:7])                          # current state (xmat lags one step)
        R = R.reshape(3, 3)

        e_p, e_v = pos - ref.pos, vel - ref.vel
        a_des = -self.kv * self.K * e_p - self.kv * e_v + ref.accel
        F_des = self.m * (a_des + G_ACC * E3)
        T = float(np.linalg.norm(F_des))
        R_d = desired_attitude(F_des, ref.yaw)
        W_d, dW_d = self._desired_rates(R_d)

        R_e = R_d.T @ R
        Gm  = np.diag(self.G)
        e_R = 0.5 * vee(Gm @ R_e - R_e.T @ Gm)
        e_W = W - R_e.T @ W_d
        tau = -e_R - self.kw * e_W
        if self.feedforward:
            tau += np.cross(W, self.J * W) - self.J * (np.cross(W, R_e.T @ W_d) - R_e.T @ dW_d)

        forces = self.A_inv @ np.r_[T, tau]
        if self.saturate:
            forces = np.clip(forces, self.f_range[:, 0], self.f_range[:, 1])

        Psi = 0.5 * float(np.trace(Gm @ (np.eye(3) - R_e)))
        log = {
            "ex": e_p[0], "ey": e_p[1], "ez": e_p[2],
            "T": T, "Psi": Psi,
            "V2": Psi + 0.5 * float(e_W @ (self.J * e_W)),
            "Wdx": W_d[0], "Wdy": W_d[1], "Wdz": W_d[2],
            "tau_x": tau[0], "tau_y": tau[1], "tau_z": tau[2],
            "f1": forces[0], "f2": forces[1], "f3": forces[2], "f4": forces[3],
        }
        return forces, log
```

### `src/control/path_following_controller.py`

```python
"""
src/control/path_following_controller.py

Path-following reference generator for the quadrotor.

A virtual target moves along the path p_d(γ) with
    γ̇ = v_d + k_γ e_s,    e_s = t̂(γ)ᵀ (p − p_d(γ))   (along-track error).
It produces a QuadReference (p_d, ṗ_d, p̈_d, ψ_d) that the cascade
controller tracks like any other reference.  ṗ_d and p̈_d are the exact
derivatives of p_d(γ(t)) along the closed loop, so the stability result
of the trajectory-tracking case carries over unchanged
(see src/symbolic/quadcopter_path_following.py).

The γ-law does not change how fast e_p converges: it only decides where
the reference is.  When the vehicle lags (e_s < 0) the target slows down,
when it leads (e_s > 0) the target speeds up.

Usage
-----
path = CirclePath(R=3.0, z_d=3.0)
pfc  = PathFollowingController.from_config(cfg, path)
pfc.reset(initial_pos)
# in _loop_step:
ref, gamma, gamma_dot = pfc.step(data.qpos[:3], data.qvel[:3], dt)
forces, log = controller.compute(data, ref)
"""

from __future__ import annotations

import numpy as np

from src.control.cascade_controller import QuadReference


# ──────────────────────────────────────────────────────────────────────────────
# Path abstraction
# ──────────────────────────────────────────────────────────────────────────────

class CirclePath:
    """
    Planar circle at constant altitude, p_d(γ) = [R cos γ, R sin γ, z_d],
    γ in radians (one lap = 2π).
    """

    def __init__(self, R: float, z_d: float):
        self.R   = R
        self.z_d = z_d

    def position(self, gamma: float) -> np.ndarray:
        return np.array([self.R * np.cos(gamma), self.R * np.sin(gamma), self.z_d])

    def tangent(self, gamma: float) -> np.ndarray:
        """∂p_d/∂γ."""
        return np.array([-self.R * np.sin(gamma), self.R * np.cos(gamma), 0.0])

    def curvature_term(self, gamma: float) -> np.ndarray:
        """∂²p_d/∂γ²."""
        return np.array([-self.R * np.cos(gamma), -self.R * np.sin(gamma), 0.0])

    def unit_tangent(self, gamma: float) -> np.ndarray:
        return np.array([-np.sin(gamma), np.cos(gamma), 0.0])

    def yaw(self, gamma: float) -> float:
        """Heading tangent to the path: ψ_d = γ + π/2."""
        return float(gamma + np.pi / 2.0)

    def closest_gamma(self, p: np.ndarray) -> float:
        """γ of the point of the circle closest to p (in the xy plane)."""
        return float(np.arctan2(p[1], p[0]))


# ──────────────────────────────────────────────────────────────────────────────
# Path-following reference generator
# ──────────────────────────────────────────────────────────────────────────────

class PathFollowingController:
    """
    Parameters (from config["path_following"])
    ------------------------------------------
    v_d     : nominal path rate [rad/s]
    k_gamma : along-track correction gain [rad/(s·m)]
    """

    def __init__(self, path: CirclePath, v_d: float, k_gamma: float):
        self.path    = path
        self.v_d     = v_d
        self.k_gamma = k_gamma
        self._gamma  = 0.0

    @classmethod
    def from_config(cls, cfg: dict, path: CirclePath) -> "PathFollowingController":
        pf = cfg.get("path_following", {})
        return cls(
            path    = path,
            v_d     = float(pf.get("v_d",     0.5)),
            k_gamma = float(pf.get("k_gamma", 1.0)),
        )

    def reset(self, initial_pos: np.ndarray) -> None:
        """Start the virtual target at the point of the path closest to the vehicle."""
        self._gamma = self.path.closest_gamma(initial_pos)

    @property
    def gamma(self) -> float:
        return self._gamma

    def step(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        dt:  float,
    ) -> tuple[QuadReference, float, float]:
        """
        Return the reference at the current γ, then advance γ by one Euler step.

        p̈_d = (∂²p_d/∂γ²) γ̇² + (∂p_d/∂γ) γ̈,   γ̈ = k_γ ė_s,
        ė_s = (∂t̂/∂γ γ̇)ᵀ e_p + t̂ᵀ (ṗ − (∂p_d/∂γ) γ̇).
        """
        gamma = self._gamma
        p_d   = self.path.position(gamma)
        dp_d  = self.path.tangent(gamma)
        ddp_d = self.path.curvature_term(gamma)
        t_hat = self.path.unit_tangent(gamma)
        dt_hat = ddp_d / self.path.R                       # ∂t̂/∂γ for the circle

        e_p = pos - p_d
        e_s = float(t_hat @ e_p)
        gamma_dot = self.v_d + self.k_gamma * e_s

        e_s_dot    = float(dt_hat @ e_p) * gamma_dot + float(t_hat @ (vel - dp_d * gamma_dot))
        gamma_ddot = self.k_gamma * e_s_dot

        ref = QuadReference(
            pos   = p_d,
            vel   = dp_d * gamma_dot,
            yaw   = self.path.yaw(gamma),
            accel = ddp_d * gamma_dot**2 + dp_d * gamma_ddot,
        )

        self._gamma = gamma + gamma_dot * dt
        return ref, gamma, gamma_dot
```

### `src/design/__init__.py`

```python

```

### `src/design/lmi_gains.py`

```python
"""
src/design/lmi_gains.py

Actuator-aware gain synthesis with LMIs (state feedback, input-bounded).

Port of fsf_input_bounded.m from the tank-LMI project (same notation):
for ẋ = A x + B u, u = K x, find W = Wᵀ ≻ 0, X with K = X W⁻¹ such that

    He(A W + B X) ⪯ −2α W                     (E = {xᵀW⁻¹x ≤ 1} invariant, rate α)
    [ 1   x0ᵀ ; x0  W ] ⪰ 0   for every x0    (design envelope inside E)
    [ ū_i²  X_i ; X_iᵀ  W ] ⪰ 0   for each i  (|K_i x| ≤ ū_i on all of E)

and maximise α by bisection (α enters bilinearly through α W).
Every trajectory of the linear model that starts in E stays in E, decays at
rate α and never asks for more than ū.  No saturation is used.

Applied to the two reduced models of the cascade, axis by axis:

  position   ė_p = e_v, ė_v = u          u = a_des − p̈_d (exact when R = R_d)
  attitude   θ̇ = e_Ω,   J_j ė_Ω = u      u = PD part of the torque (linearised
                                            error dynamics after the exact
                                            gyroscopic / feedforward cancellation)

The input budgets come from the motor limits 0 ≤ f_i ≤ f_max, split between
collective thrust and torques (see motor_budgets()).
"""

from __future__ import annotations

from dataclasses import dataclass

import cvxpy as cp
import numpy as np

DOUBLE_INTEGRATOR_A = np.array([[0.0, 1.0], [0.0, 0.0]])


@dataclass
class LMIDesign:
    K: np.ndarray        # u = K x
    W: np.ndarray        # E = {x : xᵀ W⁻¹ x ≤ 1}
    alpha: float         # certified decay rate of V = xᵀW⁻¹x  (‖x‖ decays at least as e^{−α t})
    ubar: float


def _solve_alpha(A, B, ubar, X0, alpha, eps=1e-6):
    n, m = B.shape
    W = cp.Variable((n, n), symmetric=True)
    X = cp.Variable((m, n))
    M = A @ W + B @ X
    cons = [W >> eps * np.eye(n), M + M.T << -2 * alpha * W]
    for x0 in X0:
        x0 = np.asarray(x0, float).reshape(n, 1)
        cons.append(cp.bmat([[np.ones((1, 1)), x0.T], [x0, W]]) >> 0)
    for i in range(m):
        cons.append(cp.bmat([[np.array([[ubar[i] ** 2]]), X[i:i + 1, :]], [X[i:i + 1, :].T, W]]) >> 0)
    prob = cp.Problem(cp.Minimize(0), cons)
    try:
        prob.solve(solver=cp.CLARABEL)
    except cp.SolverError:
        return False, None, None
    ok = prob.status == cp.OPTIMAL
    return ok, (W.value if ok else None), (X.value if ok else None)


def fsf_input_bounded(A, B, ubar, X0, alpha_max: float = 50.0, tol: float = 1e-3) -> LMIDesign:
    """Largest certified rate α ≤ alpha_max with |u_i| ≤ ū_i on an invariant ellipsoid containing X0."""
    A = np.asarray(A, float); B = np.asarray(B, float).reshape(A.shape[0], -1)
    ubar = np.atleast_1d(np.asarray(ubar, float))
    lo, hi = 0.0, 1e-2
    while hi < alpha_max and _solve_alpha(A, B, ubar, X0, hi)[0]:
        lo, hi = hi, min(2 * hi, alpha_max)
    if hi >= alpha_max and _solve_alpha(A, B, ubar, X0, alpha_max)[0]:
        lo = hi = alpha_max
    while hi - lo > tol * max(hi, 1.0):
        mid = 0.5 * (lo + hi)
        if _solve_alpha(A, B, ubar, X0, mid)[0]:
            lo = mid
        else:
            hi = mid
    ok, W, X = _solve_alpha(A, B, ubar, X0, lo)
    if not ok or lo <= 0:
        raise RuntimeError("No positive-rate design satisfies the input budget on the requested envelope.")
    return LMIDesign(K=X @ np.linalg.inv(W), W=W, alpha=lo, ubar=float(ubar[0]))


def _box_vertices(a: float, b: float) -> list[np.ndarray]:
    return [np.array([sa * a, sb * b]) for sa in (-1, 1) for sb in (-1, 1)]


# ──────────────────────────────────────────────────────────────────────────────
# Budgets from the motor limits
# ──────────────────────────────────────────────────────────────────────────────

def motor_budgets(cfg: dict, vehicle: dict) -> dict:
    """
    f_i = T/4 + δ_i,  |δ_i| ≤ |τx|/(4d) + |τy|/(4d) + |τz|/(4k_m)  (from A⁻¹).
    Reserving σ newtons per motor for torques and keeping T/4 ∈ [σ, f_max − σ]
    guarantees 0 ≤ f_i ≤ f_max.  σ is split between the three torque axes.

    Collective thrust with u = a_des − p̈_d and |u_x|,|u_y| ≤ ū_xy, |u_z| ≤ ū_z:
        T/m ≥ g − ū_z,     T/m ≤ √((a_h + √2 ū_xy)² + (g + ū_z)²),
    a_h = max horizontal ‖p̈_d‖ (= R ω² on the circle).
    """
    m, g, A = vehicle["mass"], 9.81, vehicle["A"]
    arm = np.abs(A[1:, :]).max(axis=1)                      # [d, d, k_m] from the allocation matrix
    f_max = float(vehicle["f_range"][:, 1].min())
    des = cfg["design"]
    sigma = des["torque_reserve_per_motor"]
    tau_max = 4 * sigma * np.asarray(des["torque_split"], float) * arm

    ubar_z = g - 4 * sigma / m                              # keeps T/4 ≥ σ
    ubar_xy = des["accel_budget_xy"]
    traj = cfg["trajectory"]
    a_h = traj["radius"] * traj["omega"] ** 2
    T_max = m * np.hypot(a_h + np.sqrt(2) * ubar_xy, g + ubar_z)
    if ubar_z <= 0 or T_max / 4 > f_max - sigma:
        raise ValueError("Thrust budget infeasible: lower accel_budget_xy or torque_reserve_per_motor.")
    return {"sigma": sigma, "tau_max": tau_max, "ubar_xy": ubar_xy, "ubar_z": ubar_z,
            "T_max_per_motor": T_max / 4, "f_max": f_max}


# ──────────────────────────────────────────────────────────────────────────────
# Gain synthesis for the cascade
# ──────────────────────────────────────────────────────────────────────────────

def design_position(cfg: dict, b: dict, e_p0: np.ndarray, e_v0: np.ndarray) -> dict:
    """Per axis: u = K x = −k_v k_i e_p − k_v e_v  ⇒  k_v = −K₂,  k_i = K₁/K₂."""
    env = cfg["design"]
    out = {}
    for i, ax in enumerate("xyz"):
        ubar = b["ubar_z"] if ax == "z" else b["ubar_xy"]
        X0 = _box_vertices(max(abs(e_p0[i]), env["min_position_envelope"]),
                           max(abs(e_v0[i]), env["min_velocity_envelope"]))
        des = fsf_input_bounded(DOUBLE_INTEGRATOR_A, np.array([[0.0], [1.0]]), [ubar], X0,
                                alpha_max=env["alpha_max_position"])
        K1, K2 = des.K.ravel()
        out[ax] = {"kv": -K2, "k": K1 / K2, "alpha": des.alpha, "ubar": ubar, "W": des.W}
    return out


def design_attitude(cfg: dict, b: dict, J: np.ndarray, theta0: np.ndarray) -> dict:
    """
    Per body axis j: u_j = −κ_j θ_j − k_Ω,j e_Ω,j.  The diagonal stiffness κ is
    realised on SO(3) by the weighted error e_R = ½ vee(G R_e − R_eᵀG) with
    g_j = Σκ − 2κ_j (linearisation e_R ≈ ½(tr G·I − G) θ = diag(κ) θ).
    """
    env = cfg["design"]
    kappa, kW, alphas, Ws = np.zeros(3), np.zeros(3), np.zeros(3), []
    # roll and pitch share one envelope (J_x = J_y, equal budgets) so that κ_x = κ_y,
    # which together with κ_z < 2κ_x makes every g_j positive
    th_env = np.abs(theta0).copy()
    th_env[:2] = th_env[:2].max()
    for j in range(3):
        ubar = env["pd_torque_fraction"] * b["tau_max"][j]
        X0 = _box_vertices(max(th_env[j], np.radians(env["min_attitude_envelope_deg"])),
                           env["min_rate_envelope"])
        des = fsf_input_bounded(DOUBLE_INTEGRATOR_A, np.array([[0.0], [1.0 / J[j]]]), [ubar], X0,
                                alpha_max=env["alpha_max_attitude"])
        kappa[j], kW[j] = -des.K[0, 0], -des.K[0, 1]
        alphas[j] = des.alpha
        Ws.append(des.W)
    G = kappa.sum() - 2 * kappa
    if np.any(G <= 0):
        raise ValueError(f"Stiffness {kappa} not realisable by a positive G (need κ_j < Σκ/2).")
    return {"kappa": kappa, "G": G, "kW": kW, "alpha": alphas, "W": Ws,
            "tau_pd_max": env["pd_torque_fraction"] * b["tau_max"]}


def design_cascade_gains(cfg: dict, model) -> dict:
    """
    Position gains from the mission start, then attitude gains from the initial
    attitude error that those position gains produce.  Returns the controller
    gains plus the certificates (rates, ellipsoids, budgets).
    """
    from scipy.spatial.transform import Rotation
    from src.control.cascade_controller import G_ACC, E3, circle_reference, desired_attitude, vehicle_parameters

    vehicle = vehicle_parameters(model)
    b = motor_budgets(cfg, vehicle)
    ic, traj = cfg["initial_state"], cfg["trajectory"]
    ref0 = circle_reference(0.0, traj["radius"], traj["omega"], traj["z_height"])
    e_p0, e_v0 = np.asarray(ic["pos"], float) - ref0.pos, -ref0.vel
    pos = design_position(cfg, b, e_p0, e_v0)

    kv = np.array([pos[a]["kv"] for a in "xyz"])
    k = np.array([pos[a]["k"] for a in "xyz"])
    R_d0 = desired_attitude(-kv * k * e_p0 - kv * e_v0 + ref0.accel + G_ACC * E3, ref0.yaw)
    R0 = Rotation.from_euler("z", ic["yaw"]).as_matrix()
    theta0 = Rotation.from_matrix(R_d0.T @ R0).as_rotvec()

    att = design_attitude(cfg, b, vehicle["J"], theta0)
    return {
        "controller": {"kx": float(k[0]), "ky": float(k[1]), "kz": float(k[2]),
                       "kv": kv.tolist(), "G": att["G"].tolist(), "kw": att["kW"].tolist()},
        "position": pos, "attitude": att, "theta0": theta0, "budgets": b,
    }
```

### `src/experiments.py`

```python
"""
src/experiments.py

Headless experiments behind every number and figure of the report.

load()                config/<name>.yaml with paths resolved
simulate_quad()       closed-loop quadrotor in MuJoCo (circle or path following)
roa_monte_carlo()     random initial positions/attitudes → convergence statistics
render_quad()         replay a simulate_quad() run in MuJoCo's renderer → mp4
simulate_unicycle()   kinematic unicycle with the Lyapunov path-following law
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pandas as pd
import yaml
from joblib import Parallel, delayed
from scipy.integrate import solve_ivp
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
    rows = Parallel(n_jobs=n_jobs)(
        delayed(_roa_rollout)(cfg, gains, offsets[i], rotvecs[i], mc, f_max) for i in range(n))
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


# ──────────────────────────────────────────────────────────────────────────────
# Unicycle
# ──────────────────────────────────────────────────────────────────────────────

def simulate_unicycle(cfg: dict, n_points: int = 1500) -> pd.DataFrame:
    """ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω(e, θ̃) integrated with scipy's RK45."""
    c, ic = cfg["control"], cfg["initial_state"]
    v, R, ky, kth = c["v"], c["R"], c["ky"], c["ktheta"]

    def errors(x, y, th):
        e = np.hypot(x, y) - R
        t_tilde = np.angle(np.exp(1j * (th - np.arctan2(y, x) - np.pi / 2)))
        return e, t_tilde

    def rhs(_, s):
        e, tt = errors(*s)
        omega = v * np.cos(tt) / (R + e) + ky * e * v - kth * np.sin(tt)
        return [v * np.cos(s[2]), v * np.sin(s[2]), omega]

    t = np.linspace(0, cfg["t_end"], n_points)
    sol = solve_ivp(rhs, (0, cfg["t_end"]), [ic["x"], ic["y"], ic["theta"]], t_eval=t, rtol=1e-9, atol=1e-9)
    x, y, th = sol.y
    e, tt = errors(x, y, th)
    return pd.DataFrame({"t": t, "x": x, "y": y, "e": e, "t_tilde": tt,
                         "V": 0.5 * e**2 + (1 - np.cos(tt)) / ky})
```

### `src/symbolic/lyapunov.py`

```python
"""
src/symbolic/lyapunov.py

Lyapunov path-following law for a unicycle on a circle of radius R
(counter-clockwise), kinematic model ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω
with constant forward speed v > 0.

Errors:  e = √(x² + y²) − R  (cross-track),  θ̃ = θ − (atan2(y, x) + π/2).

Result (all identities computed below):
    V = ½e² + (1 − cos θ̃)/k_y,   V̇ = −(k_θ/k_y) sin² θ̃ ≤ 0.
LaSalle on the compact sublevel set Ω_c = {V ≤ c}, c < ½R² (which keeps
|e| < R, i.e. away from the singularity at the circle centre) gives
convergence to M = {(0, 0), (0, π)}.  (0, π) — driving the circle
clockwise — is a saddle, so it attracts only a measure-zero set.  If in
addition c < 2/k_y, (0, π) ∉ Ω_c and every trajectory starting in Ω_c
converges to (0, 0).  The result is not global: initial conditions with
V ≥ ½R² are not covered.
"""

import sympy as sp


def derive_circular_path_following() -> dict:
    e, t_tilde = sp.symbols(r"e \tilde{\theta}", real=True)
    v, R = sp.symbols("v R", positive=True)
    k_y, k_theta = sp.symbols(r"k_y k_\theta", positive=True)
    omega = sp.Symbol(r"\omega", real=True)

    # ── error kinematics ─────────────────────────────────────────────────────
    e_dot       = -v * sp.sin(t_tilde)
    t_p_dot     = v * sp.cos(t_tilde) / (R + e)          # rate of the path tangent angle
    t_tilde_dot = omega - t_p_dot

    # ── Lyapunov function and control law ───────────────────────────────────
    V         = sp.Rational(1, 2) * e**2 + (1 - sp.cos(t_tilde)) / k_y
    omega_law = t_p_dot + k_y * e * v - k_theta * sp.sin(t_tilde)

    V_dot     = sp.diff(V, e) * e_dot + sp.diff(V, t_tilde) * t_tilde_dot
    V_dot_sub = sp.simplify(V_dot.subs(omega, omega_law))

    # ── closed loop, equilibria, linearisation ──────────────────────────────
    f = sp.Matrix([e_dot, t_tilde_dot.subs(omega, omega_law)])
    x = sp.Matrix([e, t_tilde])
    Jac = f.jacobian(x)
    J_good   = sp.simplify(Jac.subs({e: 0, t_tilde: 0}))
    J_saddle = sp.simplify(Jac.subs({e: 0, t_tilde: sp.pi}))

    checks = {
        "V_dot_is_minus_ktheta_over_ky_sin2": sp.simplify(V_dot_sub + k_theta / k_y * sp.sin(t_tilde)**2) == 0,
        "origin_is_equilibrium":  sp.simplify(f.subs({e: 0, t_tilde: 0})) == sp.zeros(2, 1),
        "reverse_is_equilibrium": sp.simplify(f.subs({e: 0, t_tilde: sp.pi})) == sp.zeros(2, 1),
        # 2×2 Hurwitz ⇔ trace < 0 and det > 0
        "origin_hurwitz": bool(sp.simplify(J_good.trace()).is_negative and sp.simplify(J_good.det()).is_positive),
        # det < 0 ⇒ one positive and one negative eigenvalue (saddle)
        "reverse_is_saddle": bool(sp.simplify(J_saddle.det()).is_negative),
    }

    return {
        "V": V,
        "e_dot": e_dot,
        "t_tilde_dot": t_tilde_dot,
        "omega_law": omega_law,
        "V_dot_sub": V_dot_sub,
        "J_origin": J_good,
        "J_reverse": J_saddle,
        "V_at_reverse": V.subs({e: 0, t_tilde: sp.pi}),
        "checks": checks,
    }


if __name__ == "__main__":
    res = derive_circular_path_following()
    print("V     =", res["V"])
    print("V_dot =", res["V_dot_sub"])
    print(res["checks"])
```

### `src/symbolic/quadcopter_certified_lyapunov.py`

```python
"""
src/symbolic/quadcopter_certified_lyapunov.py

Symbolic stability analysis of the geometric cascade controller
(src/control/cascade_controller.py, feedforward=True) on the full rigid-body model

    ṗ = v,      m v̇ = T R e₃ − m g e₃,
    Ṙ = R Ω̂,    J Ω̇ = τ − Ω × JΩ.

Every identity below is computed by SymPy from these equations and the
control law; nothing is asserted by hand.  Each function returns the
expressions to display plus `checks`, a dict of booleans that are True
only if the corresponding identity simplified to zero.

What is proved (and what is not)
--------------------------------
1. derive_attitude_certificate():
     with the full Lee et al. torque law (weighted error, diagonal gains
     G, K_Ω) the attitude error obeys
     J ė_Ω = −e_R − K_Ω e_Ω,  Ṙ_e = R_e ê_Ω  (autonomous), and
     V₂ = Ψ + ½ e_Ωᵀ J e_Ω  has  V̇₂ = −e_Ωᵀ K_Ω e_Ω  exactly.
     ⇒ (R_e, e_Ω) = (I, 0) is asymptotically stable and every initial
       condition with V₂ < min_{i≠j}(g_i + g_j) converges to it (LaSalle on
       a compact sublevel set that excludes all 180° equilibria).  Locally
       the convergence is exponential.  It is NOT globally asymptotically
       stable: no continuous feedback on SO(3) is (Bhat & Bernstein 2000).
     Without the feedforward terms V̇₂ is sign-indefinite (explicit
     counterexample returned).
2. derive_position_certificate():
     the position/velocity error obeys ė_p = e_v,
     ė_v = −K_v K e_p − K_v e_v + Δ  with  Δ = (T/m)(R − R_d)e₃,
     ‖Δ‖ ≤ (2T/m)√Ψ₁,  Ψ₁ = ½ tr(I − R_e) ≤ Ψ / min g.  For Δ = 0 the system is linear, Hurwitz, and
     V₁ = xᵀPx (P from the Lyapunov equation) proves global exponential
     stability of THAT reduced system.
3. derive_cascade_linearization():
     the closed loop in error coordinates is a cascade
     (attitude → position).  Its linearization along the reference is
     block-triangular with Hurwitz diagonal blocks and a bounded coupling,
     hence uniformly exponentially stable; by Lyapunov's indirect method
     (Khalil, Thm 4.15) the full nonlinear closed loop is LOCALLY
     exponentially stable.  The size of its region of attraction is not
     characterised analytically.

Assumptions shared by all results: unbounded, signed motor forces (no
saturation); F_des ≠ 0 and F_des ∦ x_c so that R_d is defined; exact
Ω_d = vee(R_dᵀṘ_d) and Ω̇_d.  The implementation approximates the last
two by backward differences at the control rate (exact on the
steady-state circle, O(dt) during transients).
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def hat(w: sp.Matrix) -> sp.Matrix:
    """ŵ such that ŵ v = w × v."""
    return sp.Matrix([
        [    0, -w[2],  w[1]],
        [ w[2],     0, -w[0]],
        [-w[1],  w[0],     0],
    ])


def vee(S: sp.Matrix) -> sp.Matrix:
    return sp.Matrix([S[2, 1], S[0, 2], S[1, 0]])


def _is_zero(expr) -> bool:
    """True iff expr (scalar or matrix) simplifies to exactly zero."""
    entries = list(expr) if isinstance(expr, sp.MatrixBase) else [expr]
    return all(sp.expand(e) == 0 or sp.simplify(e) == 0 for e in entries)


def _generic_matrix(name: str) -> sp.Matrix:
    """3×3 matrix of independent real symbols (identities proved for it hold for any R_e)."""
    return sp.Matrix(3, 3, lambda i, j: sp.Symbol(f"{name}_{{{i+1}{j+1}}}", real=True))


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Attitude loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_attitude_certificate() -> dict:
    """
    Exact Lyapunov analysis of the attitude error (R_e, e_Ω), R_e = R_dᵀR, with
    the weighted error of the controller:

        Ψ = ½ tr(G (I − R_e)),   e_R = ½ vee(G R_e − R_eᵀ G),   G = diag(g) ≻ 0,
        τ = −e_R − K_Ω e_Ω + Ω × JΩ − J (Ω̂ R_eᵀ Ω_d − R_eᵀ Ω̇_d),  K_Ω = diag ≻ 0.

    Kinematics.  From Ṙ = RΩ̂, Ṙ_d = R_dΩ̂_d and R ŵ Rᵀ = (Rw)^ on SO(3):
        Ṙ_e = R_e ê_Ω,   e_Ω = Ω − R_eᵀΩ_d.
    The identity R ŵ Rᵀ = (Rw)^ is verified on the Euler–Rodrigues
    parametrisation; the remaining identities are verified for a generic
    3×3 matrix R_e, so they need no small-angle approximation.

    Critical points.  e_R = 0 ⇔ S = G R_e is symmetric ⇒ S² = S Sᵀ = G², which
    forces R_e = I or a rotation by π about some axis n (case analysis on the
    eigenvalues of G).  At such a rotation Ψ = tr G − nᵀG n ≥ min_{i≠j}(g_i + g_j).
    Hence on {V₂ < min_{i≠j}(g_i + g_j)} the only point with e_R = 0 is R_e = I.
    """
    # ── symbols ──────────────────────────────────────────────────────────────
    g = sp.symbols("g_1:4", positive=True)
    kW = [sp.Symbol(rf"k_{{\Omega{i}}}", positive=True) for i in (1, 2, 3)]
    G, KW = sp.diag(*g), sp.diag(*kW)
    J1, J2, J3 = sp.symbols("J_1 J_2 J_3", positive=True)
    J = sp.diag(J1, J2, J3)
    Om   = sp.Matrix(sp.symbols(r"\Omega_1:4", real=True))
    Omd  = sp.Matrix(sp.symbols(r"\Omega_{d1} \Omega_{d2} \Omega_{d3}", real=True))
    dOmd = sp.Matrix(sp.symbols(r"\dot\Omega_{d1} \dot\Omega_{d2} \dot\Omega_{d3}", real=True))
    Re   = _generic_matrix("r")

    # ── SO(3) identity R ŵ Rᵀ = (Rw)^  (Euler–Rodrigues, homogeneous form) ───
    q0, q1, q2, q3 = sp.symbols("q_0:4", real=True)
    w = sp.Matrix(sp.symbols("w_1:4", real=True))
    M = sp.Matrix([
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3),         2*(q1*q3+q0*q2)],
        [2*(q1*q2+q0*q3),         q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2),         2*(q2*q3+q0*q1),         q0**2-q1**2-q2**2+q3**2],
    ])                                            # R = M / |q|²,  M Mᵀ = |q|⁴ I
    n2 = q0**2 + q1**2 + q2**2 + q3**2
    check_conj = _is_zero(M * hat(w) * M.T - n2 * hat(M * w))
    check_orth = _is_zero(M * M.T - n2**2 * sp.eye(3))

    # ── error quantities ────────────────────────────────────────────────────
    e_R  = sp.Rational(1, 2) * vee(G * Re - Re.T * G)
    Psi  = sp.Rational(1, 2) * (G * (sp.eye(3) - Re)).trace()
    e_W  = Om - Re.T * Omd

    # ── Ψ̇ = e_R · e_Ω   (from Ṙ_e = R_e ê_Ω) ────────────────────────────────
    Re_dot  = Re * hat(e_W)
    Psi_dot = -sp.Rational(1, 2) * (G * Re_dot).trace()
    check_Psi_dot = _is_zero(Psi_dot - e_R.dot(e_W))

    # ── ė_Ω  from Euler's equation and Ṙ_eᵀ = −ê_Ω R_eᵀ ─────────────────────
    def e_W_dot(tau):
        Om_dot = J.inv() * (tau - Om.cross(J * Om))
        return Om_dot - (Re_dot.T * Omd + Re.T * dOmd)

    tau_full = (-e_R - KW * e_W + Om.cross(J * Om)
                - J * (hat(Om) * Re.T * Omd - Re.T * dOmd))
    tau_pd   = -e_R - KW * e_W

    check_closed_loop = _is_zero(J * e_W_dot(tau_full) - (-e_R - KW * e_W))

    # ── Lyapunov function and its derivative (J constant and symmetric) ─────
    V2 = Psi + sp.Rational(1, 2) * (e_W.T * J * e_W)[0, 0]

    def V2_dot(tau):
        return Psi_dot + (e_W.T * J * e_W_dot(tau))[0, 0]

    check_V2_dot = _is_zero(V2_dot(tau_full) + (e_W.T * KW * e_W)[0, 0])

    # ── PD law only: residual and an explicit point where V̇₂ > 0 ───────────
    V2_dot_pd = sp.expand(V2_dot(tau_pd))
    residual_pd = sp.expand(V2_dot_pd + (e_W.T * KW * e_W)[0, 0])
    # R_e = I, Ω_d = 0, Ω = e_Ω = (0.1,0,0), Ω̇_d = (−1000,0,0):
    # residual = −e_Ωᵀ J Ω̇_d = 0.35 > k_Ω1‖e_Ω‖² = 0.025
    numeric_point = {**{Re[i, j]: int(i == j) for i in range(3) for j in range(3)},
                     Om[0]: sp.Rational(1, 10), Om[1]: 0, Om[2]: 0,
                     **{Omd[i]: 0 for i in range(3)},
                     dOmd[0]: -1000, dOmd[1]: 0, dOmd[2]: 0,
                     J1: sp.Rational(35, 10000), J2: sp.Rational(35, 10000), J3: sp.Rational(6, 1000),
                     **{gi: sp.Rational(1, 2) for gi in g}, **{ki: sp.Rational(5, 2) for ki in kW}}
    V2_dot_pd_counterexample = V2_dot_pd.subs(numeric_point)

    # In set-point regulation (Ω_d = Ω̇_d = 0) the gyroscopic term is harmless:
    # e_Ω = Ω and Ωᵀ(Ω × JΩ) = 0.
    regulation = {**{Omd[i]: 0 for i in range(3)}, **{dOmd[i]: 0 for i in range(3)}}
    check_pd_regulation = _is_zero(residual_pd.subs(regulation))

    # ── Ψ at a rotation by π about a unit axis n: R = 2nnᵀ − I ──────────────
    n = sp.Matrix(sp.symbols("n_1:4", real=True))
    R_pi = 2 * n * n.T - sp.eye(3)
    Psi_pi = Psi.subs({Re[i, j]: R_pi[i, j] for i in range(3) for j in range(3)})
    check_Psi_pi = _is_zero(Psi_pi - (G.trace() - (n.T * G * n)[0, 0]))
    # ⇒ for ‖n‖ = 1: Ψ(π-rotation) = tr G − nᵀGn ≥ tr G − max g = min_{i≠j}(g_i + g_j)

    # ── S = G R symmetric ⇒ S² = G²  (uses only R Rᵀ = I) ───────────────────
    # S Sᵀ = G R Rᵀ G = G², and S = Sᵀ ⇒ S² = S Sᵀ.
    check_S2 = _is_zero((G * M) * (G * M).T - n2**2 * G * G)

    # ── linearisation at (I, 0): R_e ≈ I + θ̂,  e_R ≈ ½(tr G·I − G) θ,  θ̇ ≈ e_Ω ──
    th = sp.Matrix(sp.symbols(r"\theta_1:4", real=True))
    e_R_lin = sp.Rational(1, 2) * vee(G * hat(th) + hat(th) * G)
    stiffness = sp.Rational(1, 2) * (G.trace() * sp.eye(3) - G)
    check_lin = _is_zero(e_R_lin - stiffness * th)
    s = sp.Symbol("s")
    char_polys = [sp.Poly(Ji * s**2 + kWi * s + stiffness[i, i], s)
                  for i, (Ji, kWi) in enumerate(zip((J1, J2, J3), kW))]
    check_hurwitz = all(all(c.is_positive for c in p.all_coeffs()) for p in char_polys)

    return {
        "e_R": e_R, "Psi": Psi, "e_W": e_W,
        "Psi_dot": e_R.dot(e_W),
        "tau_full": tau_full, "tau_pd": tau_pd,
        "closed_loop": -e_R - KW * e_W,
        "V2": V2,
        "V2_dot_full": -sum(ki * sp.Symbol(rf"e_{{\Omega{i}}}")**2 for i, ki in enumerate(kW, 1)),
        "residual_pd": residual_pd,
        "V2_dot_pd_counterexample": V2_dot_pd_counterexample,
        "stiffness": stiffness,
        "level_bound": "min_{i≠j}(g_i + g_j)",
        "char_polys": [p.as_expr() for p in char_polys],
        "symbols": {"G": G, "KW": KW, "J": J, "Om": Om, "Omd": Omd, "dOmd": dOmd, "Re": Re},
        "checks": {
            "SO3_conjugation_identity": check_conj and check_orth,
            "Psi_dot_equals_eR_dot_eW": check_Psi_dot,
            "closed_loop_J_eW_dot": check_closed_loop,
            "V2_dot_equals_minus_eW_KW_eW": check_V2_dot,
            "pd_V2_dot_positive_somewhere": bool(V2_dot_pd_counterexample > 0),
            "pd_harmless_in_regulation": check_pd_regulation,
            "Psi_at_pi_rotation_is_trG_minus_nGn": check_Psi_pi,
            "GR_symmetric_implies_S2_equals_G2": check_S2,
            "linearisation_stiffness": check_lin,
            "linearisation_hurwitz": check_hurwitz,
        },
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Position loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_position_certificate() -> dict:
    """
    Translational error dynamics under
        a_des = k_v(ṗ_d − K e_p − ṗ) + p̈_d,   F_des = m(a_des + g e₃) = T R_d e₃,
    with thrust T = ‖F_des‖ applied along the actual body axis R e₃.
    """
    m, g = sp.symbols("m g", positive=True)
    k = sp.symbols("k_x k_y k_z", positive=True)
    kvs = sp.symbols("k_{vx} k_{vy} k_{vz}", positive=True)
    K, kv = sp.diag(*k), sp.diag(*kvs)
    T = sp.Symbol("T", positive=True)
    e_p  = sp.Matrix(sp.symbols("e_{p1:4}", real=True))
    e_v  = sp.Matrix(sp.symbols("e_{v1:4}", real=True))
    pdd  = sp.Matrix(sp.symbols(r"\ddot{p}_{d1:4}", real=True))
    Rd   = _generic_matrix("R_d")
    Re   = _generic_matrix("r")
    e3   = sp.Matrix([0, 0, 1])

    a_des = -kv * K * e_p - kv * e_v + pdd                         # = K_v(ṗ_d − K e_p − ṗ) + p̈_d
    # R_d is built so that its third column is F_des / T
    Rd = Rd.copy()
    Rd[:, 2] = m * (a_des + g * e3) / T
    acc   = (T / m) * Rd * Re * e3 - g * e3                         # v̇ = (T/m) R e₃ − g e₃,  R = R_d R_e
    Delta = (T / m) * Rd * (Re - sp.eye(3)) * e3
    e_v_dot = acc - pdd
    check_error_dyn = _is_zero(e_v_dot - (-kv * K * e_p - kv * e_v + Delta))

    # ‖(R_e − I)e₃‖² = 2 − 2 r₃₃  (R_e orthogonal)  and  4Ψ − that = 4 − 2r₁₁ − 2r₂₂ ≥ 0
    Psi = sp.Rational(1, 2) * (sp.eye(3) - Re).trace()
    gap = sp.expand(4 * Psi - (2 - 2 * Re[2, 2]))

    # ── per-axis linear system  ẋ_i = A_i x_i,  x_i = (e_p,i, e_v,i) ─────────
    P_list, checks_P = [], []
    for ki, kvi in zip(k, kvs):
        A = sp.Matrix([[0, 1], [-kvi * ki, -kvi]])
        p11, p12, p22 = sp.symbols("p_11 p_12 p_22")
        P = sp.Matrix([[p11, p12], [p12, p22]])
        sol = sp.solve(list(A.T * P + P * A + sp.eye(2)), [p11, p12, p22], dict=True)[0]
        P = sp.simplify(P.subs(sol))
        checks_P.append(bool(sp.simplify(P[0, 0]).is_positive and sp.factor(P.det()).is_positive))
        P_list.append(P)

    return {
        "a_des": a_des,
        "Delta": Delta,
        "e_v_dot": -kv * K * e_p - kv * e_v + Delta,
        "Psi_gap": gap,
        "P": P_list,
        "char_poly": [sp.Symbol("s")**2 + kvi * sp.Symbol("s") + kvi * ki for ki, kvi in zip(k, kvs)],
        "checks": {
            "error_dynamics": check_error_dyn,
            "Delta_bound_gap_is_4_minus_2r11_minus_2r22": _is_zero(gap - (4 - 2 * Re[0, 0] - 2 * Re[1, 1])),
            "P_positive_definite_all_axes": all(checks_P),
        },
    }


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Full cascade: linearisation along the reference
# ──────────────────────────────────────────────────────────────────────────────

def derive_cascade_linearization() -> dict:
    """
    State (x, ξ) with x = (e_p, e_v) ∈ ℝ⁶ and ξ = (θ, e_Ω) ∈ ℝ⁶, R_e = exp(θ̂).
    Exact closed loop:  ẋ = A x + Δ(t, x, R_e),   (Ṙ_e, ė_Ω) = f(R_e, e_Ω).
    Linearising Δ = (T/m) R_d (R_e − I) e₃ at θ = 0 gives
        Δ ≈ (T/m) R_d (θ̂ e₃) = −(T/m) R_d ê₃ θ  =: B(t) θ,
    so the Jacobian is block upper-triangular  [[A, [B(t) 0]], [0, A_ξ]].
    A and A_ξ are constant and Hurwitz; B(t) is bounded because along the
    reference T = m‖p̈_d + g e₃‖ ≥ m g > 0 is bounded.  Hence the
    linearisation is uniformly exponentially stable and, by Khalil Thm 4.15,
    the origin of the nonlinear error system is locally exponentially stable.
    """
    m, T = sp.symbols("m T", positive=True)
    th = sp.Matrix(sp.symbols(r"\theta_1:4", real=True))
    Rd = _generic_matrix("R_d")
    e3 = sp.Matrix([0, 0, 1])
    Re_first_order = sp.eye(3) + hat(th)
    Delta_lin = (T / m) * Rd * (Re_first_order - sp.eye(3)) * e3
    B = Delta_lin.jacobian(th)
    check_B = _is_zero(B - (-(T / m) * Rd * hat(e3)))
    return {
        "Delta_lin": Delta_lin,
        "B": B,
        "checks": {"B_equals_minus_T_over_m_Rd_hat_e3": check_B},
    }


def run_all_checks() -> dict:
    """All symbolic checks, keyed by '<section>.<check>'."""
    out = {}
    for name, fn in [("attitude", derive_attitude_certificate),
                     ("position", derive_position_certificate),
                     ("cascade", derive_cascade_linearization)]:
        for k, v in fn()["checks"].items():
            out[f"{name}.{k}"] = v
    return out
```

### `src/symbolic/quadcopter_path_following.py`

```python
"""
src/symbolic/quadcopter_path_following.py

Path following on the circle p_d(γ) = [R cos γ, R sin γ, z_d] with the
virtual-target law used by src/control/path_following_controller.py:

    γ̇ = v_d + k_γ e_s,     e_s = t̂(γ)ᵀ e_p   (along-track error),
    v_cmd = (∂p_d/∂γ) γ̇ − K e_p.

What this shows
---------------
Under the kinematic model ṗ = v_cmd the error obeys ė_p = −K e_p for ANY
choice of γ̇: the γ-law does not enter the error dynamics.  Hence
    e_p(t) = e^{−Kt} e_p(0)   and   γ̇(t) → v_d.
Consequence: comparing ‖e_p(t)‖ between path following and trajectory
tracking measures only e_p(0), i.e. where the reference starts; it says
nothing about the merit of the γ-law.  The same holds in the full
dynamic model, because the cascade controller sees p_d(γ(t)) as an
ordinary reference (its exact second derivative is supplied as
feedforward), so the position/attitude result of
quadcopter_certified_lyapunov.py applies unchanged.
"""

from __future__ import annotations
import sympy as sp


def derive_path_following() -> dict:
    R, v_d, k_gamma = sp.symbols(r"R v_d k_\gamma", positive=True)
    gamma = sp.Symbol(r"\gamma", real=True)
    k = sp.symbols("k_x k_y k_z", positive=True)
    K = sp.diag(*k)
    e_p = sp.Matrix(sp.symbols("e_x e_y e_z", real=True))
    gamma_dot = sp.Symbol(r"\dot\gamma", real=True)        # arbitrary, then specialised

    p_d   = sp.Matrix([R * sp.cos(gamma), R * sp.sin(gamma), 0])
    dp_d  = p_d.diff(gamma)
    t_hat = sp.simplify(dp_d / R)
    n_hat = sp.Matrix([-sp.cos(gamma), -sp.sin(gamma), 0])

    v_cmd   = dp_d * gamma_dot - K * e_p
    e_p_dot = sp.simplify(v_cmd - dp_d * gamma_dot)          # ṗ − ṗ_d with ṗ = v_cmd

    V     = sp.Rational(1, 2) * e_p.dot(e_p)
    V_dot = sp.expand(e_p.dot(e_p_dot))

    e_s = t_hat.dot(e_p)
    gamma_law = v_d + k_gamma * e_s

    return {
        "p_d": p_d, "dp_d": dp_d, "t_hat": t_hat, "n_hat": n_hat,
        "e_s": e_s, "e_n": n_hat.dot(e_p),
        "gamma_law": gamma_law,
        "v_cmd": v_cmd.subs(gamma_dot, gamma_law),
        "e_p_dot": e_p_dot,
        "V": V, "V_dot": V_dot,
        "gamma_dot_at_zero_error": gamma_law.subs({s: 0 for s in e_p}),
        "checks": {
            "error_dynamics_independent_of_gamma_dot": e_p_dot.equals(-K * e_p) and not e_p_dot.has(gamma_dot),
            "V_dot_is_minus_eT_K_e": sp.simplify(V_dot + (e_p.T * K * e_p)[0, 0]) == 0,
        },
    }
```

### `tests/conftest.py`

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

### `tests/test_closed_loop.py`

```python
"""End-to-end checks on the MuJoCo closed loop with the LMI gains."""
import numpy as np
import pytest

from src.control.cascade_controller import vehicle_parameters
from src.design.lmi_gains import design_cascade_gains
from src.experiments import load, load_model, simulate_quad


@pytest.fixture(scope="module")
def cfg():
    return load("quadcopter")


@pytest.fixture(scope="module")
def gains(cfg):
    return design_cascade_gains(cfg, load_model(cfg))["controller"]


@pytest.fixture(scope="module")
def run(cfg, gains):
    return simulate_quad(cfg, gains, t_end=30.0)


def test_vehicle_parameters_from_model(cfg):
    p = vehicle_parameters(load_model(cfg))
    assert np.isclose(p["mass"], 0.8) and np.allclose(p["J"], [0.0035, 0.0035, 0.006])
    assert np.allclose(p["A"][0], 1.0)                       # every motor thrusts along body z


def test_mission_converges(run):
    assert run["e_norm"].iloc[-1] < 1e-6


def test_motor_forces_within_limits(run, cfg):
    f = run[["f1", "f2", "f3", "f4"]].to_numpy()
    f_max = load_model(cfg).actuator_ctrlrange[:, 1].min()
    assert f.min() >= 0.0 and f.max() <= f_max


def test_attitude_lyapunov_function_never_increases(run):
    assert np.diff(run["V2"].to_numpy()[2:]).max() < 1e-9   # Ω_d needs two samples


def test_desired_rate_matches_steady_circle(run, cfg):
    # steady circle: R_d(t) = Rz(ωt) R_d(0) ⇒ ‖Ω_d‖ = ω, and the backward difference is exact
    Wd = run[["Wdx", "Wdy", "Wdz"]].to_numpy()[-1]
    assert abs(np.linalg.norm(Wd) - cfg["trajectory"]["omega"]) < 1e-6


def test_pd_law_leaves_residual_error(cfg, gains):
    pd = simulate_quad(cfg, gains, feedforward=False, t_end=30.0)
    assert pd["e_norm"].iloc[-1] > 1e-4
```

### `tests/test_lmi.py`

```python
"""LMI certificates re-checked numerically, independently of the SDP solver."""
import numpy as np
import pytest

from src.design.lmi_gains import DOUBLE_INTEGRATOR_A, design_cascade_gains, fsf_input_bounded
from src.experiments import load, load_model


def test_double_integrator_certificate():
    B = np.array([[0.0], [1.0]])
    X0 = [np.array([s1 * 2.0, s2 * 1.0]) for s1 in (-1, 1) for s2 in (-1, 1)]
    d = fsf_input_bounded(DOUBLE_INTEGRATOR_A, B, [3.0], X0, alpha_max=10.0)
    P, Acl = np.linalg.inv(d.W), DOUBLE_INTEGRATOR_A + B @ d.K
    # rate α:  Acl W + W Aclᵀ + 2α W ⪯ 0
    assert np.linalg.eigvalsh(Acl @ d.W + d.W @ Acl.T + 2 * d.alpha * d.W).max() <= 1e-6
    # every design point lies in E = {xᵀPx ≤ 1}
    assert all(x0 @ P @ x0 <= 1 + 1e-4 for x0 in X0)
    # max over E of |K x| = √(K W Kᵀ) ≤ ū
    assert np.sqrt(d.K @ d.W @ d.K.T).item() <= 3.0 * (1 + 1e-4)


@pytest.fixture(scope="module")
def design():
    cfg = load("quadcopter")
    return design_cascade_gains(cfg, load_model(cfg))


def test_cascade_design(design):
    assert all(design["position"][a]["kv"] > 0 and design["position"][a]["k"] > 0 for a in "xyz")
    att = design["attitude"]
    assert np.all(att["G"] > 0) and np.all(att["kW"] > 0)
    assert np.allclose(0.5 * (att["G"].sum() - att["G"]), att["kappa"])   # κ_j = ½(tr G − g_j)
    # inner loop faster than outer loop
    assert att["alpha"].min() > max(design["position"][a]["alpha"] for a in "xyz")
```

### `tests/test_symbolic.py`

```python
"""Every identity used in the stability analysis must simplify to zero."""
import pytest

from src.symbolic.quadcopter_certified_lyapunov import run_all_checks
from src.symbolic.lyapunov import derive_circular_path_following
from src.symbolic.quadcopter_path_following import derive_path_following

CHECKS = run_all_checks()


@pytest.mark.parametrize("name", sorted(CHECKS))
def test_quadrotor_identity(name):
    assert CHECKS[name], name


@pytest.mark.parametrize("name,ok", sorted(derive_circular_path_following()["checks"].items()))
def test_unicycle_identity(name, ok):
    assert ok, name


@pytest.mark.parametrize("name,ok", sorted(derive_path_following()["checks"].items()))
def test_path_following_identity(name, ok):
    assert ok, name
```

## Summary

Included **23** readable text files (2435 source lines).

Omitted files:
- unrecognized/generated file type: 19
