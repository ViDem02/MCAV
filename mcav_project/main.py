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
