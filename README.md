# MCAV — Actuator-aware geometric flight control for a quadrotor

A 0.8 kg quadrotor, simulated in MuJoCo, takes off and tracks a circular trajectory with a
geometric controller on SO(3). Gains are synthesised with LMIs so that every motor stays inside
its 0–8 N range, and the stability analysis is checked by SymPy instead of being asserted by hand.

| Result | Value |
|---|---|
| Steady-state tracking error | ~1e-9 m (machine precision) |
| Motor forces, nominal mission | 1.7–2.7 N, inside the 0–8 N range (gains from the literature: −36…46 N) |
| Monte Carlo, 500 random initial positions and attitudes up to 175° | ~96 % converge, every failure is a ground contact |
| Symbolic identities verified | 21 |
| Full reproduction | one command, about 2 minutes on a laptop |

The technical report is in [`report/`](report/) — `report.pdf`: project overview; `changes.pdf`: what changed and why, for readers with one control course; `changes_technical.pdf`: the same with full proofs and algorithms.

## Quickstart

Requires [uv](https://docs.astral.sh/uv/) (it installs Python and every dependency).

```bash
cd mcav_project
uv run python main.py          # every number and figure of the report → results/  (~2 min)
uv run python main.py --quick  # smaller Monte Carlo (~30 s)
uv run pytest                  # 29 tests: symbolic proofs, LMI certificates, closed loop
uv run jupyter lab             # notebooks/ — final.ipynb is the guided walkthrough
```

Build the report after `main.py` has run: `cd report && latexmk -pdf report.tex changes.tex changes_technical.tex`.

## Layout

```
mcav_project/
├── main.py                        reproduces every result (symbolic checks → LMI → simulation → Monte Carlo)
├── config/                        mission, gains, design budgets (YAML)
├── models/quadcopter/quadcopter.xml   MuJoCo model — single source of mass, inertia, motor geometry and limits
├── src/
│   ├── control/                   geometric controller, path-following reference generator
│   ├── design/lmi_gains.py        input-bounded LMI state-feedback synthesis (cvxpy)
│   ├── symbolic/                  stability analysis, every identity verified by SymPy
│   └── experiments.py             headless simulations and Monte Carlo (MuJoCo, joblib)
├── notebooks/                     walkthrough, path following, unicycle
└── tests/                         pytest suite
```

## Built on

MuJoCo (physics, allocation matrix from the model), SymPy (symbolic verification),
cvxpy + Clarabel (LMIs), SciPy (rotations, ODE integration), joblib (parallel Monte Carlo),
pandas, matplotlib, pytest.
