Type stubs for Pylance / pyright (`stubPath`, set in the repo-root `pyrightconfig.json`).

- `mujoco/` — generated: `uv run pybind11-stubgen mujoco -o typings`

SymPy is not stubbed here: Pylance's bundled SymPy stubs take precedence over `stubPath` and reject
arithmetic on `Expr` (e.g. `2 * expr`). Code in `src/symbolic/` keeps symbolic entries untyped (`Any`)
where that matters, see `_el` in `quadcopter_certified_lyapunov.py`.
