# MCAV — Quickstart

Very brief instructions to activate the environment and run the notebooks.

Activation (preferred via `uv`):

- Start Jupyter Lab inside the project's environment without manual activation:

  `cd mcav_project && uv run jupyter lab`

Alternative: activate the created virtualenv manually:

1. `cd mcav_project`
2. `source .venv/bin/activate`
3. `jupyter lab` (or `jupyter notebook`)

Running the notebooks:

- Open Jupyter Lab/Notebook in your browser and navigate to the `notebooks/` folder.
- Open the notebook you want to run (for example, `final.ipynb`).

Notes:

- Prefer using `uv` for creating and running commands in the project environment.
- If you need to install packages, use `uv add <package>` inside the project folder.
