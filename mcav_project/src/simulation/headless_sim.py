# src/simulation/headless_sim.py
from .base_sim import BaseSim

class HeadlessSim(BaseSim):
    """Pure physics, no rendering. Fast — use for optimization and batch runs."""
    pass   # BaseSim is already headless by default