from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from joblib import Parallel, delayed
from tqdm.auto import tqdm


@dataclass(frozen=True)
class MonteCarloROASettings:
    n_samples: int = 10_000
    max_radius: float = 3.0
    n_jobs: int = -1
    t_max: float = 4.0
    initial_altitude_error: float = 0.0
    ground_clearance: float = 0.1
    success_threshold: float = 0.05
    seed: int | None = None
    show_progress: bool = True

    @classmethod
    def from_config(cls, config: dict) -> "MonteCarloROASettings":
        multithreading = config.get("multithreading", {})
        return cls(
            n_samples=int(multithreading.get("n_samples", cls.n_samples)),
            max_radius=float(multithreading.get("max_radius", cls.max_radius)),
            n_jobs=int(multithreading.get("n_jobs", cls.n_jobs)),
            t_max=float(multithreading.get("t_max", cls.t_max)),
            initial_altitude_error=float(
                multithreading.get("initial_altitude_error", cls.initial_altitude_error)
            ),
            ground_clearance=float(
                multithreading.get("ground_clearance", cls.ground_clearance)
            ),
            success_threshold=float(
                multithreading.get("success_threshold", cls.success_threshold)
            ),
            seed=multithreading.get("seed", cls.seed),
            show_progress=bool(multithreading.get("show_progress", cls.show_progress)),
        )


def _sample_initial_positions(settings: MonteCarloROASettings) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(settings.seed)
    radii = np.sqrt(rng.uniform(0.0, settings.max_radius**2, settings.n_samples))
    angles = rng.uniform(0.0, 2.0 * np.pi, settings.n_samples)
    return radii * np.cos(angles), radii * np.sin(angles)


def _run_single_rollout(simulator_cls, config_path: str, ex: float, ey: float, settings: MonteCarloROASettings):
    simulator = simulator_cls(config_path)
    success = simulator.run_rollout(
        ex0=ex,
        ey0=ey,
        ez0=settings.initial_altitude_error,
        T_max=settings.t_max,
        ground_clearance=settings.ground_clearance,
        success_threshold=settings.success_threshold,
    )
    return ex, ey, success


def run_parallel_roa_rollouts(
    simulator_cls,
    config_path: str,
    config: dict,
    settings: MonteCarloROASettings | None = None,
    progress_desc: str = "Running headless rollouts",
) -> dict:
    """Execute the empirical ROA sampling loop in parallel."""
    settings = settings or MonteCarloROASettings.from_config(config)
    ex_samples, ey_samples = _sample_initial_positions(settings)
    params = list(zip(ex_samples, ey_samples))

    results = Parallel(n_jobs=settings.n_jobs)(
        delayed(_run_single_rollout)(simulator_cls, config_path, ex, ey, settings)
        for ex, ey in tqdm(
            params,
            total=len(params),
            desc=progress_desc,
            disable=not settings.show_progress,
        )
    )

    ex_res, ey_res, success_res = zip(*results)
    return {
        "ex": np.asarray(ex_res),
        "ey": np.asarray(ey_res),
        "success": np.asarray(success_res, dtype=bool),
        "settings": settings,
    }
