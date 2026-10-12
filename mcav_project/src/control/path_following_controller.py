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
