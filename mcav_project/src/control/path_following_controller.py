"""
src/control/path_following_controller.py

Path-following controller for the quadrotor.

Architecture
------------
                        γ dynamics
   ┌─────────────────────────────────────────────┐
   │   γ̇ = v_d + k_γ t̂(γ)^T e_p              │
   │   e_p = p − p_d(γ)                          │
   └──────────────────┬──────────────────────────┘
                      │  p_d(γ), ∂p_d/∂γ · γ̇, ψ_d(γ), ψ̇_d
                      ▼
           ┌─────────────────────┐
           │  QuadReference      │   v_cmd = ∂p_d/∂γ · γ̇ − K e_p
           │  (to CascadeCtrl)   │   (cascade controller handles inner loop)
           └─────────────────────┘

Why smoother transients
-----------------------
Trajectory tracking  :  p_d(t) moves at fixed speed ω regardless of
                         vehicle position → vehicle must "chase" the
                         reference when starting far away.

Path following       :  γ̇ = v_d + k_γ t̂^T e_p.  When the vehicle is
                         laterally far from the path (e_p mostly normal),
                         t̂^T e_p ≈ 0 and γ̇ ≈ v_d, but the position
                         controller pulls the vehicle to p_d(γ) without
                         the reference running away.  When the vehicle is
                         behind the virtual target (negative along-track
                         error), γ̇ decreases, giving the vehicle time to
                         catch up before progressing along the path.

Usage
-----
path = CirclePath(R=3.0, z_d=3.0)
pfc  = PathFollowingController.from_config(cfg, path)

# in _loop_step:
ref, gamma, gamma_dot = pfc.step(state, dt)
forces, log = cascade_ctrl.compute(state, ref)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from src.control.cascade_controller import QuadReference


# ──────────────────────────────────────────────────────────────────────────────
# Path abstraction
# ──────────────────────────────────────────────────────────────────────────────

class CirclePath:
    """
    Planar circle at constant altitude.

        p_d(γ) = [R cos γ,  R sin γ,  z_d]

    The parameter γ is the angle in radians.
    One full lap corresponds to Δγ = 2π.
    """

    def __init__(self, R: float, z_d: float):
        self.R   = R
        self.z_d = z_d

    def position(self, gamma: float) -> np.ndarray:
        """Virtual target position p_d(γ)."""
        return np.array([self.R * np.cos(gamma),
                         self.R * np.sin(gamma),
                         self.z_d])

    def tangent(self, gamma: float) -> np.ndarray:
        """Unnormalised tangent  ∂p_d/∂γ = [−R sinγ, R cosγ, 0]."""
        return np.array([-self.R * np.sin(gamma),
                          self.R * np.cos(gamma),
                          0.0])

    def unit_tangent(self, gamma: float) -> np.ndarray:
        """Unit tangent  t̂(γ) = [−sinγ, cosγ, 0]."""
        return np.array([-np.sin(gamma), np.cos(gamma), 0.0])

    def yaw(self, gamma: float) -> float:
        """
        Desired yaw: vehicle should face along the path tangent.
        For a circle:  ψ_d = γ + π/2.
        """
        return float(gamma + np.pi / 2.0)

    def yaw_rate(self, gamma_dot: float) -> float:
        """
        ψ̇_d = dψ_d/dt = (dψ_d/dγ) γ̇ = 1 · γ̇   (for circle).
        """
        return float(gamma_dot)

    def closest_gamma(self, p: np.ndarray) -> float:
        """
        Initial γ that minimises ‖p_d(γ) − p‖² in the xy plane.
        For a circle this is the angle of the vehicle projected onto the circle.
        """
        return float(np.arctan2(p[1], p[0]))


# ──────────────────────────────────────────────────────────────────────────────
# Path-following outer loop
# ──────────────────────────────────────────────────────────────────────────────

class PathFollowingController:
    """
    Outer-loop path-following controller.

    Maintains the virtual path parameter γ and produces a QuadReference
    for the CascadeController's inner loop.

    Parameters (from config["path_following"])
    ------------------------------------------
    v_d     : desired path speed [rad/s]
    k_gamma : along-track correction gain [-]
    """

    def __init__(self, path: CirclePath, v_d: float, k_gamma: float):
        self.path    = path
        self.v_d     = v_d
        self.k_gamma = k_gamma
        self._gamma  = 0.0          # virtual path parameter (initialised in reset)

    @classmethod
    def from_config(cls, cfg: dict, path: CirclePath) -> "PathFollowingController":
        pf = cfg.get("path_following", {})
        return cls(
            path    = path,
            v_d     = float(pf.get("v_d",     0.5)),
            k_gamma = float(pf.get("k_gamma", 1.0)),
        )

    def reset(self, initial_pos: np.ndarray) -> None:
        """
        Initialise γ to the closest point on the path to initial_pos.
        This avoids a large initial along-track error that would otherwise
        cause the virtual target to immediately "run away" from the vehicle.
        """
        self._gamma = self.path.closest_gamma(initial_pos)

    @property
    def gamma(self) -> float:
        return self._gamma

    def step(
        self,
        pos:    np.ndarray,
        dt:     float,
    ) -> tuple[QuadReference, float, float]:
        """
        Advance γ by one timestep and return the QuadReference.

        Parameters
        ----------
        pos : current vehicle position (world frame)
        dt  : timestep [s]

        Returns
        -------
        ref        : QuadReference for the cascade controller
        gamma      : current path parameter
        gamma_dot  : current path parameter rate
        """
        gamma = self._gamma

        # ── path quantities at current γ ──────────────────────────────────
        p_d     = self.path.position(gamma)
        dp_d    = self.path.tangent(gamma)          # ∂p_d/∂γ (unnormalised)
        t_hat   = self.path.unit_tangent(gamma)     # normalised tangent

        # ── position error and along-track component ──────────────────────
        e_p = pos - p_d
        e_s = float(t_hat @ e_p)                    # along-track error

        # ── gamma dynamics:  γ̇ = v_d + k_γ e_s ──────────────────────────
        # When e_s > 0 (vehicle ahead of target): advance γ faster.
        # When e_s < 0 (vehicle behind target):   slow γ down.
        gamma_dot = self.v_d + self.k_gamma * e_s

        # ── integrate γ (Euler step; same integrator as MuJoCo) ──────────
        self._gamma = gamma + gamma_dot * dt

        # ── build reference for the cascade controller ────────────────────
        # ref.vel = ∂p_d/∂γ · γ̇  is the feedforward velocity.
        # The cascade controller computes:
        #   v_cmd = ref.vel − K_p (pos − ref.pos)
        #         = dp_d · γ̇  −  K_p e_p
        # which is exactly the path-following velocity law.
        ref = QuadReference(
            pos      = p_d,
            vel      = dp_d * gamma_dot,
            yaw      = self.path.yaw(gamma),
            yaw_rate = self.path.yaw_rate(gamma_dot),
        )

        return ref, gamma, gamma_dot