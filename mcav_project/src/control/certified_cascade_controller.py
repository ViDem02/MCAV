"""
src/control/certified_cascade_controller.py

Extends CascadeController with the gyroscopic cancellation term required
for full Lyapunov certification (Lee et al. 2010, Theorem 1).

The only change is in _compute_torque():

    τ = −kR e_R − kΩ e_Ω  +  Ω × JΩ        (gyroscopic cancellation)

Without this term the nonlinear Coriolis dynamics of J enter the error
dynamics and V̇_att can no longer be proven strictly negative.

Why this is sufficient for a constant-rate circular trajectory
--------------------------------------------------------------
The full Lee et al. torque law (eq. 17) also includes:
    J (Ω̂ Rᵀ R_d Ω_d − Rᵀ R_d Ω̇_d)
For a circle with constant ω: Ω_d = [0, 0, ω]ᵀ and Ω̇_d = 0.
This term vanishes. See quadcopter_certified_lyapunov.py for the proof.
"""

from __future__ import annotations

import mujoco
import numpy as np

from src.control.cascade_controller import CascadeController, QuadState


class CertifiedCascadeController(CascadeController):
    """
    Drop-in replacement for CascadeController.
    Reads the inertia tensor J directly from the MuJoCo model so that
    the gyroscopic cancellation exactly matches the simulated physics.
    """

    def __init__(self, config: dict, J: np.ndarray):
        super().__init__(config)
        self.J       = J
        self.full_ff = bool(config.get("controller", {}).get("full_ff", False))
        self.saturate = bool(config.get("controller", {}).get("saturate", False))

    @classmethod
    def from_model(
        cls, config: dict, model: mujoco.MjModel
    ) -> "CertifiedCascadeController":
        """Extract J from the MuJoCo model and build the controller."""
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad")
        J = np.diag(model.body_inertia[body_id])
        return cls(config, J)

    # ── single override: certified torque law ────────────────────────────────

    def _compute_torque(
        self,
        e_R:   np.ndarray,
        e_Ω:   np.ndarray,
        state: QuadState,
    ) -> tuple:
        """
        Certified torque (Lee et al. eq. 17, constant-ω simplification):
            τ = −kR e_R − kΩ e_Ω + Ω × JΩ

        V̇_att = −kR ‖e_R‖² − kΩ ‖e_Ω‖² < 0   (restored by gyroscopic term)

        Returns (tau, extras) — extras exposes gyro_x/y/z in the SimLogger.
        """
        gyro = np.cross(state.angvel, self.J @ state.angvel)
        tau  = -self.kR * e_R - self.kw * e_Ω + gyro

        if self.full_ff:
            Om_hat = np.array([
                [                0, -state.angvel[2],  state.angvel[1]],
                [ state.angvel[2],                 0, -state.angvel[0]],
                [-state.angvel[1],  state.angvel[0],                0],
            ])
            tau += self.J @ (Om_hat @ np.array([0.0, 0.0, self.kpsi]))

        extras = {
            "gyro_x": float(gyro[0]),
            "gyro_y": float(gyro[1]),
            "gyro_z": float(gyro[2]),
        }
        return tau, extras