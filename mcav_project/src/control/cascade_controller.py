"""
src/control/cascade_controller.py

Cascade geometric controller for a quadrotor — Lee et al. (2010).

Notation follows the paper:
  e_p   : position error  (world frame)
  e_ψ   : yaw error
  e_R   : attitude error  (SO3, body frame)
  e_Ω   : angular velocity error (body frame)
  R     : rotation matrix body → world
  Ω     : angular velocity in body frame
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple


# ──────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """MuJoCo quaternion [qw, qx, qy, qz] → rotation matrix R (body → world)."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [  2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qw*qx)],
        [  2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
    ])


def _quat_to_yaw(q: np.ndarray) -> float:
    qw, qx, qy, qz = q
    return float(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))


def _vee(S: np.ndarray) -> np.ndarray:
    """Vee map: skew-symmetric matrix → axial vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def _wrap(angle: float) -> float:
    """Wrap angle to (−π, π]."""
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def _safe_normalize(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else fallback


# ──────────────────────────────────────────────────────────────────────────────
# Data containers
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class QuadState:
    """Full vehicle state as read from MuJoCo."""
    pos:    np.ndarray   # p  ∈ ℝ³   world frame
    quat:   np.ndarray   # [qw, qx, qy, qz]
    vel:    np.ndarray   # ṗ  ∈ ℝ³   world frame
    angvel: np.ndarray   # Ω  ∈ ℝ³   body frame


@dataclass
class QuadReference:
    """
    Desired trajectory point at time t.

    accel is the feedforward term ẍ_d(t).
    Defaults to zero → pure feedback.
    For a circular trajectory: accel = [−Rω²cos(ωt), −Rω²sin(ωt), 0].
    """
    pos:      np.ndarray                                               # p_d
    vel:      np.ndarray                                               # ṗ_d
    yaw:      float                                                    # ψ_d
    yaw_rate: float                                                    # ψ̇_d
    accel:    np.ndarray = field(default_factory=lambda: np.zeros(3)) # ẍ_d


# ──────────────────────────────────────────────────────────────────────────────
# Cascade controller
# ──────────────────────────────────────────────────────────────────────────────

class CascadeController:
    """
    Hierarchical cascade controller.

    Outer loop  : Lyapunov position  → desired velocity  v_cmd
    Middle loop : velocity P-control → desired acceleration  a_des
    Thrust      : a_des + g·e₃  → thrust vector F_des, magnitude T
    Attitude    : geometric SO(3) tracking  → torques τ   (Lee eq. 15)
    Allocation  : [T, τ]  → individual motor forces  f₁…f₄

    Override hooks
    --------------
    _compute_a_des()  : replace the middle loop (e.g. lambdified symbolic law)
    _compute_torque() : replace the torque law  (e.g. add gyroscopic term)
    """

    def __init__(self, config: dict):
        self.m          = float(config["mass"])
        self.g          = 9.81
        d               = float(config["arm_d"])    # L / √2
        km              = float(config["km"])
        self.max_thrust = float(config.get("max_thrust_per_motor", 4.0))

        ctrl = config.get("controller", {})
        self.kx   = float(ctrl.get("kx",   1.0))
        self.ky   = float(ctrl.get("ky",   1.0))
        self.kz   = float(ctrl.get("kz",   1.0))
        self.kpsi = float(ctrl.get("kpsi", 1.0))
        self.kv   = float(ctrl.get("kv",   4.0))
        self.kR   = float(ctrl.get("kR",   8.0))
        self.kw   = float(ctrl.get("kw",   2.5))

        # Allocation matrix A  s.t.  [T, τx, τy, τz]ᵀ = A · [f₁ f₂ f₃ f₄]ᵀ
        # Derived symbolically in quadcopter_dynamic_control.py.
        # Motor order: FR(1), FL(2), BR(3), BL(4)
        self._A = np.array([
            [ 1.0,  1.0,  1.0,  1.0],
            [  -d,   +d,   -d,   +d],
            [  -d,   -d,   +d,   +d],
            [ -km,  +km,  +km,  -km],
        ])
        self._Ainv = np.linalg.inv(self._A)

        self.saturate = bool(config.get("controller", {}).get("saturate", False))

    @classmethod
    def from_config(cls, cfg: dict) -> "CascadeController":
        return cls(cfg)

    # ── override points ───────────────────────────────────────────────────────

    def _compute_a_des(
        self,
        v_cmd: np.ndarray,
        state: QuadState,
        ref:   QuadReference,
    ) -> np.ndarray:
        """
        Middle loop: velocity tracking → desired acceleration (world frame).
            a_des = kv · (v_cmd − ṗ) + ẍ_d
        Override to use a lambdified symbolic law.
        """
        return self.kv * (v_cmd - state.vel) + ref.accel

    def _compute_torque(
        self,
        e_R:   np.ndarray,
        e_Ω:   np.ndarray,
        state: QuadState,
    ) -> Tuple[np.ndarray, dict]:
        """
        Inner loop: attitude tracking → control torques (body frame).
            τ = −kR e_R − kΩ e_Ω             (Lee et al. eq. 15)

        Returns (tau, extras) where extras is a dict of additional quantities
        to merge into the SimLogger log.  Override both to add terms
        (e.g. gyroscopic cancellation) and expose them in the DataFrame.
        """
        tau = -self.kR * e_R - self.kw * e_Ω
        return tau, {}

    # ── main entry point ──────────────────────────────────────────────────────

    def compute(
        self,
        state: QuadState,
        ref:   QuadReference,
    ) -> Tuple[np.ndarray, dict]:
        """
        Compute motor forces from current state and reference.

        Returns
        -------
        forces : np.ndarray (4,)  clipped to [0, max_thrust_per_motor]
        log    : dict  all intermediate quantities for SimLogger
        """

        # ── Outer loop: Lyapunov position (GAS) ──────────────────────────
        # V = ½‖e_p‖²  →  V̇ = −e_pᵀ K e_p < 0
        e_p      = state.pos - ref.pos
        e_ψ      = _wrap(_quat_to_yaw(state.quat) - ref.yaw)
        v_cmd    = ref.vel    - np.array([self.kx*e_p[0], self.ky*e_p[1], self.kz*e_p[2]])
        ψ_dot_cmd = ref.yaw_rate - self.kpsi * e_ψ

        # ── Middle loop: velocity → desired acceleration ──────────────────
        a_des = self._compute_a_des(v_cmd, state, ref)

        # ── Thrust vector (world frame) ───────────────────────────────────
        # F_des = m (a_des + g e₃)
        F_des = self.m * (a_des + np.array([0.0, 0.0, self.g]))
        T     = max(float(np.linalg.norm(F_des)), 0.1 * self.m * self.g)

        # ── Desired attitude (Lee et al. geometric construction) ──────────
        # z_b,d = F_des / T
        # x_c   = [cos ψ_d, sin ψ_d, 0]ᵀ
        # y_b,d = (z_b,d × x_c) / ‖z_b,d × x_c‖
        # x_b,d = y_b,d × z_b,d
        z_d = F_des / T
        x_c = np.array([np.cos(ref.yaw), np.sin(ref.yaw), 0.0])
        y_d = _safe_normalize(np.cross(z_d, x_c), np.array([0.0, 1.0, 0.0]))
        R_d = np.column_stack([np.cross(y_d, z_d), y_d, z_d])

        # ── Attitude error (Lee et al. eq. 10) ────────────────────────────
        # e_R = ½ vee(R_dᵀ R − Rᵀ R_d)
        R   = _quat_to_rot(state.quat)
        R_e = R_d.T @ R
        e_R = 0.5 * _vee(R_e - R_e.T)

        Ω_d = np.array([0.0, 0.0, ψ_dot_cmd])
        e_Ω = state.angvel - R.T @ R_d @ Ω_d

        # ── Torque law (overridable) ──────────────────────────────────────
        tau, torque_extras = self._compute_torque(e_R, e_Ω, state)

        # ── Motor allocation ──────────────────────────────────────────────
        forces = self._Ainv @ np.array([T, tau[0], tau[1], tau[2]])
        if self.saturate:
            forces = np.clip(forces, 0.0, self.max_thrust)

        log = {
            "ex": float(e_p[0]),   "ey": float(e_p[1]),   "ez": float(e_p[2]),
            "epsi": float(e_ψ),
            "vx_cmd": float(v_cmd[0]), "vy_cmd": float(v_cmd[1]), "vz_cmd": float(v_cmd[2]),
            "ax_des": float(a_des[0]), "ay_des": float(a_des[1]), "az_des": float(a_des[2]),
            "ax_ff":  float(ref.accel[0]), "ay_ff": float(ref.accel[1]), "az_ff": float(ref.accel[2]),
            "T": T,
            "eRx": float(e_R[0]),  "eRy": float(e_R[1]),  "eRz": float(e_R[2]),
            "tau_x": float(tau[0]), "tau_y": float(tau[1]), "tau_z": float(tau[2]),
            "f1": float(forces[0]), "f2": float(forces[1]),
            "f3": float(forces[2]), "f4": float(forces[3]),
            **torque_extras,   # certified controller adds gyro_x/y/z here
        }

        return forces, log