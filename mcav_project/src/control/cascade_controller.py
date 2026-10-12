"""
src/control/cascade_controller.py

Geometric cascade controller for a quadrotor — Lee, Leok, McClamroch (2010),
with the weighted attitude error of Lee (2012) so that every axis can have
its own gain.

  e_p, e_v : position / velocity error           (world frame)
  R_e      : R_dᵀR,  attitude error rotation
  e_R      : ½ vee(G R_e − R_eᵀG),  G = diag(g) ≻ 0   (body frame)
  Ψ        : ½ tr(G (I − R_e))
  e_Ω      : Ω − R_eᵀΩ_d                           (body frame)

  a_des = K_v(ṗ_d − K e_p − ṗ) + p̈_d            PD on position + feedforward
  F_des = m(a_des + g e₃),  T = ‖F_des‖,  b₃d = F_des / T
  R_d   = [b₂d × b₃d, b₂d, b₃d],  b₂d ∥ b₃d × [cos ψ_d, sin ψ_d, 0]
  τ     = −e_R − K_Ω e_Ω  [+ Ω×JΩ − J(Ω̂ R_eᵀΩ_d − R_eᵀΩ̇_d)   if feedforward]

Only with the bracketed terms is the attitude error system autonomous, which
the stability analysis (src/symbolic/quadcopter_certified_lyapunov.py)
requires.  Ω_d and Ω̇_d come from backward differences of R_d at the control
rate (exact on the steady circle, O(dt) in transients).

Mass, inertia, allocation and motor limits are read from the MuJoCo model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

G_ACC = 9.81
E3 = np.array([0.0, 0.0, 1.0])


def vee(S: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix → axial vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


@dataclass
class QuadReference:
    pos:   np.ndarray                                           # p_d
    vel:   np.ndarray                                           # ṗ_d
    yaw:   float                                                # ψ_d
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # p̈_d


def circle_reference(t: float, radius: float, omega: float, z_d: float) -> QuadReference:
    """p_d(t) = [R cos ωt, R sin ωt, z_d] with exact derivatives, yaw tangent to the circle."""
    c, s = np.cos(omega * t), np.sin(omega * t)
    return QuadReference(
        pos   = np.array([radius * c, radius * s, z_d]),
        vel   = np.array([-radius * omega * s, radius * omega * c, 0.0]),
        yaw   = omega * t + np.pi / 2.0,
        accel = np.array([-radius * omega**2 * c, -radius * omega**2 * s, 0.0]),
    )


def desired_attitude(F_des: np.ndarray, yaw: float) -> np.ndarray:
    """R_d with third axis along F_des and first axis as close as possible to heading yaw."""
    z_d = F_des / np.linalg.norm(F_des)
    y_d = np.cross(z_d, [np.cos(yaw), np.sin(yaw), 0.0])
    y_d /= np.linalg.norm(y_d)
    return np.column_stack([np.cross(y_d, z_d), y_d, z_d])


def vehicle_parameters(model: mujoco.MjModel, body: str = "quad") -> dict:
    """Mass, principal inertia, allocation matrix A ([T, τ] = A f) and motor range, from the MJCF."""
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    if not np.allclose(model.body_iquat[b], [1.0, 0.0, 0.0, 0.0]):
        raise ValueError("Principal inertia axes must coincide with the body frame.")
    data = mujoco.MjData(model)                      # qpos0: level pose, so body = world axes
    mujoco.mj_forward(model, data)
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz,
                            data.moment_rowadr, data.moment_colind)
    return {
        "mass": float(model.body_subtreemass[b]),
        "J": model.body_inertia[b].copy(),
        "A": moment[:, 2:6].T,                       # free-joint dofs: [Fz, τx, τy, τz]
        "f_range": model.actuator_ctrlrange.copy(),
    }


class GeometricController:
    """
    gains: kx, ky, kz, kv (scalar or per axis), G or kR (scalar ⇒ G = kR·I),
           kw (scalar or per axis), saturate.
    feedforward=False gives the PD-only law (no Ω×JΩ, no desired-frame terms).
    """

    def __init__(self, model: mujoco.MjModel, gains: dict, feedforward: bool = True):
        p = vehicle_parameters(model)
        self.m, self.J, self.f_range = p["mass"], p["J"], p["f_range"]
        self.A_inv = np.linalg.inv(p["A"])
        self.dt = model.opt.timestep
        self.feedforward = feedforward

        per_axis = lambda v: np.broadcast_to(np.asarray(v, float), (3,)).copy()
        self.K  = np.array([gains["kx"], gains["ky"], gains["kz"]], float)
        self.kv = per_axis(gains["kv"])
        self.G  = per_axis(gains["G"] if "G" in gains else gains["kR"])
        self.kw = per_axis(gains["kw"])
        self.saturate = bool(gains.get("saturate", False))
        self._R_d_prev = self._W_d_prev = None

    def _desired_rates(self, R_d):
        """Ω_d = log(R_d,prevᵀ R_d)/dt,  Ω̇_d = (Ω_d − Ω_d,prev)/dt  (zero until history exists)."""
        W_d, dW_d = np.zeros(3), np.zeros(3)
        if self._R_d_prev is not None:
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (self._R_d_prev.T @ R_d).ravel())
            mujoco.mju_quat2Vel(W_d, q, self.dt)
            if self._W_d_prev is not None:
                dW_d = (W_d - self._W_d_prev) / self.dt
            self._W_d_prev = W_d
        self._R_d_prev = R_d
        return W_d, dW_d

    def compute(self, data: mujoco.MjData, ref: QuadReference) -> tuple[np.ndarray, dict]:
        """Motor forces for the current MuJoCo state.  Call exactly once per period dt."""
        pos, vel, W = data.qpos[:3], data.qvel[:3], data.qvel[3:6]   # Ω in body frame
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, data.qpos[3:7])                          # current state (xmat lags one step)
        R = R.reshape(3, 3)

        e_p, e_v = pos - ref.pos, vel - ref.vel
        a_des = -self.kv * self.K * e_p - self.kv * e_v + ref.accel
        F_des = self.m * (a_des + G_ACC * E3)
        T = float(np.linalg.norm(F_des))
        R_d = desired_attitude(F_des, ref.yaw)
        W_d, dW_d = self._desired_rates(R_d)

        R_e = R_d.T @ R
        Gm  = np.diag(self.G)
        e_R = 0.5 * vee(Gm @ R_e - R_e.T @ Gm)
        e_W = W - R_e.T @ W_d
        tau = -e_R - self.kw * e_W
        if self.feedforward:
            tau += np.cross(W, self.J * W) - self.J * (np.cross(W, R_e.T @ W_d) - R_e.T @ dW_d)

        forces = self.A_inv @ np.r_[T, tau]
        if self.saturate:
            forces = np.clip(forces, self.f_range[:, 0], self.f_range[:, 1])

        Psi = 0.5 * float(np.trace(Gm @ (np.eye(3) - R_e)))
        log = {
            "ex": e_p[0], "ey": e_p[1], "ez": e_p[2],
            "T": T, "Psi": Psi,
            "V2": Psi + 0.5 * float(e_W @ (self.J * e_W)),
            "Wdx": W_d[0], "Wdy": W_d[1], "Wdz": W_d[2],
            "tau_x": tau[0], "tau_y": tau[1], "tau_z": tau[2],
            "f1": forces[0], "f2": forces[1], "f3": forces[2], "f4": forces[3],
        }
        return forces, log
