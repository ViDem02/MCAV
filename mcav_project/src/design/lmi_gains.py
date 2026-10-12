"""
src/design/lmi_gains.py

Actuator-aware gain synthesis with LMIs (state feedback, input-bounded).

Port of fsf_input_bounded.m from the tank-LMI project (same notation):
for ẋ = A x + B u, u = K x, find W = Wᵀ ≻ 0, X with K = X W⁻¹ such that

    He(A W + B X) ⪯ −2α W                     (E = {xᵀW⁻¹x ≤ 1} invariant, rate α)
    [ 1   x0ᵀ ; x0  W ] ⪰ 0   for every x0    (design envelope inside E)
    [ ū_i²  X_i ; X_iᵀ  W ] ⪰ 0   for each i  (|K_i x| ≤ ū_i on all of E)

and maximise α by bisection (α enters bilinearly through α W).
Every trajectory of the linear model that starts in E stays in E, decays at
rate α and never asks for more than ū.  No saturation is used.

Applied to the two reduced models of the cascade, axis by axis:

  position   ė_p = e_v, ė_v = u          u = a_des − p̈_d (exact when R = R_d)
  attitude   θ̇ = e_Ω,   J_j ė_Ω = u      u = PD part of the torque (linearised
                                            error dynamics after the exact
                                            gyroscopic / feedforward cancellation)

The input budgets come from the motor limits 0 ≤ f_i ≤ f_max, split between
collective thrust and torques (see motor_budgets()).
"""

from __future__ import annotations

from dataclasses import dataclass

import cvxpy as cp
import numpy as np

DOUBLE_INTEGRATOR_A = np.array([[0.0, 1.0], [0.0, 0.0]])


@dataclass
class LMIDesign:
    K: np.ndarray        # u = K x
    W: np.ndarray        # E = {x : xᵀ W⁻¹ x ≤ 1}
    alpha: float         # certified decay rate of V = xᵀW⁻¹x  (‖x‖ decays at least as e^{−α t})
    ubar: float


def _solve_alpha(A, B, ubar, X0, alpha, eps=1e-6):
    n, m = B.shape
    W = cp.Variable((n, n), symmetric=True)
    X = cp.Variable((m, n))
    M = A @ W + B @ X
    cons = [W >> eps * np.eye(n), M + M.T << -2 * alpha * W]
    for x0 in X0:
        x0 = np.asarray(x0, float).reshape(n, 1)
        cons.append(cp.bmat([[np.ones((1, 1)), x0.T], [x0, W]]) >> 0)
    for i in range(m):
        cons.append(cp.bmat([[np.array([[ubar[i] ** 2]]), X[i:i + 1, :]], [X[i:i + 1, :].T, W]]) >> 0)
    prob = cp.Problem(cp.Minimize(0), cons)
    try:
        prob.solve(solver=cp.CLARABEL)
    except cp.SolverError:
        return False, None, None
    ok = prob.status == cp.OPTIMAL
    return ok, (W.value if ok else None), (X.value if ok else None)


def fsf_input_bounded(A, B, ubar, X0, alpha_max: float = 50.0, tol: float = 1e-3) -> LMIDesign:
    """Largest certified rate α ≤ alpha_max with |u_i| ≤ ū_i on an invariant ellipsoid containing X0."""
    A = np.asarray(A, float); B = np.asarray(B, float).reshape(A.shape[0], -1)
    ubar = np.atleast_1d(np.asarray(ubar, float))
    lo, hi = 0.0, 1e-2
    while hi < alpha_max and _solve_alpha(A, B, ubar, X0, hi)[0]:
        lo, hi = hi, min(2 * hi, alpha_max)
    if hi >= alpha_max and _solve_alpha(A, B, ubar, X0, alpha_max)[0]:
        lo = hi = alpha_max
    while hi - lo > tol * max(hi, 1.0):
        mid = 0.5 * (lo + hi)
        if _solve_alpha(A, B, ubar, X0, mid)[0]:
            lo = mid
        else:
            hi = mid
    ok, W, X = _solve_alpha(A, B, ubar, X0, lo)
    if not ok or lo <= 0:
        raise RuntimeError("No positive-rate design satisfies the input budget on the requested envelope.")
    return LMIDesign(K=X @ np.linalg.inv(W), W=W, alpha=lo, ubar=float(ubar[0]))


def _box_vertices(a: float, b: float) -> list[np.ndarray]:
    return [np.array([sa * a, sb * b]) for sa in (-1, 1) for sb in (-1, 1)]


# ──────────────────────────────────────────────────────────────────────────────
# Budgets from the motor limits
# ──────────────────────────────────────────────────────────────────────────────

def motor_budgets(cfg: dict, vehicle: dict) -> dict:
    """
    f_i = T/4 + δ_i,  |δ_i| ≤ |τx|/(4d) + |τy|/(4d) + |τz|/(4k_m)  (from A⁻¹).
    Reserving σ newtons per motor for torques and keeping T/4 ∈ [σ, f_max − σ]
    guarantees 0 ≤ f_i ≤ f_max.  σ is split between the three torque axes.

    Collective thrust with u = a_des − p̈_d and |u_x|,|u_y| ≤ ū_xy, |u_z| ≤ ū_z:
        T/m ≥ g − ū_z,     T/m ≤ √((a_h + √2 ū_xy)² + (g + ū_z)²),
    a_h = max horizontal ‖p̈_d‖ (= R ω² on the circle).
    """
    m, g, A = vehicle["mass"], 9.81, vehicle["A"]
    arm = np.abs(A[1:, :]).max(axis=1)                      # [d, d, k_m] from the allocation matrix
    f_max = float(vehicle["f_range"][:, 1].min())
    des = cfg["design"]
    sigma = des["torque_reserve_per_motor"]
    tau_max = 4 * sigma * np.asarray(des["torque_split"], float) * arm

    ubar_z = g - 4 * sigma / m                              # keeps T/4 ≥ σ
    ubar_xy = des["accel_budget_xy"]
    traj = cfg["trajectory"]
    a_h = traj["radius"] * traj["omega"] ** 2
    T_max = m * np.hypot(a_h + np.sqrt(2) * ubar_xy, g + ubar_z)
    if ubar_z <= 0 or T_max / 4 > f_max - sigma:
        raise ValueError("Thrust budget infeasible: lower accel_budget_xy or torque_reserve_per_motor.")
    return {"sigma": sigma, "tau_max": tau_max, "ubar_xy": ubar_xy, "ubar_z": ubar_z,
            "T_max_per_motor": T_max / 4, "f_max": f_max}


# ──────────────────────────────────────────────────────────────────────────────
# Gain synthesis for the cascade
# ──────────────────────────────────────────────────────────────────────────────

def design_position(cfg: dict, b: dict, e_p0: np.ndarray, e_v0: np.ndarray) -> dict:
    """Per axis: u = K x = −k_v k_i e_p − k_v e_v  ⇒  k_v = −K₂,  k_i = K₁/K₂."""
    env = cfg["design"]
    out = {}
    for i, ax in enumerate("xyz"):
        ubar = b["ubar_z"] if ax == "z" else b["ubar_xy"]
        X0 = _box_vertices(max(abs(e_p0[i]), env["min_position_envelope"]),
                           max(abs(e_v0[i]), env["min_velocity_envelope"]))
        des = fsf_input_bounded(DOUBLE_INTEGRATOR_A, np.array([[0.0], [1.0]]), [ubar], X0,
                                alpha_max=env["alpha_max_position"])
        K1, K2 = des.K.ravel()
        out[ax] = {"kv": -K2, "k": K1 / K2, "alpha": des.alpha, "ubar": ubar, "W": des.W}
    return out


def design_attitude(cfg: dict, b: dict, J: np.ndarray, theta0: np.ndarray) -> dict:
    """
    Per body axis j: u_j = −κ_j θ_j − k_Ω,j e_Ω,j.  The diagonal stiffness κ is
    realised on SO(3) by the weighted error e_R = ½ vee(G R_e − R_eᵀG) with
    g_j = Σκ − 2κ_j (linearisation e_R ≈ ½(tr G·I − G) θ = diag(κ) θ).
    """
    env = cfg["design"]
    kappa, kW, alphas, Ws = np.zeros(3), np.zeros(3), np.zeros(3), []
    # roll and pitch share one envelope (J_x = J_y, equal budgets) so that κ_x = κ_y,
    # which together with κ_z < 2κ_x makes every g_j positive
    th_env = np.abs(theta0).copy()
    th_env[:2] = th_env[:2].max()
    for j in range(3):
        ubar = env["pd_torque_fraction"] * b["tau_max"][j]
        X0 = _box_vertices(max(th_env[j], np.radians(env["min_attitude_envelope_deg"])),
                           env["min_rate_envelope"])
        des = fsf_input_bounded(DOUBLE_INTEGRATOR_A, np.array([[0.0], [1.0 / J[j]]]), [ubar], X0,
                                alpha_max=env["alpha_max_attitude"])
        kappa[j], kW[j] = -des.K[0, 0], -des.K[0, 1]
        alphas[j] = des.alpha
        Ws.append(des.W)
    G = kappa.sum() - 2 * kappa
    if np.any(G <= 0):
        raise ValueError(f"Stiffness {kappa} not realisable by a positive G (need κ_j < Σκ/2).")
    return {"kappa": kappa, "G": G, "kW": kW, "alpha": alphas, "W": Ws,
            "tau_pd_max": env["pd_torque_fraction"] * b["tau_max"]}


def design_cascade_gains(cfg: dict, model) -> dict:
    """
    Position gains from the mission start, then attitude gains from the initial
    attitude error that those position gains produce.  Returns the controller
    gains plus the certificates (rates, ellipsoids, budgets).
    """
    from scipy.spatial.transform import Rotation
    from src.control.cascade_controller import G_ACC, E3, circle_reference, desired_attitude, vehicle_parameters

    vehicle = vehicle_parameters(model)
    b = motor_budgets(cfg, vehicle)
    ic, traj = cfg["initial_state"], cfg["trajectory"]
    ref0 = circle_reference(0.0, traj["radius"], traj["omega"], traj["z_height"])
    e_p0, e_v0 = np.asarray(ic["pos"], float) - ref0.pos, -ref0.vel
    pos = design_position(cfg, b, e_p0, e_v0)

    kv = np.array([pos[a]["kv"] for a in "xyz"])
    k = np.array([pos[a]["k"] for a in "xyz"])
    R_d0 = desired_attitude(-kv * k * e_p0 - kv * e_v0 + ref0.accel + G_ACC * E3, ref0.yaw)
    R0 = Rotation.from_euler("z", ic["yaw"]).as_matrix()
    theta0 = Rotation.from_matrix(R_d0.T @ R0).as_rotvec()

    att = design_attitude(cfg, b, vehicle["J"], theta0)
    return {
        "controller": {"kx": float(k[0]), "ky": float(k[1]), "kz": float(k[2]),
                       "kv": kv.tolist(), "G": att["G"].tolist(), "kw": att["kW"].tolist()},
        "position": pos, "attitude": att, "theta0": theta0, "budgets": b,
    }
