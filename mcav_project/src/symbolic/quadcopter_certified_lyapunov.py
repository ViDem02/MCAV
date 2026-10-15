"""
src/symbolic/quadcopter_certified_lyapunov.py

Symbolic stability analysis of the geometric cascade controller
(src/control/cascade_controller.py, feedforward=True) on the full rigid-body model

    ṗ = v,      m v̇ = T R e₃ - m g e₃,
    Ṙ = R Ω̂,    J Ω̇ = τ - Ω x JΩ.

Every identity below is computed by SymPy from these equations and the
control law; nothing is asserted by hand.  Each function returns the
expressions to display plus `checks`, a dict of booleans that are True
only if the corresponding identity simplified to zero.

What is proved (and what is not)
--------------------------------
1. derive_attitude_certificate():
     with the full Lee et al. torque law (weighted error, diagonal gains
     G, K_Ω) the attitude error obeys
     J ė_Ω = -e_R - K_Ω e_Ω,  Ṙ_e = R_e ê_Ω  (autonomous), and
     V₂ = Ψ + ½ e_Ωᵀ J e_Ω  has  V̇₂ = -e_Ωᵀ K_Ω e_Ω  exactly.
     ⇒ (R_e, e_Ω) = (I, 0) is asymptotically stable and every initial
       condition with V₂ < min_{i≠j}(g_i + g_j) converges to it (LaSalle on
       a compact sublevel set that excludes all 180° equilibria).  Locally
       the convergence is exponential.  It is NOT globally asymptotically
       stable: no continuous feedback on SO(3) is (Bhat & Bernstein 2000).
     Without the feedforward terms V̇₂ is sign-indefinite (explicit
     counterexample returned).
2. derive_position_certificate():
     the position/velocity error obeys ė_p = e_v,
     ė_v = -K_v K e_p - K_v e_v + Δ  with  Δ = (T/m)(R - R_d)e₃,
     ‖Δ‖ ≤ (2T/m)√Ψ₁,  Ψ₁ = ½ tr(I - R_e) ≤ Ψ / min g.  For Δ = 0 the system is linear, Hurwitz, and
     V₁ = xᵀPx (P from the Lyapunov equation) proves global exponential
     stability of THAT reduced system.
3. derive_cascade_linearization():
     the closed loop in error coordinates is a cascade
     (attitude → position).  Its linearization along the reference is
     block-triangular with Hurwitz diagonal blocks and a bounded coupling,
     hence uniformly exponentially stable; by Lyapunov's indirect method
     (Khalil, Thm 4.15) the full nonlinear closed loop is LOCALLY
     exponentially stable.  The size of its region of attraction is not
     characterised analytically.

Assumptions shared by all results: unbounded, signed motor forces (no
saturation); F_des ≠ 0 and F_des ∦ x_c so that R_d is defined; exact
Ω_d = vee(R_dᵀṘ_d) and Ω̇_d.  The implementation approximates the last
two by backward differences at the control rate (exact on the
steady-state circle, O(dt) during transients).
"""

from __future__ import annotations
from typing import Any

import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _el(M: sp.MatrixBase, i: int, j: int = 0) -> Any:
    """Entry M[i, j]; typed Any because Pylance's bundled SymPy stubs reject arithmetic on Expr."""
    return M[i, j]


def hat(w: sp.Matrix) -> sp.Matrix:
    """ŵ such that ŵ v = w x v."""
    wx, wy, wz = (_el(w, i) for i in range(3))
    return sp.Matrix([
        [  0, -wz,  wy],
        [ wz,   0, -wx],
        [-wy,  wx,   0],
    ])


def vee(S: sp.Matrix) -> sp.Matrix:
    return sp.Matrix([S[2, 1], S[0, 2], S[1, 0]])


def _is_zero(expr) -> bool:
    """True iff expr (scalar or matrix) simplifies to exactly zero."""
    entries = [e for row in expr.tolist() for e in row] if isinstance(expr, sp.MatrixBase) else [expr]
    return all(sp.expand(e) == 0 or sp.simplify(e) == 0 for e in entries)


def _generic_matrix(name: str) -> sp.Matrix:
    """3×3 matrix of independent real symbols (identities proved for it hold for any R_e)."""
    return sp.Matrix(3, 3, lambda i, j: sp.Symbol(f"{name}_{{{i+1}{j+1}}}", real=True))


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Attitude loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_attitude_certificate() -> dict:
    """
    Exact Lyapunov analysis of the attitude error (R_e, e_Ω), R_e = R_dᵀR, with
    the weighted error of the controller:

        Ψ = ½ tr(G (I - R_e)),   e_R = ½ vee(G R_e - R_eᵀ G),   G = diag(g) ≻ 0,
        τ = -e_R - K_Ω e_Ω + Ω × JΩ - J (Ω̂ R_eᵀ Ω_d - R_eᵀ Ω̇_d),  K_Ω = diag ≻ 0.

    Kinematics.  From Ṙ = RΩ̂, Ṙ_d = R_dΩ̂_d and R ŵ Rᵀ = (Rw)^ on SO(3):
        Ṙ_e = R_e ê_Ω,   e_Ω = Ω - R_eᵀΩ_d.
    The identity R ŵ Rᵀ = (Rw)^ is verified on the Euler–Rodrigues
    parametrisation; the remaining identities are verified for a generic
    3×3 matrix R_e, so they need no small-angle approximation.

    Critical points.  e_R = 0 ⇔ S = G R_e is symmetric ⇒ S² = S Sᵀ = G², which
    forces R_e = I or a rotation by π about some axis n (case analysis on the
    eigenvalues of G).  At such a rotation Ψ = tr G - nᵀG n ≥ min_{i≠j}(g_i + g_j).
    Hence on {V₂ < min_{i≠j}(g_i + g_j)} the only point with e_R = 0 is R_e = I.
    """
    # ── symbols ──────────────────────────────────────────────────────────────
    g = sp.symbols("g_1:4", positive=True)
    kW = list(sp.symbols(r"k_{\Omega1} k_{\Omega2} k_{\Omega3}", positive=True))
    G, KW = sp.diag(*g), sp.diag(*kW)
    J1, J2, J3 = sp.symbols("J_1 J_2 J_3", positive=True)
    J = sp.diag(J1, J2, J3)
    Om   = sp.Matrix(sp.symbols(r"\Omega_1:4", real=True))
    Omd  = sp.Matrix(sp.symbols(r"\Omega_{d1} \Omega_{d2} \Omega_{d3}", real=True))
    dOmd = sp.Matrix(sp.symbols(r"\dot\Omega_{d1} \dot\Omega_{d2} \dot\Omega_{d3}", real=True))
    Re   = _generic_matrix("r")

    # ── SO(3) identity R ŵ Rᵀ = (Rw)^  (Euler–Rodrigues, homogeneous form) ───
    q0, q1, q2, q3 = sp.symbols("q_0:4", real=True)
    w = sp.Matrix(sp.symbols("w_1:4", real=True))
    M = sp.Matrix([
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3),         2*(q1*q3+q0*q2)],
        [2*(q1*q2+q0*q3),         q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2),         2*(q2*q3+q0*q1),         q0**2-q1**2-q2**2+q3**2],
    ])                                            # R = M / |q|²,  M Mᵀ = |q|⁴ I
    n2 = q0**2 + q1**2 + q2**2 + q3**2
    check_conj = _is_zero(M * hat(w) * M.T - n2 * hat(M * w))
    check_orth = _is_zero(M * M.T - n2**2 * sp.eye(3))

    # ── error quantities ────────────────────────────────────────────────────
    e_R  = vee(G * Re - Re.T * G) / 2
    Psi  = sp.Rational(1, 2) * (G * (sp.eye(3) - Re)).trace()
    e_W  = Om - Re.T * Omd

    # ── Ψ̇ = e_R · e_Ω   (from Ṙ_e = R_e ê_Ω) ────────────────────────────────
    Re_dot  = Re * hat(e_W)
    Psi_dot = -sp.Rational(1, 2) * (G * Re_dot).trace()
    check_Psi_dot = _is_zero(Psi_dot - e_R.dot(e_W))

    # ── ė_Ω  from Euler's equation and Ṙ_eᵀ = -ê_Ω R_eᵀ ─────────────────────
    def e_W_dot(tau):
        Om_dot = J.inv() * (tau - Om.cross(J * Om))
        return Om_dot - (Re_dot.T * Omd + Re.T * dOmd)

    tau_full = (-e_R - KW * e_W + Om.cross(J * Om)
                - J * (hat(Om) * Re.T * Omd - Re.T * dOmd))
    tau_pd   = -e_R - KW * e_W

    check_closed_loop = _is_zero(J * e_W_dot(tau_full) - (-e_R - KW * e_W))

    # ── Lyapunov function and its derivative (J constant and symmetric) ─────
    V2 = Psi + sp.Rational(1, 2) * (e_W.T * J * e_W)[0, 0]

    def V2_dot(tau):
        return Psi_dot + (e_W.T * J * e_W_dot(tau))[0, 0]

    check_V2_dot = _is_zero(V2_dot(tau_full) + (e_W.T * KW * e_W)[0, 0])

    # ── PD law only: residual and an explicit point where V̇₂ > 0 ───────────
    V2_dot_pd = sp.expand(V2_dot(tau_pd))
    residual_pd = sp.expand(V2_dot_pd + (e_W.T * KW * e_W)[0, 0])
    # R_e = I, Ω_d = 0, Ω = e_Ω = (0.1,0,0), Ω̇_d = (-1000,0,0):
    # residual = -e_Ωᵀ J Ω̇_d = 0.35 > k_Ω1‖e_Ω‖² = 0.025
    numeric_point = {**{_el(Re, i, j): int(i == j) for i in range(3) for j in range(3)},
                     _el(Om, 0): sp.Rational(1, 10), _el(Om, 1): 0, _el(Om, 2): 0,
                     **{_el(Omd, i): 0 for i in range(3)},
                     _el(dOmd, 0): -1000, _el(dOmd, 1): 0, _el(dOmd, 2): 0,
                     J1: sp.Rational(35, 10000), J2: sp.Rational(35, 10000), J3: sp.Rational(6, 1000),
                     **{gi: sp.Rational(1, 2) for gi in g}, **{ki: sp.Rational(5, 2) for ki in kW}}
    V2_dot_pd_counterexample = V2_dot_pd.subs(numeric_point)

    # In set-point regulation (Ω_d = Ω̇_d = 0) the gyroscopic term is harmless:
    # e_Ω = Ω and Ωᵀ(Ω × JΩ) = 0.
    regulation = {**{Omd[i]: 0 for i in range(3)}, **{dOmd[i]: 0 for i in range(3)}}
    check_pd_regulation = _is_zero(residual_pd.subs(regulation))

    # ── Ψ at a rotation by π about a unit axis n: R = 2nnᵀ - I ──────────────
    n = sp.Matrix(sp.symbols("n_1:4", real=True))
    R_pi = 2 * n * n.T - sp.eye(3)
    Psi_pi = Psi.subs({Re[i, j]: R_pi[i, j] for i in range(3) for j in range(3)})
    check_Psi_pi = _is_zero(Psi_pi - (G.trace() - (n.T * G * n)[0, 0]))
    # ⇒ for ‖n‖ = 1: Ψ(π-rotation) = tr G - nᵀGn ≥ tr G - max g = min_{i≠j}(g_i + g_j)

    # ── S = G R symmetric ⇒ S² = G²  (uses only R Rᵀ = I) ───────────────────
    # S Sᵀ = G R Rᵀ G = G², and S = Sᵀ ⇒ S² = S Sᵀ.
    check_S2 = _is_zero((G * M) * (G * M).T - n2**2 * G * G)

    # ── linearisation at (I, 0): R_e ≈ I + θ̂,  e_R ≈ ½(tr G·I - G) θ,  θ̇ ≈ e_Ω ──
    th = sp.Matrix(sp.symbols(r"\theta_1:4", real=True))
    e_R_lin = sp.Rational(1, 2) * vee(G * hat(th) + hat(th) * G)
    stiffness = sp.Rational(1, 2) * (G.trace() * sp.eye(3) - G)
    check_lin = _is_zero(e_R_lin - stiffness * th)
    s = sp.Symbol("s")
    char_polys = [sp.Poly(Ji * s**2 + kWi * s + stiffness[i, i], s)
                  for i, (Ji, kWi) in enumerate(zip((J1, J2, J3), kW))]
    check_hurwitz = all(all(c.is_positive for c in p.all_coeffs()) for p in char_polys)

    return {
        "e_R": e_R, "Psi": Psi, "e_W": e_W,
        "Psi_dot": e_R.dot(e_W),
        "tau_full": tau_full, "tau_pd": tau_pd,
        "closed_loop": -e_R - KW * e_W,
        "V2": V2,
        "V2_dot_full": -sum(ki * sp.Symbol(rf"e_{{\Omega{i}}}")**2 for i, ki in enumerate(kW, 1)),
        "residual_pd": residual_pd,
        "V2_dot_pd_counterexample": V2_dot_pd_counterexample,
        "stiffness": stiffness,
        "level_bound": "min_{i≠j}(g_i + g_j)",
        "char_polys": [p.as_expr() for p in char_polys],
        "symbols": {"G": G, "KW": KW, "J": J, "Om": Om, "Omd": Omd, "dOmd": dOmd, "Re": Re},
        "checks": {
            "SO3_conjugation_identity": check_conj and check_orth,
            "Psi_dot_equals_eR_dot_eW": check_Psi_dot,
            "closed_loop_J_eW_dot": check_closed_loop,
            "V2_dot_equals_minus_eW_KW_eW": check_V2_dot,
            "pd_V2_dot_positive_somewhere": bool(V2_dot_pd_counterexample > 0),
            "pd_harmless_in_regulation": check_pd_regulation,
            "Psi_at_pi_rotation_is_trG_minus_nGn": check_Psi_pi,
            "GR_symmetric_implies_S2_equals_G2": check_S2,
            "linearisation_stiffness": check_lin,
            "linearisation_hurwitz": check_hurwitz,
        },
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Position loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_position_certificate() -> dict:
    """
    Translational error dynamics under
        a_des = k_v(ṗ_d - K e_p - ṗ) + p̈_d,   F_des = m(a_des + g e₃) = T R_d e₃,
    with thrust T = ‖F_des‖ applied along the actual body axis R e₃.
    """
    m, g = sp.symbols("m g", positive=True)
    k = sp.symbols("k_x k_y k_z", positive=True)
    kvs = sp.symbols("k_{vx} k_{vy} k_{vz}", positive=True)
    K, kv = sp.diag(*k), sp.diag(*kvs)
    T = sp.Symbol("T", positive=True)
    e_p  = sp.Matrix(sp.symbols("e_{p1:4}", real=True))
    e_v  = sp.Matrix(sp.symbols("e_{v1:4}", real=True))
    pdd  = sp.Matrix(sp.symbols(r"\ddot{p}_{d1:4}", real=True))
    Rd   = _generic_matrix("R_d")
    Re   = _generic_matrix("r")
    e3   = sp.Matrix([0, 0, 1])

    a_des = -kv * K * e_p - kv * e_v + pdd                         # = K_v(ṗ_d - K e_p - ṗ) + p̈_d
    # R_d is built so that its third column is F_des / T
    Rd = Rd.copy()
    Rd[:, 2] = m * (a_des + g * e3) / T
    acc   = (T / m) * Rd * Re * e3 - g * e3                         # v̇ = (T/m) R e₃ - g e₃,  R = R_d R_e
    Delta = (T / m) * Rd * (Re - sp.eye(3)) * e3
    e_v_dot = acc - pdd
    check_error_dyn = _is_zero(e_v_dot - (-kv * K * e_p - kv * e_v + Delta))

    # ‖(R_e - I)e₃‖² = 2 - 2 r₃₃  (R_e orthogonal)  and  4Ψ - that = 4 - 2r₁₁ - 2r₂₂ ≥ 0
    Psi = sp.Rational(1, 2) * (sp.eye(3) - Re).trace()
    gap = sp.expand(4 * Psi - (2 - 2 * _el(Re, 2, 2)))

    # ── per-axis linear system  ẋ_i = A_i x_i,  x_i = (e_p,i, e_v,i) ─────────
    P_list, checks_P = [], []
    for ki, kvi in zip(k, kvs):
        A = sp.Matrix([[0, 1], [-kvi * ki, -kvi]])
        p11, p12, p22 = sp.symbols("p_11 p_12 p_22")
        P = sp.Matrix([[p11, p12], [p12, p22]])
        sol = sp.solve(list(A.T * P + P * A + sp.eye(2)), [p11, p12, p22], dict=True)[0]
        P = sp.simplify(P.subs(sol))
        checks_P.append(bool(sp.simplify(P[0, 0]).is_positive and sp.factor(P.det()).is_positive))
        P_list.append(P)

    return {
        "a_des": a_des,
        "Delta": Delta,
        "e_v_dot": -kv * K * e_p - kv * e_v + Delta,
        "Psi_gap": gap,
        "P": P_list,
        "char_poly": [sp.Symbol("s")**2 + kvi * sp.Symbol("s") + kvi * ki for ki, kvi in zip(k, kvs)],
        "checks": {
            "error_dynamics": check_error_dyn,
            "Delta_bound_gap_is_4_minus_2r11_minus_2r22": _is_zero(gap - (4 - 2 * _el(Re, 0, 0) - 2 * _el(Re, 1, 1))),
            "P_positive_definite_all_axes": all(checks_P),
        },
    }


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Full cascade: linearisation along the reference
# ──────────────────────────────────────────────────────────────────────────────

def derive_cascade_linearization() -> dict:
    """
    State (x, ξ) with x = (e_p, e_v) ∈ ℝ⁶ and ξ = (θ, e_Ω) ∈ ℝ⁶, R_e = exp(θ̂).
    Exact closed loop:  ẋ = A x + Δ(t, x, R_e),   (Ṙ_e, ė_Ω) = f(R_e, e_Ω).
    Linearising Δ = (T/m) R_d (R_e - I) e₃ at θ = 0 gives
        Δ ≈ (T/m) R_d (θ̂ e₃) = -(T/m) R_d ê₃ θ  =: B(t) θ,
    so the Jacobian is block upper-triangular  [[A, [B(t) 0]], [0, A_ξ]].
    A and A_ξ are constant and Hurwitz; B(t) is bounded because along the
    reference T = m‖p̈_d + g e₃‖ ≥ m g > 0 is bounded.  Hence the
    linearisation is uniformly exponentially stable and, by Khalil Thm 4.15,
    the origin of the nonlinear error system is locally exponentially stable.
    """
    m, T = sp.symbols("m T", positive=True)
    th = sp.Matrix(sp.symbols(r"\theta_1:4", real=True))
    Rd = _generic_matrix("R_d")
    e3 = sp.Matrix([0, 0, 1])
    Re_first_order = sp.eye(3) + hat(th)
    Delta_lin = (T / m) * Rd * (Re_first_order - sp.eye(3)) * e3
    B = Delta_lin.jacobian(th)
    check_B = _is_zero(B - (-(T / m) * Rd * hat(e3)))
    return {
        "Delta_lin": Delta_lin,
        "B": B,
        "checks": {"B_equals_minus_T_over_m_Rd_hat_e3": check_B},
    }


def run_all_checks() -> dict:
    """All symbolic checks, keyed by '<section>.<check>'."""
    out = {}
    for name, fn in [("attitude", derive_attitude_certificate),
                     ("position", derive_position_certificate),
                     ("cascade", derive_cascade_linearization)]:
        for k, v in fn()["checks"].items():
            out[f"{name}.{k}"] = v
    return out
