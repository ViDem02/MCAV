"""
src/symbolic/quadcopter_path_following.py

Symbolic Lyapunov derivation for the path-following controller on a
parametric circle path.

Key difference from trajectory tracking
----------------------------------------
Trajectory tracking:   p_d(t)  — reference moves with physical time
Path following:        p_d(γ)  — reference moves with virtual parameter γ

The virtual parameter evolves as:
    γ̇ = v_d + k_γ ŷ_t(γ)^T e_p

where ŷ_t is the unit tangent to the path.  This lets the virtual target
adapt its speed to the vehicle's position, giving smoother transients when
starting far from the path.
"""

from __future__ import annotations
import sympy as sp


def derive_path_following() -> dict:
    """
    Lyapunov-based path following for a circle p_d(γ) = [R cosγ, R sinγ, z_d].

    State      : position error  e_p = p - p_d(γ)
    Control    : velocity command  v_cmd = (∂p_d/∂γ)γ̇ − K e_p
    Gamma law  : γ̇ = v_d + k_γ t̂(γ)^T e_p
    """
    # ── symbols ──────────────────────────────────────────────────────────────
    R, gamma, v_d   = sp.symbols(r"R \gamma v_d", positive=True)
    k_gamma         = sp.Symbol(r"k_\gamma", positive=True)
    kx, ky, kz      = sp.symbols("k_x k_y k_z", positive=True)
    ex, ey, ez      = sp.symbols("e_x e_y e_z", real=True)

    # ── path geometry ─────────────────────────────────────────────────────────
    # Position on path
    p_d  = sp.Matrix([R * sp.cos(gamma), R * sp.sin(gamma), 0])

    # Unnormalised tangent ∂p_d/∂γ
    dp_d = sp.diff(p_d, gamma)

    # Unit tangent t̂ = ∂p_d/∂γ / |∂p_d/∂γ|
    dp_d_norm = sp.sqrt(dp_d.dot(dp_d))
    t_hat     = sp.simplify(dp_d / dp_d_norm)   # [-sin γ, cos γ, 0]

    # Path yaw  ψ_d(γ) = atan2(dp_d_y, dp_d_x) = γ + π/2   (for circle)
    psi_d     = gamma + sp.pi / 2
    dpsi_d_dg = sp.diff(psi_d, gamma)            # = 1

    # ── error dynamics ────────────────────────────────────────────────────────
    e_p = sp.Matrix([ex, ey, ez])

    # Gamma dynamics
    e_s      = t_hat.dot(e_p)                    # along-path ("tangential") error
    #gamma_dot = v_d + k_gamma * e_s
    gamma_dot = v_d * (1.0 + k_gamma * e_s)

    # Feedforward velocity from the moving virtual target
    v_ff = dp_d * gamma_dot

    # Diagonal gain matrix
    K = sp.diag(kx, ky, kz)

    # Full velocity command
    v_cmd = v_ff - K * e_p

    # Error dynamics  ė_p = v_cmd − ∂p_d/∂γ · γ̇ = −K e_p
    e_p_dot = sp.simplify(v_cmd - dp_d * gamma_dot)

    # ── Lyapunov function ─────────────────────────────────────────────────────
    V = sp.Rational(1, 2) * e_p.dot(e_p)

    # V̇ = ∇V · ė_p = e_p^T (−K e_p)
    V_dot = sp.expand(e_p.dot(e_p_dot))

    # Confirm negative definiteness symbolically
    # V_dot = −kx ex² − ky ey² − kz ez²
    V_dot_expected = -(kx * ex**2 + ky * ey**2 + kz * ez**2)
    assert sp.simplify(V_dot - V_dot_expected) == 0, \
        "V_dot does not match expected form"

    # ── gamma convergence ─────────────────────────────────────────────────────
    # As e_p → 0:  γ̇ → v_d  (desired path speed)
    gamma_dot_at_zero = gamma_dot.subs({ex: 0, ey: 0, ez: 0})

    # ── along-path vs cross-path decomposition ────────────────────────────────
    # Normal to path (inward for circle): n̂ = [−cos γ, −sin γ, 0]
    n_hat = sp.Matrix([-sp.cos(gamma), -sp.sin(gamma), 0])

    # Cross-track error
    e_n = n_hat.dot(e_p)

    # Along-track error
    e_s_sym = t_hat.dot(e_p)

    return {
        # path geometry
        "p_d":          p_d,
        "dp_d":         dp_d,
        "t_hat":        t_hat,
        "n_hat":        n_hat,
        "psi_d":        psi_d,
        "dpsi_d_dg":    dpsi_d_dg,
        # gamma dynamics
        "gamma_dot":    gamma_dot,
        "e_s":          e_s_sym,
        "e_n":          e_n,
        "gamma_dot_at_zero": gamma_dot_at_zero,
        # error dynamics
        "v_ff":         v_ff,
        "v_cmd":        v_cmd,
        "e_p_dot":      e_p_dot,
        # Lyapunov
        "V":            V,
        "V_dot":        V_dot,
        "is_ND":        True,
        "stability":    "GAS",
    }


def compare_trajectory_vs_path_following() -> dict:
    """
    Symbolic comparison: show what changes when switching from trajectory
    tracking to path following.

    In trajectory tracking:
        p_d(t) = [R cos(ω t), R sin(ω t), z_d]
        γ = ω t  (fixed, not adaptive)
        ė_p = −K e_p  +  ZERO perturbation   (if inner loop perfect)

    In path following:
        p_d(γ),  γ̇ = v_d + k_γ t̂^T e_p
        ė_p = −K e_p  (exactly, by construction)

    The difference:  path following adds k_γ t̂^T e_p to γ̇.
    This slows the virtual target when the vehicle is behind it
    and advances it when the vehicle is ahead.
    """
    R, omega, t     = sp.symbols(r"R \omega t", positive=True)
    v_d             = sp.Symbol("v_d", positive=True)
    kx, ky          = sp.symbols("k_x k_y", positive=True)
    k_gamma         = sp.Symbol(r"k_\gamma", positive=True)
    ex, ey          = sp.symbols("e_x e_y", real=True)

    # ── Trajectory tracking ───────────────────────────────────────────────────
    # Reference at fixed time:  γ_TT = ω t
    # v_ref(t) = dγ/dt * dp_d/dγ = ω * [-R sin(ω t), R cos(ω t)]
    gamma_TT     = omega * t
    v_ref_TT     = sp.Matrix([
        -R * omega * sp.sin(gamma_TT),
         R * omega * sp.cos(gamma_TT),
    ])

    # ── Path following ────────────────────────────────────────────────────────
    gamma_sym    = sp.Symbol(r"\gamma")
    t_hat_sym    = sp.Matrix([-sp.sin(gamma_sym), sp.cos(gamma_sym)])
    e_p_sym      = sp.Matrix([ex, ey])

    gamma_dot_PF = v_d + k_gamma * t_hat_sym.dot(e_p_sym)
    v_ref_PF     = sp.Matrix([
        -R * sp.sin(gamma_sym),
         R * sp.cos(gamma_sym),
    ]) * gamma_dot_PF

    # ── Key difference ────────────────────────────────────────────────────────
    # γ̇_PF − γ̇_TT = k_γ t̂^T e_p
    # This adapts the virtual target speed based on vehicle position.
    gamma_dot_diff = sp.simplify(gamma_dot_PF - v_d)   # = k_γ t̂^T e_p

    return {
        "gamma_TT":        gamma_TT,
        "v_ref_TT":        v_ref_TT,
        "gamma_dot_PF":    gamma_dot_PF,
        "v_ref_PF":        v_ref_PF,
        "gamma_dot_diff":  gamma_dot_diff,
    }