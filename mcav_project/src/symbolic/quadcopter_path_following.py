"""
src/symbolic/quadcopter_path_following.py

Path following on the circle p_d(γ) = [R cos γ, R sin γ, z_d] with the
virtual-target law used by src/control/path_following_controller.py:

    γ̇ = v_d + k_γ e_s,     e_s = t̂(γ)ᵀ e_p   (along-track error),
    v_cmd = (∂p_d/∂γ) γ̇ − K e_p.

What this shows
---------------
Under the kinematic model ṗ = v_cmd the error obeys ė_p = −K e_p for ANY
choice of γ̇: the γ-law does not enter the error dynamics.  Hence
    e_p(t) = e^{−Kt} e_p(0)   and   γ̇(t) → v_d.
Consequence: comparing ‖e_p(t)‖ between path following and trajectory
tracking measures only e_p(0), i.e. where the reference starts; it says
nothing about the merit of the γ-law.  The same holds in the full
dynamic model, because the cascade controller sees p_d(γ(t)) as an
ordinary reference (its exact second derivative is supplied as
feedforward), so the position/attitude result of
quadcopter_certified_lyapunov.py applies unchanged.
"""

from __future__ import annotations
import sympy as sp


def derive_path_following() -> dict:
    R, v_d, k_gamma = sp.symbols(r"R v_d k_\gamma", positive=True)
    gamma = sp.Symbol(r"\gamma", real=True)
    k = sp.symbols("k_x k_y k_z", positive=True)
    K = sp.diag(*k)
    e_p = sp.Matrix(sp.symbols("e_x e_y e_z", real=True))
    gamma_dot = sp.Symbol(r"\dot\gamma", real=True)        # arbitrary, then specialised

    p_d   = sp.Matrix([R * sp.cos(gamma), R * sp.sin(gamma), 0])
    dp_d  = p_d.diff(gamma)
    t_hat = sp.simplify(dp_d / R)
    n_hat = -p_d / R                                          # inward normal [-cos γ, -sin γ, 0]

    v_cmd   = dp_d * gamma_dot - K * e_p
    e_p_dot = sp.simplify(v_cmd - dp_d * gamma_dot)          # ṗ − ṗ_d with ṗ = v_cmd

    V     = sp.Rational(1, 2) * e_p.dot(e_p)
    V_dot = sp.expand(e_p.dot(e_p_dot))

    e_s = t_hat.dot(e_p)
    gamma_law = v_d + k_gamma * e_s

    return {
        "p_d": p_d, "dp_d": dp_d, "t_hat": t_hat, "n_hat": n_hat,
        "e_s": e_s, "e_n": n_hat.dot(e_p),
        "gamma_law": gamma_law,
        "v_cmd": v_cmd.subs(gamma_dot, gamma_law),
        "e_p_dot": e_p_dot,
        "V": V, "V_dot": V_dot,
        "gamma_dot_at_zero_error": gamma_law.subs({s: 0 for s in e_p}),
        "checks": {
            "error_dynamics_independent_of_gamma_dot": e_p_dot.equals(-K * e_p) and not e_p_dot.has(gamma_dot),
            "V_dot_is_minus_eT_K_e": sp.simplify(V_dot + (e_p.T * K * e_p)[0, 0]) == 0,
        },
    }
