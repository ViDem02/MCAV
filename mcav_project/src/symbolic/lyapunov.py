"""
src/symbolic/lyapunov.py

Lyapunov path-following law for a unicycle on a circle of radius R
(counter-clockwise), kinematic model ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω
with constant forward speed v > 0.

Errors:  e = √(x² + y²) − R  (cross-track),  θ̃ = θ − (atan2(y, x) + π/2).

Result (all identities computed below):
    V = ½e² + (1 − cos θ̃)/k_y,   V̇ = −(k_θ/k_y) sin² θ̃ ≤ 0.
LaSalle on the compact sublevel set Ω_c = {V ≤ c}, c < ½R² (which keeps
|e| < R, i.e. away from the singularity at the circle centre) gives
convergence to M = {(0, 0), (0, π)}.  (0, π) — driving the circle
clockwise — is a saddle, so it attracts only a measure-zero set.  If in
addition c < 2/k_y, (0, π) ∉ Ω_c and every trajectory starting in Ω_c
converges to (0, 0).  The result is not global: initial conditions with
V ≥ ½R² are not covered.
"""

import sympy as sp


def derive_circular_path_following() -> dict:
    e, t_tilde = sp.symbols(r"e \tilde{\theta}", real=True)
    v, R = sp.symbols("v R", positive=True)
    k_y, k_theta = sp.symbols(r"k_y k_\theta", positive=True)
    omega = sp.Symbol(r"\omega", real=True)

    # ── error kinematics ─────────────────────────────────────────────────────
    e_dot       = -v * sp.sin(t_tilde)
    t_p_dot     = v * sp.cos(t_tilde) / (R + e)          # rate of the path tangent angle
    t_tilde_dot = omega - t_p_dot

    # ── Lyapunov function and control law ───────────────────────────────────
    V         = sp.Rational(1, 2) * e**2 + (1 - sp.cos(t_tilde)) / k_y
    omega_law = t_p_dot + k_y * e * v - k_theta * sp.sin(t_tilde)

    V_dot     = sp.diff(V, e) * e_dot + sp.diff(V, t_tilde) * t_tilde_dot
    V_dot_sub = sp.simplify(V_dot.subs(omega, omega_law))

    # ── closed loop, equilibria, linearisation ──────────────────────────────
    f = sp.Matrix([e_dot, t_tilde_dot.subs(omega, omega_law)])
    x = sp.Matrix([e, t_tilde])
    Jac = f.jacobian(x)
    J_good   = sp.simplify(Jac.subs({e: 0, t_tilde: 0}))
    J_saddle = sp.simplify(Jac.subs({e: 0, t_tilde: sp.pi}))

    checks = {
        "V_dot_is_minus_ktheta_over_ky_sin2": sp.simplify(V_dot_sub + k_theta / k_y * sp.sin(t_tilde)**2) == 0,
        "origin_is_equilibrium":  sp.simplify(f.subs({e: 0, t_tilde: 0})) == sp.zeros(2, 1),
        "reverse_is_equilibrium": sp.simplify(f.subs({e: 0, t_tilde: sp.pi})) == sp.zeros(2, 1),
        # 2×2 Hurwitz ⇔ trace < 0 and det > 0
        "origin_hurwitz": bool(sp.simplify(J_good.trace()).is_negative and sp.simplify(J_good.det()).is_positive),
        # det < 0 ⇒ one positive and one negative eigenvalue (saddle)
        "reverse_is_saddle": bool(sp.simplify(J_saddle.det()).is_negative),
    }

    return {
        "V": V,
        "e_dot": e_dot,
        "t_tilde_dot": t_tilde_dot,
        "omega_law": omega_law,
        "V_dot_sub": V_dot_sub,
        "J_origin": J_good,
        "J_reverse": J_saddle,
        "V_at_reverse": V.subs({e: 0, t_tilde: sp.pi}),
        "checks": checks,
    }


if __name__ == "__main__":
    res = derive_circular_path_following()
    print("V     =", res["V"])
    print("V_dot =", res["V_dot_sub"])
    print(res["checks"])
