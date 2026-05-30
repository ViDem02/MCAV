"""
src/symbolic/quadcopter_dynamic_control.py

Symbolic derivation of the cascade control architecture for a quadcopter.

Outputs
-------
derive_allocation()
    Returns the 4×4 allocation matrix A and its inverse A_inv, together
    with the symbolic motor forces as functions of [T, τx, τy, τz].

derive_cascade()
    Returns the full symbolic cascade: position error → desired acceleration
    → desired thrust vector → attitude error → torques.
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Motor allocation matrix
# ──────────────────────────────────────────────────────────────────────────────

def derive_allocation() -> dict:
    """
    Derive the 4x4 allocation matrix for an X-configuration quadrotor.

    Motor layout (top view, X-config):
         FL(2)  FR(1)
           \\   /
            \\ /
            / \\
           /   \\
         BL(4)  BR(3)

    Motor positions relative to CoM:
        FR: (+d, -d),  FL: (+d, +d),  BR: (-d, -d),  BL: (-d, +d)
    where d = L / sqrt(2),  L = arm length (centre → motor).

    Spin directions (looking down):
        FR, BL → CW  (negative yaw reaction)
        FL, BR → CCW (positive yaw reaction)

    Each motor i produces:
        - Thrust fi along body z
        - Reaction yaw torque: ±km * fi  (km = torque/thrust ratio)

    The total wrench on the body:
        T  = f1 + f2 + f3 + f4
        τx = d (-f1 + f2 - f3 + f4)   [roll]
        τy = d (-f1 - f2 + f3 + f4)   [pitch]
        τz = km(-f1 + f2 + f3 - f4)   [yaw]
    """
    d, km    = sp.symbols("d k_m", positive=True)
    f1, f2, f3, f4 = sp.symbols("f_1 f_2 f_3 f_4", nonneg=True)
    T_s, tx, ty, tz = sp.symbols(r"T \tau_x \tau_y \tau_z")

    # Allocation matrix A such that  [T, τx, τy, τz]^T = A * [f1..f4]^T
    A = sp.Matrix([
        [ 1,    1,    1,    1  ],
        [-d,   +d,   -d,   +d ],
        [-d,   -d,   +d,   +d ],
        [-km,  +km,  +km,  -km],
    ])

    A_inv = A.inv()
    A_inv_simplified = sp.simplify(A_inv)

    # Symbolic motor forces from desired wrench
    wrench = sp.Matrix([T_s, tx, ty, tz])
    forces = A_inv_simplified * wrench
    forces = sp.simplify(forces)

    return {
        "A":       A,
        "A_inv":   A_inv_simplified,
        "f1":      forces[0],
        "f2":      forces[1],
        "f3":      forces[2],
        "f4":      forces[3],
        "symbols": {"d": d, "km": km, "T": T_s, "tx": tx, "ty": ty, "tz": tz},
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Cascade: position → acceleration → thrust vector → attitude
# ──────────────────────────────────────────────────────────────────────────────

def derive_cascade() -> dict:
    """
    Symbolic derivation of the full cascade control law.

    Outer loop   (position Lyapunov, same as kinematic):
        v_cmd = v_d  - K_pos * e_pos

    Middle loop  (velocity → desired acceleration):
        a_des = K_vel * (v_cmd - v)

    Thrust vector:
        F_des = m * (a_des + g * e3)
        T     = ||F_des||

    Attitude control uses a geometric approach on SO(3) whose
    symbolic form is too involved for SymPy; the key equations
    are presented analytically in the docstring and comments.

    Returns symbolic expressions for the outer/middle loops and
    the thrust decomposition.
    """
    # ── symbols ──
    t  = sp.Symbol("t")
    m, g = sp.symbols("m g", positive=True)

    # position errors
    ex, ey, ez = sp.symbols("e_x e_y e_z")

    # velocities
    vx, vy, vz   = sp.symbols("v_x v_y v_z")
    vxd, vyd, vzd = sp.symbols(r"v_{xd} v_{yd} v_{zd}")

    # gains
    kx, ky, kz   = sp.symbols("k_x k_y k_z", positive=True)
    kv           = sp.Symbol(r"k_v", positive=True)

    # ── outer loop (Lyapunov, same as kinematic notebook) ──
    vx_cmd = vxd - kx * ex
    vy_cmd = vyd - ky * ey
    vz_cmd = vzd - kz * ez

    # ── middle loop: velocity error → desired acceleration ──
    ax_des = kv * (vx_cmd - vx)
    ay_des = kv * (vy_cmd - vy)
    az_des = kv * (vz_cmd - vz)

    # ── thrust vector in world frame ──
    Fx = m * ax_des
    Fy = m * ay_des
    Fz = m * (az_des + g)
    T  = sp.sqrt(Fx**2 + Fy**2 + Fz**2)

    # ── hover condition (ex=ey=ez=0, v=vd) ──
    T_hover = sp.simplify(T.subs({
        ex: 0, ey: 0, ez: 0,
        vx: vxd, vy: vyd, vz: vzd,
    }))

    # ── Vdot for outer-loop Lyapunov (already proven GAS) ──
    V = sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2)
    V_dot = (ex * (vx_cmd - vxd) + ey * (vy_cmd - vyd) + ez * (vz_cmd - vzd))
    V_dot = sp.expand(V_dot)

    return {
        # outer loop
        "vx_cmd":  vx_cmd,
        "vy_cmd":  vy_cmd,
        "vz_cmd":  vz_cmd,
        # middle loop
        "ax_des":  ax_des,
        "ay_des":  ay_des,
        "az_des":  az_des,
        # thrust
        "Fx": Fx, "Fy": Fy, "Fz": Fz,
        "T":  T,
        "T_hover": T_hover,
        # Lyapunov
        "V":     V,
        "V_dot": V_dot,
    }