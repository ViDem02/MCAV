import sympy as sp

def derive_lyapunov():
    """
    Derives Lyapunov-based kinematic control for a quadcopter in 3D space tracking a trajectory.
    We model the quadcopter kinematically as a point mass in R^3 with heading.
    State: p = [x, y, z]^T, psi (yaw).
    Inputs: v = [v_x, v_y, v_z]^T, omega_psi.
    Reference: p_d = [x_d, y_d, z_d]^T, psi_d.
    """
    # 1. Define states and references
    x, y, z, psi = sp.symbols('x y z psi', real=True)
    xd, yd, zd, psid = sp.symbols('x_d y_d z_d psi_d', real=True)
    vxd, vyd, vzd, psid_dot = sp.symbols(r'v_{x\,d} v_{y\,d} v_{z\,d} \dot{\psi}_{d}', real=True)

    # 2. Define Tracking Errors
    ex = x - xd
    ey = y - yd
    ez = z - zd
    epsi = psi - psid

    # 3. Define the Lyapunov function Candidate
    # We want to drive all position and yaw errors to zero
    V = sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2 + epsi**2)

    # 4. Propose control inputs to make \dot{V} negative definite
    # \dot{x} = v_x, \dot{y} = v_y, \dot{z} = v_z, \dot{\psi} = \omega_\psi
    vx, vy, vz, w_psi = sp.symbols(r'v_x v_y v_z \omega_\psi', real=True)
    kx, ky, kz, kpsi = sp.symbols(r'k_x k_y k_z k_\psi', positive=True, real=True)

    # Proposed Control Laws
    law_vx = vxd - kx * ex
    law_vy = vyd - ky * ey
    law_vz = vzd - kz * ez
    law_wpsi = psid_dot - kpsi * epsi

    # 5. Compute \dot{V}
    # \dot{V} = e_x \dot{e}_x + e_y \dot{e}_y + e_z \dot{e}_z + e_\psi \dot{e}_\psi
    # where \dot{e}_x = v_x - v_{x,d}, etc.
    ex_dot = vx - vxd
    ey_dot = vy - vyd
    ez_dot = vz - vzd
    epsi_dot = w_psi - psid_dot

    V_dot = ex * ex_dot + ey * ey_dot + ez * ez_dot + epsi * epsi_dot

    # Substitute our control laws into V_dot
    V_dot_sub = V_dot.subs({
        vx: law_vx,
        vy: law_vy,
        vz: law_vz,
        w_psi: law_wpsi
    }).simplify()

    return {
        "V": V,
        "V_dot": V_dot,
        "V_dot_sub": V_dot_sub,
        "law_vx": law_vx,
        "law_vy": law_vy,
        "law_vz": law_vz,
        "law_wpsi": law_wpsi,
    }
