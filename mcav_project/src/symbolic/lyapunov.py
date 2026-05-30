import sympy as sp

def derive_circular_path_following():
    """
    Derives the control law omega for a unicycle to follow a circular path
    of radius R. The path is traversed counter-clockwise.
    Returns a dictionary of symbolic expressions.
    """
    # Define symbolic variables
    e, t_tilde = sp.symbols('e \\tilde{\\theta}', real=True)
    v, R = sp.symbols('v R', real=True, positive=True)
    k_y, k_theta = sp.symbols('k_y k_\\theta', real=True, positive=True)
    omega = sp.symbols('omega', real=True)
    
    # Error Kinematics
    # e: distance to the circle = sqrt(x^2 + y^2) - R
    # t_tilde: heading relative to the path's tangent -> theta - (atan2(y,x) + pi/2)
    e_dot = -v * sp.sin(t_tilde)
    
    # Path tangent angle derivative
    t_p_dot = v * sp.cos(t_tilde) / (R + e)
    
    # Heading error derivative
    t_tilde_dot = omega - t_p_dot
    
    # Proposed Lyapunov Candidate
    V = 0.5 * e**2 + (1 - sp.cos(t_tilde)) / k_y
    
    # Derivative of Lyapunov Candidate
    # V_dot = dV/de * e_dot + dV/dt_tilde * t_tilde_dot
    V_dot = e * e_dot + (sp.sin(t_tilde) / k_y) * t_tilde_dot
    
    # Propose control law for omega
    omega_law = t_p_dot + k_y * e * v - k_theta * sp.sin(t_tilde)
    
    # Substitute omega into V_dot and simplify
    V_dot_sub = sp.simplify(V_dot.subs(omega, omega_law))
    
    return {
        'V': V,
        'e_dot': e_dot,
        't_tilde_dot': t_tilde_dot,
        'omega_law': omega_law,
        'V_dot_sub': V_dot_sub
    }

if __name__ == '__main__':
    res = derive_circular_path_following()
    print("Lyapunov candidate V:", res['V'])
    print("Derivative V_dot (controlled):", res['V_dot_sub'])
