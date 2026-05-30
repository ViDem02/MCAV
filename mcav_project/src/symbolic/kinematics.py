# src/symbolic/kinematics.py
from sympy import symbols, Matrix, cos, sin, simplify, lambdify

def unicycle_model():
    x, y, theta, v, omega = symbols("x y θ v ω")
    f = Matrix([v * cos(theta), v * sin(theta), omega])
    return {"state": (x, y, theta), "input": (v, omega), "f": f}