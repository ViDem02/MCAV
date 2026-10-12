"""LMI certificates re-checked numerically, independently of the SDP solver."""
import numpy as np
import pytest

from src.design.lmi_gains import DOUBLE_INTEGRATOR_A, design_cascade_gains, fsf_input_bounded
from src.experiments import load, load_model


def test_double_integrator_certificate():
    B = np.array([[0.0], [1.0]])
    X0 = [np.array([s1 * 2.0, s2 * 1.0]) for s1 in (-1, 1) for s2 in (-1, 1)]
    d = fsf_input_bounded(DOUBLE_INTEGRATOR_A, B, [3.0], X0, alpha_max=10.0)
    P, Acl = np.linalg.inv(d.W), DOUBLE_INTEGRATOR_A + B @ d.K
    # rate α:  Acl W + W Aclᵀ + 2α W ⪯ 0
    assert np.linalg.eigvalsh(Acl @ d.W + d.W @ Acl.T + 2 * d.alpha * d.W).max() <= 1e-6
    # every design point lies in E = {xᵀPx ≤ 1}
    assert all(x0 @ P @ x0 <= 1 + 1e-4 for x0 in X0)
    # max over E of |K x| = √(K W Kᵀ) ≤ ū
    assert np.sqrt(d.K @ d.W @ d.K.T).item() <= 3.0 * (1 + 1e-4)


@pytest.fixture(scope="module")
def design():
    cfg = load("quadcopter")
    return design_cascade_gains(cfg, load_model(cfg))


def test_cascade_design(design):
    assert all(design["position"][a]["kv"] > 0 and design["position"][a]["k"] > 0 for a in "xyz")
    att = design["attitude"]
    assert np.all(att["G"] > 0) and np.all(att["kW"] > 0)
    assert np.allclose(0.5 * (att["G"].sum() - att["G"]), att["kappa"])   # κ_j = ½(tr G − g_j)
    # inner loop faster than outer loop
    assert att["alpha"].min() > max(design["position"][a]["alpha"] for a in "xyz")
