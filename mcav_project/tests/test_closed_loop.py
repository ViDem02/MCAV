"""End-to-end checks on the MuJoCo closed loop with the LMI gains."""
import numpy as np
import pytest

from src.control.cascade_controller import vehicle_parameters
from src.design.lmi_gains import design_cascade_gains
from src.experiments import load, load_model, simulate_quad


@pytest.fixture(scope="module")
def cfg():
    return load("quadcopter")


@pytest.fixture(scope="module")
def gains(cfg):
    return design_cascade_gains(cfg, load_model(cfg))["controller"]


@pytest.fixture(scope="module")
def run(cfg, gains):
    return simulate_quad(cfg, gains, t_end=30.0)


def test_vehicle_parameters_from_model(cfg):
    p = vehicle_parameters(load_model(cfg))
    assert np.isclose(p["mass"], 0.8) and np.allclose(p["J"], [0.0035, 0.0035, 0.006])
    assert np.allclose(p["A"][0], 1.0)                       # every motor thrusts along body z


def test_mission_converges(run):
    assert run["e_norm"].iloc[-1] < 1e-6


def test_motor_forces_within_limits(run, cfg):
    f = run[["f1", "f2", "f3", "f4"]].to_numpy()
    f_max = load_model(cfg).actuator_ctrlrange[:, 1].min()
    assert f.min() >= 0.0 and f.max() <= f_max


def test_attitude_lyapunov_function_never_increases(run):
    assert np.diff(run["V2"].to_numpy()[2:]).max() < 1e-9   # Ω_d needs two samples


def test_desired_rate_matches_steady_circle(run, cfg):
    # steady circle: R_d(t) = Rz(ωt) R_d(0) ⇒ ‖Ω_d‖ = ω, and the backward difference is exact
    Wd = run[["Wdx", "Wdy", "Wdz"]].to_numpy()[-1]
    assert abs(np.linalg.norm(Wd) - cfg["trajectory"]["omega"]) < 1e-6


def test_pd_law_leaves_residual_error(cfg, gains):
    pd = simulate_quad(cfg, gains, feedforward=False, t_end=30.0)
    assert pd["e_norm"].iloc[-1] > 1e-4
