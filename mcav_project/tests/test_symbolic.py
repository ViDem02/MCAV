"""Every identity used in the stability analysis must simplify to zero."""
import pytest

from src.symbolic.quadcopter_certified_lyapunov import run_all_checks
from src.symbolic.quadcopter_path_following import derive_path_following

CHECKS = run_all_checks()


@pytest.mark.parametrize("name", sorted(CHECKS))
def test_quadrotor_identity(name):
    assert CHECKS[name], name


@pytest.mark.parametrize("name,ok", sorted(derive_path_following()["checks"].items()))
def test_path_following_identity(name, ok):
    assert ok, name
