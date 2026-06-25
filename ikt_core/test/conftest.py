"""Shared pytest fixtures: build RobotModels from the bundled sample URDFs.

These make the test-suite fully self-contained — no xacro, no external
``robot_description``, no colcon workspace. Each fixture is session-scoped so
the (cheap) models are built once.
"""

import pytest

from ikt_core import assets
from ikt_core.robot_model import RobotModel


def load_bundled(name: str) -> str:
    """URDF XML string of a bundled sample (e.g. 'dual_arm')."""
    return assets.load_sample_urdf(name)


@pytest.fixture(scope="session")
def planar_model() -> RobotModel:
    return RobotModel(load_bundled("planar_3r"))


@pytest.fixture(scope="session")
def arm6_model() -> RobotModel:
    return RobotModel(load_bundled("arm_6dof"))


@pytest.fixture(scope="session")
def srs_model() -> RobotModel:
    return RobotModel(load_bundled("srs_7dof"))


@pytest.fixture(scope="session")
def dual_model() -> RobotModel:
    return RobotModel(load_bundled("dual_arm"))
