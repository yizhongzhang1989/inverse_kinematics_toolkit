"""Tests for the bundled sample-URDF asset helpers — offline, no ROS."""

import pytest

from ikt_core import assets
from ikt_core.robot_model import RobotModel

_EXPECTED = {"planar_3r", "arm_6dof", "srs_7dof", "dual_arm"}


def test_list_sample_urdfs():
    names = set(assets.list_sample_urdfs())
    assert _EXPECTED <= names, f"missing bundled URDFs: {_EXPECTED - names}"


def test_sample_urdf_path_exists_and_loads():
    for name in _EXPECTED:
        path = assets.sample_urdf_path(name)
        assert path.endswith(f"{name}.urdf")
        xml = assets.load_sample_urdf(name)
        assert xml.lstrip().startswith("<")
        # every bundled URDF must build a kinematic model
        model = RobotModel(xml)
        assert model.nq >= 3


def test_sample_urdf_path_accepts_suffix():
    a = assets.sample_urdf_path("arm_6dof")
    b = assets.sample_urdf_path("arm_6dof.urdf")
    assert a == b


def test_unknown_sample_raises():
    with pytest.raises(FileNotFoundError):
        assets.sample_urdf_path("does_not_exist")


def test_expected_dof_counts():
    counts = {name: RobotModel(assets.load_sample_urdf(name)).nq
              for name in _EXPECTED}
    assert counts == {"planar_3r": 3, "arm_6dof": 6,
                      "srs_7dof": 7, "dual_arm": 12}
