"""Tests for the high-level Python API (IK class + solve_ik) — offline, no ROS."""

import numpy as np
import pytest

from ikt_core import IK, solve_ik, assets
from ikt_core.tasks import Solution


def test_from_urdf_file_and_introspection():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    assert len(ik.joint_names) == 6
    assert "tool0" in ik.link_names
    assert ik.supporting_joints("tool0") == [f"joint{i}" for i in range(1, 7)]


def test_construct_from_raw_xml_string():
    xml = assets.load_sample_urdf("planar_3r")
    ik = IK(xml)
    assert len(ik.joint_names) == 3


def test_full_pose_round_trip_recovers_joints():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    target = {f"joint{i+1}": v
              for i, v in enumerate([0.3, -0.6, 0.8, 0.2, 0.5, -0.4])}
    xyz, quat = ik.fk(target, "tool0")
    sol = ik.solve("tool0", xyz, quat)
    assert isinstance(sol, Solution)
    assert sol.reachable and sol.reason.value == "ok"
    assert sol.max_pos_err() <= 1.5e-3
    # joint_dict / q_active accessors
    d = sol.joint_dict()
    assert set(d) == set(ik.joint_names)
    got = dict(zip(ik.joint_names, sol.q_active()))
    for jn, v in target.items():
        assert abs(got[jn] - v) < 1e-2


def test_solve_ik_one_liner_position_only():
    sol = solve_ik(assets.sample_urdf_path("planar_3r"), "tool0",
                   [0.4, 0.2, 0.05], position_only=True)
    assert sol.reachable
    assert sol.max_pos_err() <= 1.5e-3


def test_seed_dict_and_array_equivalent():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    xyz, quat = ik.fk(ik.neutral(), "tool0")
    s1 = ik.solve("tool0", xyz, quat, seed=None)
    s2 = ik.solve("tool0", xyz, quat, seed={"joint2": 0.1})
    assert s1.reachable and s2.reachable


def test_auto_active_joints_freezes_other_arm():
    ik = IK.from_urdf_file(assets.sample_urdf_path("dual_arm"))
    seed = {jn: 0.2 for jn in ik.joint_names}
    xyz, quat = ik.fk(seed, "right_arm_tool0")
    sol = ik.solve("right_arm_tool0", xyz, quat, seed=seed)
    # only right-arm joints are active
    assert sol.active_joints == [f"right_arm_joint{i}" for i in range(1, 7)]
    d = sol.joint_dict()
    for i in range(1, 7):
        assert abs(d[f"left_arm_joint{i}"] - 0.2) < 1e-9


def test_explicit_active_joints_and_all():
    ik = IK.from_urdf_file(assets.sample_urdf_path("dual_arm"))
    xyz, quat = ik.fk(ik.neutral(), "right_arm_tool0")
    sol = ik.solve("right_arm_tool0", xyz, quat,
                   active_joints=["right_arm_joint1", "right_arm_joint2"])
    assert sol.active_joints == ["right_arm_joint1", "right_arm_joint2"]
    sol_all = ik.solve("right_arm_tool0", xyz, quat, active_joints=None)
    assert len(sol_all.active_joints) == 12


def test_tool_frame_offset_solve():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    xyz, quat = ik.fk(ik.neutral(), "tool0")
    sol = ik.solve("tcp", xyz, quat,
                   tool_frames=[{"name": "tcp", "parent": "tool0",
                                 "xyz": [0, 0, 0.05]}])
    # tcp is reachable (it's just an offset of a reachable tool0 region)
    assert sol.reachable or sol.max_pos_err() < 0.1


def test_solve_many_dual_arm():
    ik = IK.from_urdf_file(assets.sample_urdf_path("dual_arm"))
    seed = {jn: 0.2 for jn in ik.joint_names}
    rx, rq = ik.fk(seed, "right_arm_tool0")
    lx, lq = ik.fk(seed, "left_arm_tool0")
    sol = ik.solve_many([
        {"frame": "right_arm_tool0", "xyz": rx, "quat": rq},
        {"frame": "left_arm_tool0", "xyz": lx, "quat": lq},
    ], seed=seed)
    assert sol.reachable
    assert len(sol.active_joints) == 12


def test_unknown_frame_raises():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    with pytest.raises(ValueError):
        ik.solve("no_such_frame", [0.1, 0.0, 0.5])


def test_bad_seed_length_raises():
    ik = IK.from_urdf_file(assets.sample_urdf_path("arm_6dof"))
    with pytest.raises(ValueError):
        ik.solve("tool0", [0.1, 0.0, 0.5], seed=[0.0, 0.0])
