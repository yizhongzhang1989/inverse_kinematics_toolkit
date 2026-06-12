"""Phase 1 tests: robot_model.py (Pinocchio) — offline, no ROS.

Renders the dual-arm RM75 URDF via xacro once, then checks FK, the analytic
frame Jacobian (against finite differences), joint limits, joint<->index
mapping, and virtual tool-frame augmentation.
"""

import os
import subprocess

import numpy as np
import pytest

from ikt_inverse_kinematics.robot_model import (
    RobotModel, R_from_quat_wxyz, quat_wxyz_from_R,
)

_XACRO = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "src", "robot_description", "urdf", "robot.urdf.xacro",
)


def _load_urdf() -> str:
    xacro = os.path.abspath(_XACRO)
    if not os.path.exists(xacro):
        pytest.skip(f"dual-arm xacro not found at {xacro}")
    try:
        out = subprocess.run(
            ["xacro", xacro, "use_mock_hardware:=true"],
            check=True, capture_output=True, text=True)
    except Exception as exc:  # xacro not on PATH in a bare env
        pytest.skip(f"xacro unavailable: {exc}")
    return out.stdout


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel(_load_urdf())


def test_joint_names_and_count(model):
    assert model.nq == 14, "RM75 dual-arm should expose 14 movable DOF"
    for jn in ("right_arm_joint1", "left_arm_joint7"):
        assert jn in model.joint_names
        idx = model.q_index(jn)
        assert 0 <= idx < model.nq


def test_frames_exist(model):
    assert model.has_frame("right_arm_Link7")
    assert model.has_frame("left_arm_Link7")
    assert not model.has_frame("nonexistent_frame")


def test_joint_limits_finite_and_ordered(model):
    lo, hi = model.joint_limits()
    assert lo.shape == (14,) and hi.shape == (14,)
    # RM75 joints are limited (not continuous): finite and lo < hi.
    assert np.all(hi > lo)
    assert np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        v = rng.standard_normal(3)
        ang = np.linalg.norm(v)
        if ang < 1e-6:
            continue
        axis = v / ang
        # build a rotation, round-trip through quat
        import pinocchio as pin
        R = pin.exp3(axis * ang)
        q = quat_wxyz_from_R(R)
        R2 = R_from_quat_wxyz(q)
        assert np.allclose(R, R2, atol=1e-9)


def test_fk_changes_with_q(model):
    q0 = model.neutral()
    p0, _ = model.fk(q0, "right_arm_Link7")
    q1 = q0.copy()
    q1[model.q_index("right_arm_joint2")] = 0.5
    p1, _ = model.fk(q1, "right_arm_Link7")
    assert np.linalg.norm(p1 - p0) > 1e-3, "moving a shoulder joint must move the tip"


def test_frame_jacobian_matches_finite_difference(model):
    rng = np.random.default_rng(1)
    frame = "right_arm_Link7"
    q = model.neutral() + 0.2 * rng.standard_normal(model.nq)
    J = model.frame_jacobian(q, frame)
    assert J.shape == (6, model.nq)

    eps = 1e-6
    Jfd = np.zeros((6, model.nq))
    p0, quat0 = model.fk(q, frame)
    R0 = R_from_quat_wxyz(quat0)
    import pinocchio as pin
    for i in range(model.nq):
        dq = np.zeros(model.nq)
        dq[i] = eps
        p1, quat1 = model.fk(q + dq, frame)
        R1 = R_from_quat_wxyz(quat1)
        Jfd[:3, i] = (p1 - p0) / eps
        Jfd[3:, i] = pin.log3(R1 @ R0.T) / eps
    # columns for joints that don't affect this frame are ~0 in both.
    assert np.allclose(J, Jfd, atol=1e-4), \
        f"max diff {np.max(np.abs(J - Jfd)):.2e}"


def test_virtual_tool_frame(model):
    base_urdf = _load_urdf()
    vm = RobotModel(base_urdf, virtual_frames=[{
        "name": "right_tool", "parent": "right_arm_Link7",
        "xyz": [0.0, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0],
    }])
    assert vm.has_frame("right_tool")
    q = vm.neutral()
    p_tip, _ = vm.fk(q, "right_arm_Link7")
    p_tool, _ = vm.fk(q, "right_tool")
    # tool is offset 10 cm from the tip along the tip's local z.
    assert np.isclose(np.linalg.norm(p_tool - p_tip), 0.10, atol=1e-6)


def test_active_mask(model):
    m = model.active_mask(["right_arm_joint1", "right_arm_joint2"])
    assert m.sum() == 2
    assert m[model.q_index("right_arm_joint1")]
    assert model.active_mask(None).all()
