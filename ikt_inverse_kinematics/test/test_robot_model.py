"""robot_model.py tests on the bundled dual_arm URDF — offline, no ROS, no xacro.

Checks FK, the analytic frame Jacobian (vs finite differences), joint limits,
joint<->index mapping, and virtual tool-frame augmentation.
"""

import numpy as np

from ikt_inverse_kinematics.robot_model import (
    RobotModel, R_from_quat_wxyz, quat_wxyz_from_R,
)
from conftest import load_bundled

_R_TIP = "right_arm_tool0"
_L_TIP = "left_arm_tool0"


def test_joint_names_and_count(dual_model):
    assert dual_model.nq == 12, "dual_arm should expose 12 movable DOF"
    for jn in ("right_arm_joint1", "left_arm_joint6"):
        assert jn in dual_model.joint_names
        idx = dual_model.q_index(jn)
        assert 0 <= idx < dual_model.nq


def test_frames_exist(dual_model):
    assert dual_model.has_frame(_R_TIP)
    assert dual_model.has_frame(_L_TIP)
    assert not dual_model.has_frame("nonexistent_frame")


def test_joint_limits_finite_and_ordered(dual_model):
    lo, hi = dual_model.joint_limits()
    assert lo.shape == (12,) and hi.shape == (12,)
    assert np.all(hi > lo)
    assert np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    import pinocchio as pin
    for _ in range(20):
        v = rng.standard_normal(3)
        ang = np.linalg.norm(v)
        if ang < 1e-6:
            continue
        axis = v / ang
        R = pin.exp3(axis * ang)
        q = quat_wxyz_from_R(R)
        R2 = R_from_quat_wxyz(q)
        assert np.allclose(R, R2, atol=1e-9)


def test_fk_changes_with_q(dual_model):
    q0 = dual_model.neutral()
    p0, _ = dual_model.fk(q0, _R_TIP)
    q1 = q0.copy()
    q1[dual_model.q_index("right_arm_joint2")] = 0.5
    p1, _ = dual_model.fk(q1, _R_TIP)
    assert np.linalg.norm(p1 - p0) > 1e-3, "moving a shoulder joint must move the tip"


def test_frame_jacobian_matches_finite_difference(dual_model):
    rng = np.random.default_rng(1)
    frame = _R_TIP
    q = dual_model.neutral() + 0.2 * rng.standard_normal(dual_model.nq)
    J = dual_model.frame_jacobian(q, frame)
    assert J.shape == (6, dual_model.nq)

    eps = 1e-6
    Jfd = np.zeros((6, dual_model.nq))
    p0, quat0 = dual_model.fk(q, frame)
    R0 = R_from_quat_wxyz(quat0)
    import pinocchio as pin
    for i in range(dual_model.nq):
        dq = np.zeros(dual_model.nq)
        dq[i] = eps
        p1, quat1 = dual_model.fk(q + dq, frame)
        R1 = R_from_quat_wxyz(quat1)
        Jfd[:3, i] = (p1 - p0) / eps
        Jfd[3:, i] = pin.log3(R1 @ R0.T) / eps
    assert np.allclose(J, Jfd, atol=1e-4), \
        f"max diff {np.max(np.abs(J - Jfd)):.2e}"


def test_virtual_tool_frame(dual_model):
    base_urdf = load_bundled("dual_arm")
    vm = RobotModel(base_urdf, virtual_frames=[{
        "name": "right_tool", "parent": _R_TIP,
        "xyz": [0.0, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0],
    }])
    assert vm.has_frame("right_tool")
    q = vm.neutral()
    p_tip, _ = vm.fk(q, _R_TIP)
    p_tool, _ = vm.fk(q, "right_tool")
    assert np.isclose(np.linalg.norm(p_tool - p_tip), 0.10, atol=1e-6)


def test_active_mask(dual_model):
    m = dual_model.active_mask(["right_arm_joint1", "right_arm_joint2"])
    assert m.sum() == 2
    assert m[dual_model.q_index("right_arm_joint1")]
    assert dual_model.active_mask(None).all()


def test_supporting_joints(dual_model):
    """Joints on the path to the right tip are exactly the right-arm joints."""
    js = dual_model.supporting_joints(_R_TIP)
    assert js == [f"right_arm_joint{i}" for i in range(1, 7)]
