"""Arm-angle psi (R6) and relative-pose (R9) tests — offline, no ROS, no xacro.

  * psi is reported and matches finite differences (srs_7dof);
  * a desired-psi soft task changes psi while the 6-DOF pose residual stays
    within tol (null-space-only motion) (srs_7dof);
  * a two-tip rigid relative constraint holds the relative transform while an
    absolute task translates the pair (dual_arm).
"""

import numpy as np
import pytest

from ikt_inverse_kinematics import ik_core
from ikt_inverse_kinematics.tasks import Task
from ikt_inverse_kinematics.arm_angle import (
    SRSChain, compute_psi, psi_jacobian, make_arm_angle_extra_task,
)
from ikt_inverse_kinematics.relative import make_relative_extra_task

# srs_7dof: shoulder/elbow/wrist marker frames + a single 7-DOF chain.
_SRS = [f"joint{i}" for i in range(1, 8)]
_CHAIN = SRSChain("arm", "shoulder_link", "elbow_link", "wrist_link",
                  "base_link", "tool0")
# dual_arm: two 6-DOF arms.
_RIGHT = [f"right_arm_joint{i}" for i in range(1, 7)]
_LEFT = [f"left_arm_joint{i}" for i in range(1, 7)]


def _rand_q(model, rng, joints, scale=0.6):
    lo, hi = model.joint_limits()
    q = np.zeros(model.nq)
    for jn in joints:
        i = model.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        q[i] = 0.5 * (a + b) + 0.5 * (b - a) * scale * rng.uniform(-1, 1)
    return q


def test_psi_defined_for_bent_arm(srs_model):
    rng = np.random.default_rng(2)
    got = 0
    for _ in range(10):
        q = _rand_q(srs_model, rng, _SRS)
        psi = compute_psi(srs_model, q, _CHAIN)
        if psi is not None:
            got += 1
            assert -np.pi - 1e-6 <= psi <= np.pi + 1e-6
    assert got >= 7, "psi should be defined for most random bent postures"


def test_psi_jacobian_matches_finite_difference(srs_model):
    rng = np.random.default_rng(4)
    for _ in range(10):
        q = _rand_q(srs_model, rng, _SRS)
        psi0, J = psi_jacobian(srs_model, q, _CHAIN)
        if psi0 is not None:
            break
    if psi0 is None:
        pytest.skip("psi undefined at sampled configs")
    i = srs_model.q_index("joint1")
    dq = np.zeros(srs_model.nq); dq[i] = 1e-4
    p1 = compute_psi(srs_model, q + dq, _CHAIN)
    p0 = compute_psi(srs_model, q - dq, _CHAIN)
    fd = ((p1 - p0 + np.pi) % (2 * np.pi) - np.pi) / 2e-4
    assert abs(fd - J[0, i]) < 1e-2


def test_desired_psi_moves_elbow_holding_pose(srs_model):
    """A soft psi task changes psi while the 6-DOF pose residual stays small."""
    rng = np.random.default_rng(6)
    frame = "tool0"
    for _ in range(20):
        q0 = _rand_q(srs_model, rng, _SRS)
        psi0 = compute_psi(srs_model, q0, _CHAIN)
        if psi0 is not None:
            break
    if psi0 is None:
        pytest.skip("psi undefined")
    p_t, quat_t = srs_model.fk(q0, frame)
    psi_des = psi0 + 0.3
    extra = make_arm_angle_extra_task(srs_model, _CHAIN, psi_des, stiffness=0.5)
    pose_task = Task.pose(frame, tuple(p_t), tuple(quat_t))
    sol = ik_core.solve(srs_model, q0, [pose_task], active_joints=_SRS,
                        extra_tasks=[extra],
                        params=ik_core.SolveParams(max_iters=300))
    assert sol.max_pos_err() <= 2e-3 and sol.max_ori_err() <= 5e-3
    psi_new = compute_psi(srs_model, sol.q, _CHAIN)
    assert psi_new is not None
    assert abs(psi_new - psi_des) < abs(psi0 - psi_des), \
        f"psi did not move toward desired: {psi0:.3f}->{psi_new:.3f} (des {psi_des:.3f})"


def test_relative_pose_holds_while_pair_moves(dual_model):
    """Two-tip rigid relative constraint holds rel transform while pair translates."""
    import pinocchio as pin
    rng = np.random.default_rng(8)
    q0 = _rand_q(dual_model, rng, _RIGHT + _LEFT, scale=0.4)
    Xa0 = dual_model.fk_se3(q0, "right_arm_tool0")
    Xb0 = dual_model.fk_se3(q0, "left_arm_tool0")
    X_rel0 = Xb0.inverse() * Xa0
    rel_quat = pin.Quaternion(X_rel0.rotation)
    rel_q_wxyz = (rel_quat.w, rel_quat.x, rel_quat.y, rel_quat.z)
    rel_extra = make_relative_extra_task(
        dual_model, "right_arm_tool0", "left_arm_tool0",
        tuple(X_rel0.translation), rel_q_wxyz, stiffness=[1, 1, 1, 1, 1, 1])
    pr, qr = dual_model.fk(q0, "right_arm_tool0")
    pr_target = (pr[0] + 0.04, pr[1], pr[2])
    abs_task = Task.pose("right_arm_tool0", pr_target, tuple(qr))
    sol = ik_core.solve(dual_model, q0, [abs_task],
                        extra_tasks=[rel_extra],
                        params=ik_core.SolveParams(max_iters=400))
    Xa1 = dual_model.fk_se3(sol.q, "right_arm_tool0")
    Xb1 = dual_model.fk_se3(sol.q, "left_arm_tool0")
    X_rel1 = Xb1.inverse() * Xa1
    rel_drift = np.linalg.norm(pin.log6(X_rel1.inverse() * X_rel0).vector)
    assert rel_drift < 2e-2, f"relative pose drifted {rel_drift:.4f}"
    pl1, _ = dual_model.fk(sol.q, "left_arm_tool0")
    pl0, _ = dual_model.fk(q0, "left_arm_tool0")
    assert np.linalg.norm(pl1 - pl0) > 1e-3, "left tip should follow to hold the object"
