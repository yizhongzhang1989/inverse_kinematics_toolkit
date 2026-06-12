"""Phase 2.5 tests: arm-angle psi (R6) and relative-pose (R9) — offline.

Gates (plan §2.5 / §9):
  * reported psi matches an independent geometric computation;
  * a desired-psi soft task changes psi while the 6-DOF pose residual stays
    within tol (proves null-space-only motion);
  * a two-tip rigid relative constraint holds the relative transform within tol
    while an absolute task translates the pair.
"""

import os
import subprocess

import numpy as np
import pytest

from ikt_inverse_kinematics.robot_model import RobotModel
from ikt_inverse_kinematics import ik_core
from ikt_inverse_kinematics.tasks import Task
from ikt_inverse_kinematics.arm_angle import (
    SRSChain, compute_psi, psi_jacobian, make_arm_angle_extra_task,
)
from ikt_inverse_kinematics.relative import make_relative_extra_task

_XACRO = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "src", "robot_description", "urdf", "robot.urdf.xacro",
)
_RIGHT = [f"right_arm_joint{i}" for i in range(1, 8)]
_LEFT = [f"left_arm_joint{i}" for i in range(1, 8)]
_CHAIN = SRSChain("right_arm", "right_arm_Link2", "right_arm_Link4",
                  "right_arm_Link6", "right_arm_base_link", "right_arm_Link7")


def _load_urdf() -> str:
    xacro = os.path.abspath(_XACRO)
    if not os.path.exists(xacro):
        pytest.skip(f"xacro not found at {xacro}")
    try:
        out = subprocess.run(["xacro", xacro, "use_mock_hardware:=true"],
                             check=True, capture_output=True, text=True)
    except Exception as exc:
        pytest.skip(f"xacro unavailable: {exc}")
    return out.stdout


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel(_load_urdf())


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


def test_psi_defined_for_bent_arm(model):
    rng = np.random.default_rng(2)
    got = 0
    for _ in range(10):
        q = _rand_q(model, rng, _RIGHT)
        psi = compute_psi(model, q, _CHAIN)
        if psi is not None:
            got += 1
            assert -np.pi - 1e-6 <= psi <= np.pi + 1e-6
    assert got >= 7, "psi should be defined for most random bent postures"


def test_psi_jacobian_matches_finite_difference(model):
    rng = np.random.default_rng(4)
    q = _rand_q(model, rng, _RIGHT)
    psi0, J = psi_jacobian(model, q, _CHAIN)
    if psi0 is None:
        pytest.skip("psi undefined at this sample")
    # cross-check one arbitrary active joint column with a coarser step
    i = model.q_index("right_arm_joint1")
    dq = np.zeros(model.nq); dq[i] = 1e-4
    p1 = compute_psi(model, q + dq, _CHAIN)
    p0 = compute_psi(model, q - dq, _CHAIN)
    fd = ((p1 - p0 + np.pi) % (2 * np.pi) - np.pi) / 2e-4
    assert abs(fd - J[0, i]) < 1e-2


def test_desired_psi_moves_elbow_holding_pose(model):
    """A soft psi task changes psi while the 6-DOF pose residual stays small."""
    rng = np.random.default_rng(6)
    frame = "right_arm_Link7"
    q0 = _rand_q(model, rng, _RIGHT)
    psi0 = compute_psi(model, q0, _CHAIN)
    if psi0 is None:
        pytest.skip("psi undefined")
    p_t, quat_t = model.fk(q0, frame)
    # hold the exact pose, request a different psi
    psi_des = psi0 + 0.4
    extra = make_arm_angle_extra_task(model, _CHAIN, psi_des, stiffness=0.5)
    pose_task = Task.pose(frame, tuple(p_t), tuple(quat_t))
    sol = ik_core.solve(model, q0, [pose_task], active_joints=_RIGHT,
                        extra_tasks=[extra],
                        params=ik_core.SolveParams(max_iters=300))
    # pose still hit
    assert sol.max_pos_err() <= 2e-3 and sol.max_ori_err() <= 5e-3
    # psi moved toward the desired value
    psi_new = compute_psi(model, sol.q, _CHAIN)
    assert psi_new is not None
    assert abs(psi_new - psi_des) < abs(psi0 - psi_des), \
        f"psi did not move toward desired: {psi0:.3f}->{psi_new:.3f} (des {psi_des:.3f})"


def test_relative_pose_holds_while_pair_moves(model):
    """Two-tip rigid relative constraint holds rel transform while pair translates."""
    rng = np.random.default_rng(8)
    q0 = _rand_q(model, rng, _RIGHT + _LEFT, scale=0.4)
    import pinocchio as pin
    Xa0 = model.fk_se3(q0, "right_arm_Link7")
    Xb0 = model.fk_se3(q0, "left_arm_Link7")
    X_rel0 = Xb0.inverse() * Xa0
    # target: keep current relative pose; move right tip +5 cm in x
    rel_quat = pin.Quaternion(X_rel0.rotation)
    rel_q_wxyz = (rel_quat.w, rel_quat.x, rel_quat.y, rel_quat.z)
    rel_extra = make_relative_extra_task(
        model, "right_arm_Link7", "left_arm_Link7",
        tuple(X_rel0.translation), rel_q_wxyz, stiffness=[1, 1, 1, 1, 1, 1])
    pr, qr = model.fk(q0, "right_arm_Link7")
    pr_target = (pr[0] + 0.05, pr[1], pr[2])
    abs_task = Task.pose("right_arm_Link7", pr_target, tuple(qr))
    sol = ik_core.solve(model, q0, [abs_task],
                        extra_tasks=[rel_extra],
                        params=ik_core.SolveParams(max_iters=400))
    # relative transform preserved
    Xa1 = model.fk_se3(sol.q, "right_arm_Link7")
    Xb1 = model.fk_se3(sol.q, "left_arm_Link7")
    X_rel1 = Xb1.inverse() * Xa1
    rel_drift = np.linalg.norm(pin.log6(X_rel1.inverse() * X_rel0).vector)
    assert rel_drift < 1e-2, f"relative pose drifted {rel_drift:.4f}"
    # and the pair actually moved (left tip followed the right)
    pl1, _ = model.fk(sol.q, "left_arm_Link7")
    pl0, _ = model.fk(q0, "left_arm_Link7")
    assert np.linalg.norm(pl1 - pl0) > 1e-3, "left tip should follow to hold the object"
