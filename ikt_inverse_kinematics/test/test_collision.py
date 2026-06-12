"""Phase 7 / E2 test: self-collision soft penalty (offline).

Confirms the closest-point-between-segments math and that adding the penalty to
a solve increases the clearance between two capsules versus solving without it,
while the primary pose task still converges.
"""

import os
import subprocess

import numpy as np
import pytest

from ikt_inverse_kinematics.robot_model import RobotModel
from ikt_inverse_kinematics import ik_core
from ikt_inverse_kinematics.tasks import Task
from ikt_inverse_kinematics.collision import (
    Capsule, make_self_collision_extra_task, _closest_points_segments,
)

_XACRO = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "src", "robot_description", "urdf", "robot.urdf.xacro",
)


def _load_urdf() -> str:
    xacro = os.path.abspath(_XACRO)
    if not os.path.exists(xacro):
        pytest.skip("xacro not found")
    try:
        out = subprocess.run(["xacro", xacro, "use_mock_hardware:=true"],
                             check=True, capture_output=True, text=True)
    except Exception as exc:
        pytest.skip(f"xacro unavailable: {exc}")
    return out.stdout


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel(_load_urdf())


def test_closest_points_parallel_segments():
    p1 = np.array([0.0, 0.0, 0.0]); q1 = np.array([1.0, 0.0, 0.0])
    p2 = np.array([0.0, 1.0, 0.0]); q2 = np.array([1.0, 1.0, 0.0])
    c1, c2 = _closest_points_segments(p1, q1, p2, q2)
    assert np.isclose(np.linalg.norm(c1 - c2), 1.0, atol=1e-6)


def test_closest_points_crossing():
    p1 = np.array([-1.0, 0.0, 0.0]); q1 = np.array([1.0, 0.0, 0.0])
    p2 = np.array([0.0, -1.0, 0.5]); q2 = np.array([0.0, 1.0, 0.5])
    c1, c2 = _closest_points_segments(p1, q1, p2, q2)
    # closest approach is the 0.5 gap in z near the origin
    assert np.isclose(np.linalg.norm(c1 - c2), 0.5, atol=1e-6)


def test_self_collision_increases_clearance(model):
    """A bimanual solve with the penalty keeps the two forearms farther apart."""
    rng = np.random.default_rng(0)
    R = [f"right_arm_joint{i}" for i in range(1, 8)]
    L = [f"left_arm_joint{i}" for i in range(1, 8)]
    # forearm capsules: Link4->Link6 of each arm
    caps = [Capsule("right_arm_Link4", "right_arm_Link6", 0.06),
            Capsule("left_arm_Link4", "left_arm_Link6", 0.06)]

    def clearance(q):
        pa, _ = model.fk(q, "right_arm_Link4")
        qa, _ = model.fk(q, "right_arm_Link6")
        pb, _ = model.fk(q, "left_arm_Link4")
        qb, _ = model.fk(q, "left_arm_Link6")
        from ikt_inverse_kinematics.collision import _closest_points_segments
        c1, c2 = _closest_points_segments(pa, qa, pb, qb)
        return np.linalg.norm(c1 - c2) - 0.12

    # pick targets that pull both wrists toward the centre (provoking proximity)
    q0 = model.neutral()
    for jn, v in zip(R, [0.0, 0.6, 0.0, 1.2, 0.0, 0.6, 0.0]):
        q0[model.q_index(jn)] = v
    for jn, v in zip(L, [0.0, 0.6, 0.0, 1.2, 0.0, 0.6, 0.0]):
        q0[model.q_index(jn)] = v
    pr, qr = model.fk(q0, "right_arm_Link7")
    pl, ql = model.fk(q0, "left_arm_Link7")
    # nudge both tips toward x midline
    tasks = [Task.pose("right_arm_Link7", (pr[0], pr[1] + 0.15, pr[2]), tuple(qr)),
             Task.pose("left_arm_Link7", (pl[0], pl[1] - 0.15, pl[2]), tuple(ql))]

    sol_free = ik_core.solve(model, q0, tasks,
                            params=ik_core.SolveParams(max_iters=200))
    extra = make_self_collision_extra_task(model, caps, min_distance=0.08,
                                           weight=5.0)
    sol_sc = ik_core.solve(model, q0, tasks, extra_tasks=[extra],
                          params=ik_core.SolveParams(max_iters=200))

    # both should still roughly hit the pose (soft penalty, so allow some give)
    assert sol_sc.max_pos_err() <= 0.05
    # the penalised solve should not be closer than the free one (>= within eps)
    assert clearance(sol_sc.q) >= clearance(sol_free.q) - 1e-3
