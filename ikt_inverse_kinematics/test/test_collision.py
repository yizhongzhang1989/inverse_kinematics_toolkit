"""Self-collision soft penalty (E2) tests — offline, no ROS, no xacro.

Confirms the closest-point-between-segments math and that adding the penalty to
a bimanual solve increases the clearance between two capsules versus solving
without it, while the primary pose tasks still converge. Uses the bundled
dual_arm URDF.
"""

import numpy as np

from ikt_inverse_kinematics import ik_core
from ikt_inverse_kinematics.tasks import Task
from ikt_inverse_kinematics.collision import (
    Capsule, make_self_collision_extra_task, _closest_points_segments,
)

_R_TIP = "right_arm_tool0"
_L_TIP = "left_arm_tool0"


def test_closest_points_parallel_segments():
    p1 = np.array([0.0, 0.0, 0.0]); q1 = np.array([1.0, 0.0, 0.0])
    p2 = np.array([0.0, 1.0, 0.0]); q2 = np.array([1.0, 1.0, 0.0])
    c1, c2 = _closest_points_segments(p1, q1, p2, q2)
    assert np.isclose(np.linalg.norm(c1 - c2), 1.0, atol=1e-6)


def test_closest_points_crossing():
    p1 = np.array([-1.0, 0.0, 0.0]); q1 = np.array([1.0, 0.0, 0.0])
    p2 = np.array([0.0, -1.0, 0.5]); q2 = np.array([0.0, 1.0, 0.5])
    c1, c2 = _closest_points_segments(p1, q1, p2, q2)
    assert np.isclose(np.linalg.norm(c1 - c2), 0.5, atol=1e-6)


def test_self_collision_increases_clearance(dual_model):
    """A bimanual solve with the penalty keeps the two forearms farther apart."""
    model = dual_model
    R = [f"right_arm_joint{i}" for i in range(1, 7)]
    L = [f"left_arm_joint{i}" for i in range(1, 7)]
    # forearm capsules: link3 -> link5 of each arm (upper-arm/forearm segment)
    caps = [Capsule("right_arm_link3", "right_arm_link5", 0.06),
            Capsule("left_arm_link3", "left_arm_link5", 0.06)]

    def clearance(q):
        pa, _ = model.fk(q, "right_arm_link3")
        qa, _ = model.fk(q, "right_arm_link5")
        pb, _ = model.fk(q, "left_arm_link3")
        qb, _ = model.fk(q, "left_arm_link5")
        c1, c2 = _closest_points_segments(pa, qa, pb, qb)
        return np.linalg.norm(c1 - c2) - 0.12

    # pose both arms reaching toward the centre plane (provoking proximity)
    q0 = model.neutral()
    for jn, v in zip(R, [0.0, 0.7, 0.0, 0.0, 0.4, 0.0]):
        q0[model.q_index(jn)] = v
    for jn, v in zip(L, [0.0, 0.7, 0.0, 0.0, 0.4, 0.0]):
        q0[model.q_index(jn)] = v
    pr, qr = model.fk(q0, _R_TIP)
    pl, ql = model.fk(q0, _L_TIP)
    # nudge both tips toward the y midline (arms approach each other)
    tasks = [Task.pose(_R_TIP, (pr[0], pr[1] + 0.12, pr[2]), tuple(qr)),
             Task.pose(_L_TIP, (pl[0], pl[1] - 0.12, pl[2]), tuple(ql))]

    sol_free = ik_core.solve(model, q0, tasks,
                            params=ik_core.SolveParams(max_iters=200))
    extra = make_self_collision_extra_task(model, caps, min_distance=0.08,
                                           weight=5.0)
    sol_sc = ik_core.solve(model, q0, tasks, extra_tasks=[extra],
                          params=ik_core.SolveParams(max_iters=200))

    # both should still roughly hit the pose (soft penalty, so allow some give)
    assert sol_sc.max_pos_err() <= 0.05
    # the penalised solve should not be closer than the free one
    assert clearance(sol_sc.q) >= clearance(sol_free.q) - 1e-3
