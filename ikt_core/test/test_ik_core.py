"""ik_core.py weighted LM-DLS solver tests on the bundled dual_arm URDF.

Headline: the FK->IK round-trip (>=95% success). Plus dual-arm, active-joint
masking, position-only stiffness, joint limits and reachability verdicts. All
offline — no ROS, no xacro.
"""

import numpy as np

from ikt_core import ik_core
from ikt_core.tasks import Task, Reason

_RIGHT = [f"right_arm_joint{i}" for i in range(1, 7)]
_LEFT = [f"left_arm_joint{i}" for i in range(1, 7)]
_R_TIP = "right_arm_tool0"
_L_TIP = "left_arm_tool0"


def _rand_q_in_limits(model, rng, joints, scale=0.85):
    """Random q with the named joints set within a fraction of their range."""
    lo, hi = model.joint_limits()
    q = np.zeros(model.nq)
    for jn in joints:
        i = model.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        mid = 0.5 * (a + b)
        half = 0.5 * (b - a) * scale
        q[i] = mid + rng.uniform(-half, half)
    return q


def test_round_trip_right_arm(dual_model):
    model = dual_model
    rng = np.random.default_rng(42)
    frame = _R_TIP
    params = ik_core.SolveParams(max_iters=200)
    lo, hi = model.joint_limits()
    n = 60
    ok = 0
    for _ in range(n):
        q_true = _rand_q_in_limits(model, rng, _RIGHT)
        p_t, quat_t = model.fk(q_true, frame)
        # Seed from a perturbation of the true config — this mirrors the
        # library's intended use (IK seeded from the current/previous pose for
        # minimal change) and is the standard FK->IK round-trip check. A
        # non-redundant 6-DOF arm is not expected to globally converge from an
        # arbitrary far posture.
        q_seed = q_true.copy()
        for jn in _RIGHT:
            q_seed[model.q_index(jn)] += rng.uniform(-0.25, 0.25)
        q_seed = np.clip(q_seed, lo, hi)
        task = Task.pose(frame, tuple(p_t), tuple(quat_t))
        sol = ik_core.solve(model, q_seed, [task], params=params,
                            active_joints=_RIGHT)
        if sol.reachable and sol.max_pos_err() <= 1.5e-3 \
                and sol.max_ori_err() <= 5e-3:
            ok += 1
    rate = ok / n
    assert rate >= 0.95, f"round-trip success {rate:.2%} < 95%"


def test_seed_is_minimal_change(dual_model):
    """Seeding from near q_true should converge with small joint motion."""
    model = dual_model
    rng = np.random.default_rng(7)
    frame = _R_TIP
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, quat_t = model.fk(q_true, frame)
    q_seed = q_true + 0.05 * rng.standard_normal(model.nq)
    sol = ik_core.solve(model, q_seed, [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                        active_joints=_RIGHT)
    assert sol.reachable
    assert sol.delta_norm() < 0.5


def test_dual_arm_simultaneous(dual_model):
    model = dual_model
    rng = np.random.default_rng(3)
    lo, hi = model.joint_limits()
    q_true = _rand_q_in_limits(model, rng, _RIGHT + _LEFT)
    pr, qr = model.fk(q_true, _R_TIP)
    pl, ql = model.fk(q_true, _L_TIP)
    # seed near the true config (minimal-change usage), perturbed
    q_seed = q_true + 0.2 * rng.standard_normal(model.nq)
    q_seed = np.clip(q_seed, lo, hi)
    tasks = [Task.pose(_R_TIP, tuple(pr), tuple(qr)),
             Task.pose(_L_TIP, tuple(pl), tuple(ql))]
    sol = ik_core.solve(model, q_seed, tasks,
                        params=ik_core.SolveParams(max_iters=300))
    assert sol.reachable, f"dual-arm reason={sol.reason}"
    assert sol.max_pos_err() <= 2e-3


def test_right_target_does_not_move_left(dual_model):
    """A right-arm-only active set must leave left-arm joints untouched."""
    model = dual_model
    rng = np.random.default_rng(11)
    q_seed = _rand_q_in_limits(model, rng, _RIGHT + _LEFT, scale=0.3)
    frame = _R_TIP
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, quat_t = model.fk(q_true, frame)
    sol = ik_core.solve(model, q_seed, [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                        active_joints=_RIGHT)
    for jn in _LEFT:
        i = model.q_index(jn)
        assert abs(sol.q[i] - q_seed[i]) < 1e-9, f"{jn} moved"


def test_position_only_lets_orientation_float(dual_model):
    """A position-only task nails position with orientation free."""
    model = dual_model
    rng = np.random.default_rng(5)
    frame = _R_TIP
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, _ = model.fk(q_true, frame)
    task = Task.point(frame, tuple(p_t))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.5)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert sol.reachable
    assert sol.max_pos_err() <= 1.5e-3


def test_joint_limits_respected(dual_model):
    """Solutions never violate the box, even for an out-of-reach target."""
    model = dual_model
    rng = np.random.default_rng(9)
    frame = _R_TIP
    lo, hi = model.joint_limits()
    task = Task.pose(frame, (2.0, 2.0, 2.0), (1.0, 0.0, 0.0, 0.0))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.2)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert np.all(sol.q >= lo - 1e-9) and np.all(sol.q <= hi + 1e-9)
    assert not sol.reachable
    assert sol.reason in (Reason.JOINT_LIMIT, Reason.SINGULAR,
                          Reason.TASK_CONFLICT, Reason.MAX_ITERS)


def test_unreachable_reports_reason_and_closest(dual_model):
    model = dual_model
    rng = np.random.default_rng(13)
    frame = _R_TIP
    task = Task.pose(frame, (3.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.2)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert not sol.reachable
    assert sol.reason != Reason.OK
    assert np.all(np.isfinite(sol.q))
    assert sol.max_pos_err() > 0.0
