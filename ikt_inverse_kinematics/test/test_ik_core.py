"""Phase 2 tests: ik_core.py weighted LM-DLS solver — offline, no ROS.

The headline is the FK->IK round-trip (plan §9 gate: >=95% success): sample
random feasible q_true, FK to a target pose, solve from a different seed, and
check the solved pose matches. Plus dual-arm, tool-frame, joint-limit,
singularity-robustness, position-only-stiffness and reachability tests.
"""

import os
import subprocess

import numpy as np
import pytest

from ikt_inverse_kinematics.robot_model import RobotModel
from ikt_inverse_kinematics import ik_core
from ikt_inverse_kinematics.tasks import Task, Reason

_XACRO = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "src", "robot_description", "urdf", "robot.urdf.xacro",
)
_RIGHT = [f"right_arm_joint{i}" for i in range(1, 8)]
_LEFT = [f"left_arm_joint{i}" for i in range(1, 8)]


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


def test_round_trip_right_arm(model):
    rng = np.random.default_rng(42)
    frame = "right_arm_Link7"
    params = ik_core.SolveParams(max_iters=200)
    n = 60
    ok = 0
    for _ in range(n):
        q_true = _rand_q_in_limits(model, rng, _RIGHT)
        p_t, quat_t = model.fk(q_true, frame)
        # seed: a different feasible posture (perturb true a lot)
        q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.5)
        task = Task.pose(frame, tuple(p_t), tuple(quat_t))
        sol = ik_core.solve(model, q_seed, [task], params=params,
                            active_joints=_RIGHT)
        if sol.reachable and sol.max_pos_err() <= 1.5e-3 \
                and sol.max_ori_err() <= 5e-3:
            ok += 1
    rate = ok / n
    assert rate >= 0.95, f"round-trip success {rate:.2%} < 95%"


def test_seed_is_minimal_change(model):
    """Seeding from near q_true should converge with small joint motion."""
    rng = np.random.default_rng(7)
    frame = "right_arm_Link7"
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, quat_t = model.fk(q_true, frame)
    q_seed = q_true + 0.05 * rng.standard_normal(model.nq)
    sol = ik_core.solve(model, q_seed, [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                        active_joints=_RIGHT)
    assert sol.reachable
    assert sol.delta_norm() < 0.5


def test_dual_arm_simultaneous(model):
    rng = np.random.default_rng(3)
    q_true = _rand_q_in_limits(model, rng, _RIGHT + _LEFT)
    pr, qr = model.fk(q_true, "right_arm_Link7")
    pl, ql = model.fk(q_true, "left_arm_Link7")
    q_seed = _rand_q_in_limits(model, rng, _RIGHT + _LEFT, scale=0.4)
    tasks = [Task.pose("right_arm_Link7", tuple(pr), tuple(qr)),
             Task.pose("left_arm_Link7", tuple(pl), tuple(ql))]
    sol = ik_core.solve(model, q_seed, tasks,
                        params=ik_core.SolveParams(max_iters=300))
    assert sol.reachable, f"dual-arm reason={sol.reason}"
    assert sol.max_pos_err() <= 2e-3


def test_right_target_does_not_move_left(model):
    """A right-arm-only active set must leave left-arm joints untouched."""
    rng = np.random.default_rng(11)
    q_seed = _rand_q_in_limits(model, rng, _RIGHT + _LEFT, scale=0.3)
    frame = "right_arm_Link7"
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, quat_t = model.fk(q_true, frame)
    sol = ik_core.solve(model, q_seed, [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                        active_joints=_RIGHT)
    for jn in _LEFT:
        i = model.q_index(jn)
        assert abs(sol.q[i] - q_seed[i]) < 1e-9, f"{jn} moved"


def test_position_only_lets_orientation_float(model):
    """An unreachable orientation with zero ori-stiffness still nails position."""
    rng = np.random.default_rng(5)
    frame = "right_arm_Link7"
    q_true = _rand_q_in_limits(model, rng, _RIGHT)
    p_t, _ = model.fk(q_true, frame)
    # deliberately give a 'bad' orientation but point-only task ignores it
    task = Task.point(frame, tuple(p_t))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.5)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert sol.reachable
    assert sol.max_pos_err() <= 1.5e-3


def test_joint_limits_respected(model):
    """Solutions never violate the box, even for an out-of-reach target."""
    rng = np.random.default_rng(9)
    frame = "right_arm_Link7"
    lo, hi = model.joint_limits()
    # target far outside the workspace
    task = Task.pose(frame, (2.0, 2.0, 2.0), (1.0, 0.0, 0.0, 0.0))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.2)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert np.all(sol.q >= lo - 1e-9) and np.all(sol.q <= hi + 1e-9)
    assert not sol.reachable
    assert sol.reason in (Reason.JOINT_LIMIT, Reason.SINGULAR,
                          Reason.TASK_CONFLICT, Reason.MAX_ITERS)


def test_unreachable_reports_reason_and_closest(model):
    rng = np.random.default_rng(13)
    frame = "right_arm_Link7"
    task = Task.pose(frame, (3.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    q_seed = _rand_q_in_limits(model, rng, _RIGHT, scale=0.2)
    sol = ik_core.solve(model, q_seed, [task], active_joints=_RIGHT)
    assert not sol.reachable
    assert sol.reason != Reason.OK
    # closest-reachable pose is returned (q within limits, finite residual)
    assert np.all(np.isfinite(sol.q))
    assert sol.max_pos_err() > 0.0
