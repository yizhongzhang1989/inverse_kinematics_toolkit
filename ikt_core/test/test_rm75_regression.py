"""Optional regression test against the workspace's real dual-arm RM75 URDF.

This runs ONLY when the consuming workspace's xacro is present (i.e. inside the
RobotControl workspace); it is skipped in a standalone checkout. It guards
against regressions on the real robot the toolkit was developed against.
"""

import os
import subprocess

import numpy as np
import pytest

from ikt_core.robot_model import RobotModel
from ikt_core import ik_core
from ikt_core.tasks import Task

_XACRO = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "src", "robot_description", "urdf", "robot.urdf.xacro",
)
_RIGHT = [f"right_arm_joint{i}" for i in range(1, 8)]


def _load_rm75() -> str:
    xacro = os.path.abspath(_XACRO)
    if not os.path.exists(xacro):
        pytest.skip(f"RM75 xacro not found at {xacro} (standalone checkout)")
    try:
        out = subprocess.run(["xacro", xacro, "use_mock_hardware:=true"],
                             check=True, capture_output=True, text=True)
    except Exception as exc:  # xacro not on PATH
        pytest.skip(f"xacro unavailable: {exc}")
    return out.stdout


@pytest.fixture(scope="module")
def rm75() -> RobotModel:
    return RobotModel(_load_rm75())


def test_rm75_dof_and_round_trip(rm75):
    assert rm75.nq == 14
    rng = np.random.default_rng(0)
    frame = "right_arm_Link7"
    lo, hi = rm75.joint_limits()

    def rand_q():
        q = rm75.neutral()
        for jn in _RIGHT:
            i = rm75.q_index(jn)
            a, b = lo[i], hi[i]
            if not (np.isfinite(a) and np.isfinite(b)):
                a, b = -np.pi, np.pi
            q[i] = 0.5 * (a + b) + 0.5 * (b - a) * 0.7 * rng.uniform(-1, 1)
        return q

    ok = 0
    for _ in range(20):
        q_true = rand_q()
        p_t, quat_t = rm75.fk(q_true, frame)
        q_seed = q_true + 0.1 * rng.standard_normal(rm75.nq)
        q_seed = np.clip(q_seed, lo, hi)
        sol = ik_core.solve(rm75, q_seed,
                            [Task.pose(frame, tuple(p_t), tuple(quat_t))],
                            active_joints=_RIGHT)
        if sol.reachable and sol.max_pos_err() <= 1.5e-3:
            ok += 1
    assert ok / 20 >= 0.9
