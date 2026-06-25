"""Offline tests for the virtual tool-frame offset (R2) that the commander uses.

Proves the mechanism the commander relies on: attaching a fixed virtual frame
``ikt_tool`` at an offset from a parent link shifts the solved tip by exactly
that offset (expressed in the parent's local axes), and IK can target the tool
tip. Pure ``ikt_core`` — no ROS.
"""

import numpy as np

from ikt_core import assets, ik_core
from ikt_core.robot_model import RobotModel, R_from_quat_wxyz
from ikt_core.tasks import Task, VirtualFrame


def _arm6_urdf():
    return assets.load_sample_urdf("arm_6dof")


def _rand_q(model, rng, scale=0.7):
    lo, hi = model.joint_limits()
    q = model.neutral()
    for jn in model.joint_names:
        i = model.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        q[i] = 0.5 * (a + b) + 0.5 * (b - a) * scale * rng.uniform(-1, 1)
    return q


def test_virtual_tool_frame_shifts_tip_by_offset():
    urdf = _arm6_urdf()
    base = RobotModel(urdf)
    parent = base.link_frame_names()[-1]
    offset = (0.0, 0.0, 0.10)
    vf = VirtualFrame("ikt_tool", parent, offset, (0.0, 0.0, 0.0)).as_aux_frame()
    m = RobotModel(urdf, virtual_frames=[vf])
    assert m.has_frame("ikt_tool")
    assert "ikt_tool" in m.virtual_frame_names

    rng = np.random.default_rng(5)
    for _ in range(8):
        q = _rand_q(m, rng)
        p_par, quat_par = m.fk(q, parent)
        p_tool, _ = m.fk(q, "ikt_tool")
        R = R_from_quat_wxyz(quat_par)
        expected = p_par + R @ np.array(offset)
        assert np.linalg.norm(p_tool - expected) < 1e-9


def test_solve_reaches_tool_tip():
    urdf = _arm6_urdf()
    parent = RobotModel(urdf).link_frame_names()[-1]
    vf = VirtualFrame("ikt_tool", parent, (0.0, 0.0, 0.10),
                      (0.0, 0.0, 0.0)).as_aux_frame()
    m = RobotModel(urdf, virtual_frames=[vf])
    rng = np.random.default_rng(6)
    q_true = _rand_q(m, rng, scale=0.5)
    p_t, _ = m.fk(q_true, "ikt_tool")
    seed = np.clip(q_true + 0.1 * rng.standard_normal(m.nq),
                   *m.joint_limits())
    sol = ik_core.solve(m, seed, [Task.point("ikt_tool", tuple(p_t))],
                        active_joints=m.joint_names)
    assert sol.reachable
    assert sol.max_pos_err() < 2e-3


def test_zero_offset_tool_equals_parent():
    """A zero offset places the tool exactly on the parent link."""
    urdf = _arm6_urdf()
    parent = RobotModel(urdf).link_frame_names()[-1]
    vf = VirtualFrame("ikt_tool", parent, (0.0, 0.0, 0.0),
                      (0.0, 0.0, 0.0)).as_aux_frame()
    m = RobotModel(urdf, virtual_frames=[vf])
    q = m.neutral()
    p_par, _ = m.fk(q, parent)
    p_tool, _ = m.fk(q, "ikt_tool")
    assert np.linalg.norm(p_tool - p_par) < 1e-9
