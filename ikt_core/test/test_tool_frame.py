"""Offline tests for the virtual tool-frame mechanism (R1 / commander Phase 2).

A tool offset is realised as a fixed virtual frame appended to the control
link's chain (``RobotModel(urdf, virtual_frames=[...])``). Targets then apply to
that offset tip. No ROS — pure ``ikt_core`` on the bundled 6-DOF arm.
"""

import numpy as np

from ikt_core import assets, ik_core
from ikt_core.robot_model import RobotModel
from ikt_core.tasks import Task, VirtualFrame


def _arm6_urdf():
    return assets.load_sample_urdf("arm_6dof")


def test_virtual_tool_frame_shifts_tip_by_offset():
    urdf = _arm6_urdf()
    base = RobotModel(urdf)
    parent = base.link_frame_names()[-1]
    vf = VirtualFrame("ikt_tool", parent, (0.0, 0.0, 0.10), (0.0, 0.0, 0.0))
    m = RobotModel(urdf, virtual_frames=[vf.as_aux_frame()])
    assert m.has_frame("ikt_tool")
    assert "ikt_tool" in m.virtual_frame_names
    q = m.neutral()
    p_parent, _ = m.fk(q, parent)
    p_tool, _ = m.fk(q, "ikt_tool")
    # the tool sits exactly 0.10 m from the parent origin (offset magnitude)
    assert abs(np.linalg.norm(p_tool - p_parent) - 0.10) < 1e-6


def test_tool_frame_inherits_parent_joint_chain():
    urdf = _arm6_urdf()
    base = RobotModel(urdf)
    parent = base.link_frame_names()[-1]
    m = RobotModel(urdf, virtual_frames=[
        {"name": "ikt_tool", "parent": parent,
         "xyz": [0.0, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0]}])
    # the tool tip is moved by exactly the same joints as the parent link
    assert m.supporting_joints("ikt_tool") == base.supporting_joints(parent)


def test_solve_places_tool_tip_on_target():
    urdf = _arm6_urdf()
    base = RobotModel(urdf)
    parent = base.link_frame_names()[-1]
    m = RobotModel(urdf, virtual_frames=[
        {"name": "ikt_tool", "parent": parent,
         "xyz": [0.0, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0]}])
    rng = np.random.default_rng(5)
    lo, hi = m.joint_limits()
    # a reachable tool pose from a random config, then solve from a perturbed seed
    q_true = m.neutral()
    for jn in m.joint_names:
        i = m.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        q_true[i] = 0.5 * (a + b) + 0.3 * (b - a) * rng.uniform(-1, 1)
    p_t, quat_t = m.fk(q_true, "ikt_tool")
    seed = np.clip(q_true + 0.1 * rng.standard_normal(m.nq), lo, hi)
    sol = ik_core.solve(m, seed, [Task.point("ikt_tool", tuple(p_t))],
                        active_joints=m.joint_names)
    assert sol.reachable
    # the TOOL tip lands on the target; the parent link is 0.10 m away from it
    p_tool, _ = m.fk(sol.q, "ikt_tool")
    p_parent, _ = m.fk(sol.q, parent)
    assert np.linalg.norm(p_tool - p_t) < 2e-3
    assert abs(np.linalg.norm(p_tool - p_parent) - 0.10) < 1e-6
