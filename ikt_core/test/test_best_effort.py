"""Offline tests backing the commander's best-effort + stiffness presets (R5).

These exercise the *solver* behaviours the commander relies on: an unreachable
target yields a box-constrained closest config with ``reachable=false`` (the
"stretch toward it" pose, Appendix A), and a position-only task reaches the
point with orientation free. No ROS — pure ``ikt_core``.
"""

import numpy as np

from ikt_core import ik_core
from ikt_core.tasks import Task


def _tip(model):
    return model.link_frame_names()[-1]


def _rand_q_in_limits(model, rng, scale=0.8):
    lo, hi = model.joint_limits()
    q = model.neutral()
    for jn in model.joint_names:
        i = model.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        mid = 0.5 * (a + b)
        half = 0.5 * (b - a) * scale
        q[i] = mid + rng.uniform(-half, half)
    return q


def test_far_target_best_effort_within_limits(arm6_model):
    """A target far beyond reach -> reachable=false, closest config in the box."""
    m = arm6_model
    tip = _tip(m)
    lo, hi = m.joint_limits()
    seed = m.neutral()
    sol = ik_core.solve(m, seed, [Task.point(tip, (0.0, 0.0, 2.0))],
                        active_joints=m.joint_names)
    assert sol.reachable is False
    # box-constrained: every finite-limit joint stays within its range
    for jn in m.joint_names:
        i = m.q_index(jn)
        if np.isfinite(lo[i]) and np.isfinite(hi[i]):
            assert lo[i] - 1e-6 <= sol.q[i] <= hi[i] + 1e-6
    # genuinely unreachable -> a large residual (the arm is extended toward it)
    assert sol.max_pos_err() > 0.5


def test_best_effort_is_stable_for_repeated_far_targets(arm6_model):
    """Re-solving the same unreachable target from its own result is a fixed
    point (no limit-cycle) -> the commander's deadband will hold it steady."""
    m = arm6_model
    tip = _tip(m)
    target = (0.0, 0.0, 2.0)
    sol1 = ik_core.solve(m, m.neutral(), [Task.point(tip, target)],
                         active_joints=m.joint_names)
    sol2 = ik_core.solve(m, sol1.q, [Task.point(tip, target)],
                         active_joints=m.joint_names)
    assert np.linalg.norm(sol2.q - sol1.q) < 1e-3


def test_position_only_reaches_point(arm6_model):
    """position_only (Task.point) reaches a reachable position, orientation free."""
    m = arm6_model
    tip = _tip(m)
    rng = np.random.default_rng(1)
    q_true = _rand_q_in_limits(m, rng)
    p_t, _ = m.fk(q_true, tip)
    seed = q_true + 0.1 * rng.standard_normal(m.nq)
    lo, hi = m.joint_limits()
    seed = np.clip(seed, lo, hi)
    sol = ik_core.solve(m, seed, [Task.point(tip, tuple(p_t))],
                        active_joints=m.joint_names)
    assert sol.reachable
    assert sol.max_pos_err() < 2e-3
