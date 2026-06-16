"""Offline tests for the Cartesian safety gate geometry (``ikt_core.safety``).

No ROS, no robot: exercises the clamp/displacement math on the bundled 6-DOF
arm so the commander's 30 cm envelope is proven before any motion.
"""

import numpy as np

from ikt_core.safety import clamp_config_to_sphere, frame_displacement


def _tip(model):
    return model.link_frame_names()[-1]


def test_displacement_zero_at_self(arm6_model):
    m = arm6_model
    tip = _tip(m)
    q0 = m.neutral()
    center, _ = m.fk(q0, tip)
    assert frame_displacement(m, q0, tip, center) < 1e-9


def test_small_step_within_radius_unscaled(arm6_model):
    m = arm6_model
    tip = _tip(m)
    q0 = m.neutral()
    center, _ = m.fk(q0, tip)
    q1 = q0.copy()
    q1[m.q_index(m.joint_names[0])] += 0.02
    d_full = frame_displacement(m, q1, tip, center)
    radius = max(0.30, d_full + 0.10)            # full step is comfortably inside
    q_out, scale, d = clamp_config_to_sphere(m, q0, q1, tip, center, radius)
    assert scale == 1.0
    assert np.allclose(q_out, q1)
    assert d <= radius + 1e-9


def test_far_step_clamped_to_boundary(arm6_model):
    m = arm6_model
    tip = _tip(m)
    q0 = m.neutral()
    center, _ = m.fk(q0, tip)
    lo, hi = m.joint_limits()
    q1 = q0.copy()
    for jn in m.joint_names[:3]:                 # bend the proximal joints a lot
        i = m.q_index(jn)
        a, b = lo[i], hi[i]
        if not (np.isfinite(a) and np.isfinite(b)):
            a, b = -np.pi, np.pi
        q1[i] = 0.5 * (a + b) + 0.20 * (b - a)
    d_full = frame_displacement(m, q1, tip, center)
    assert d_full > 0.05, "test target must move the tip appreciably"
    radius = 0.40 * d_full                        # full step overshoots the sphere
    q_out, scale, d = clamp_config_to_sphere(m, q0, q1, tip, center, radius)
    assert 0.0 < scale < 1.0
    assert d <= radius + 1e-6                      # HARD: never outside the sphere
    assert d >= 0.90 * radius                      # but uses most of the budget
    # independent FK re-check of the returned configuration
    assert frame_displacement(m, q_out, tip, center) <= radius + 1e-6


def test_seed_already_outside_holds(arm6_model):
    m = arm6_model
    tip = _tip(m)
    q0 = m.neutral()
    far_center = m.fk(q0, tip)[0] + np.array([10.0, 0.0, 0.0])  # 10 m away
    q1 = q0.copy()
    q1[m.q_index(m.joint_names[0])] += 0.3
    q_out, scale, d = clamp_config_to_sphere(m, q0, q1, tip, far_center, 0.30)
    assert scale == 0.0
    assert np.allclose(q_out, q0)                  # do not move further out


def test_zero_step_is_noop(arm6_model):
    m = arm6_model
    tip = _tip(m)
    q0 = m.neutral()
    center, _ = m.fk(q0, tip)
    q_out, scale, d = clamp_config_to_sphere(m, q0, q0.copy(), tip, center, 0.30)
    assert np.allclose(q_out, q0)
    assert d < 1e-9
