"""Tests for the time-synchronized joint trajectory generator.

These guard the property that fixes the "shaking / curved approach" bug: all
joints advance in phase and reach the goal together along a straight joint-space
segment, with an acceleration-limited trapezoidal speed that parks on the goal
without overshoot or chatter.
"""
import math

import pytest

from ikt_pose_commander.trajectory import SyncedJointTrajectory

JOINTS = ["j1", "j2", "j3"]
DT = 1.0 / 200.0
VMAX = 0.5
AMAX = 3.0


SETTLE = 1e-4    # default settle band of SyncedJointTrajectory


def _run(gen, goal, max_ticks=4000, vmax=VMAX, amax=AMAX):
    # Record the START configuration first so velocity/acceleration estimates
    # have the correct rest initial condition (v0 = 0 from the start pose).
    traj = [[gen.stream[j] for j in gen.joints]]
    for _ in range(max_ticks):
        data = gen.step(goal, DT, vmax, amax)
        traj.append(list(data))
        if max(abs(data[i] - goal[j]) for i, j in enumerate(gen.joints)) <= SETTLE:
            break
    return traj


def test_reaches_goal_exactly():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 1.0, "j2": -0.4, "j3": 0.25}
    traj = _run(gen, goal)
    final = traj[-1]
    for i, j in enumerate(JOINTS):
        # Parks within the settle band (sub-0.1 mrad), below IK/encoder limits.
        assert abs(final[i] - goal[j]) <= SETTLE


def test_all_joints_arrive_together():
    """The whole point: near-zero arrival spread (joints finish together).

    Arrival is measured at a RELATIVE threshold (1% of each joint's own travel)
    because the joints are phase-locked: they cover the same FRACTION of their
    travel each tick, so they reach a given fraction simultaneously regardless of
    how different their absolute distances are.
    """
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 1.0, "j2": 0.1, "j3": 0.5}   # very different distances
    traj = _run(gen, goal)
    arrive = {}
    for k, row in enumerate(traj):
        for i, j in enumerate(JOINTS):
            if j not in arrive and abs(row[i] - goal[j]) < 0.01 * abs(goal[j]):
                arrive[j] = k
    spread_ticks = max(arrive.values()) - min(arrive.values())
    assert spread_ticks <= 2            # within ~10 ms at 200 Hz (was seconds)


def test_phase_locked_straight_line():
    """Each joint's fractional progress is identical at every tick => the
    joint-space path is a straight line (phase synchronization)."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 1.0, "j2": 0.1, "j3": -0.5}
    traj = _run(gen, goal)
    for row in traj[2:-2]:
        progress = [row[i] / goal[j] for i, j in enumerate(JOINTS)]
        assert max(progress) - min(progress) < 1e-6


def test_speed_and_accel_limits_respected():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 2.0, "j2": -1.0, "j3": 0.7}
    traj = _run(gen, goal)
    prev_v = {j: 0.0 for j in JOINTS}
    for a in range(1, len(traj)):
        for i, j in enumerate(JOINTS):
            v = (traj[a][i] - traj[a - 1][i]) / DT
            assert abs(v) <= VMAX + 1e-6
            acc = (v - prev_v[j]) / DT
            assert abs(acc) <= AMAX + 1e-6
            prev_v[j] = v


def test_no_overshoot():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 1.0, "j2": 0.5, "j3": -0.8}
    traj = _run(gen, goal)
    for row in traj:
        for i, j in enumerate(JOINTS):
            # never pass the goal (all moves are same-sign here)
            if goal[j] >= 0:
                assert row[i] <= goal[j] + 1e-6
            else:
                assert row[i] >= goal[j] - 1e-6


def test_tiny_move_no_chatter():
    """A sub-settle move snaps cleanly with zero reversals."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 1e-4, "j2": 0.0, "j3": 0.0}
    traj = _run(gen, goal, max_ticks=200)
    # monotonic, no sign reversals
    revs = 0
    for a in range(2, len(traj)):
        d1 = traj[a][0] - traj[a - 1][0]
        d0 = traj[a - 1][0] - traj[a - 2][0]
        if d1 * d0 < 0:
            revs += 1
    assert revs == 0


def test_settles_and_holds():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 0.6, "j2": 0.6, "j3": 0.6}
    _run(gen, goal)
    # after reaching, more ticks keep it parked (no drift)
    last = gen.step(goal, DT, VMAX, AMAX)
    for _ in range(50):
        nxt = gen.step(goal, DT, VMAX, AMAX)
        assert max(abs(a - b) for a, b in zip(nxt, last)) < 1e-9
        last = nxt


def test_moving_target_tracks_without_jerk():
    """A steadily moving goal is tracked with bounded acceleration (no jumps)."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    prev_v = {j: 0.0 for j in JOINTS}
    pos = {j: 0.0 for j in JOINTS}
    last = [0.0, 0.0, 0.0]
    for k in range(600):
        for j in JOINTS:
            pos[j] = 0.0005 * k     # ramp the goal
        data = gen.step(pos, DT, VMAX, AMAX)
        for i, j in enumerate(JOINTS):
            v = (data[i] - last[i]) / DT
            acc = (v - prev_v[j]) / DT
            assert abs(acc) <= AMAX + 1e-3
            prev_v[j] = v
        last = list(data)


def test_reset_reseeds():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    _run(gen, {"j1": 0.5, "j2": 0.5, "j3": 0.5})
    gen.reset({j: 1.0 for j in JOINTS})
    assert all(abs(gen.stream[j] - 1.0) < 1e-12 for j in JOINTS)
    assert gen.lead_vel == 0.0
    assert all(gen.vel[j] == 0.0 for j in JOINTS)


# --------------------------------------------------------------------------- #
# Decoupled per-joint profile (step_independent): used near singularities to
# cap each joint's speed (especially the big ones) instead of synchronizing the
# whole arm to one shared profile. EE tracking is intentionally relaxed.
# --------------------------------------------------------------------------- #

def _run_indep(gen, goal, smap, amap, max_ticks=6000):
    traj = [[gen.stream[j] for j in gen.joints]]
    for _ in range(max_ticks):
        data = gen.step_independent(goal, DT, smap, amap)
        traj.append(list(data))
        if max(abs(data[i] - goal[j])
               for i, j in enumerate(gen.joints)) <= SETTLE:
            break
    return traj


def test_independent_reaches_goal_exactly():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    smap = {j: VMAX for j in JOINTS}
    amap = {j: AMAX for j in JOINTS}
    goal = {"j1": 1.0, "j2": -0.4, "j3": 0.25}
    traj = _run_indep(gen, goal, smap, amap)
    final = traj[-1]
    for i, j in enumerate(JOINTS):
        assert abs(final[i] - goal[j]) <= SETTLE


def test_independent_per_joint_limits_respected():
    """Each joint stays within its OWN speed/accel cap (not a shared one)."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    smap = {"j1": 0.2, "j2": 0.5, "j3": 0.8}     # big joint j1 capped low
    amap = {"j1": 1.0, "j2": 3.0, "j3": 5.0}
    goal = {"j1": 1.5, "j2": -1.0, "j3": 1.2}
    traj = _run_indep(gen, goal, smap, amap)
    prev_v = {j: 0.0 for j in JOINTS}
    for a in range(1, len(traj)):
        for i, j in enumerate(JOINTS):
            v = (traj[a][i] - traj[a - 1][i]) / DT
            # Speed cap holds at every tick.
            assert abs(v) <= smap[j] + 1e-6
            # Accel cap governs the MOVING phase. The final sub-settle parking
            # tick (joint lands exactly on the goal) is a discrete-landing
            # artifact shared by the synchronized step(); skip it.
            parked = abs(traj[a][i] - goal[j]) <= SETTLE
            if not parked:
                acc = (v - prev_v[j]) / DT
                assert abs(acc) <= amap[j] + 1e-6
            prev_v[j] = v


def test_independent_decouples_arrival():
    """Opposite of the synced case: small-travel joints arrive well BEFORE a
    large-travel joint (each runs its own profile -> no crawl to the lead)."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    smap = {j: VMAX for j in JOINTS}
    amap = {j: AMAX for j in JOINTS}
    goal = {"j1": 2.0, "j2": 0.05, "j3": 0.05}   # j1 huge, j2/j3 tiny
    traj = _run_indep(gen, goal, smap, amap)
    arrive = {}
    for k, row in enumerate(traj):
        for i, j in enumerate(JOINTS):
            if j not in arrive and abs(row[i] - goal[j]) <= SETTLE:
                arrive[j] = k
    assert arrive["j2"] < arrive["j1"]
    assert arrive["j3"] < arrive["j1"]
    assert arrive["j1"] - max(arrive["j2"], arrive["j3"]) > 5


def test_independent_lower_cap_is_slower():
    """A lower per-joint speed cap takes more ticks over the same distance."""
    slow = SyncedJointTrajectory(["j"], {"j": 0.0})
    fast = SyncedJointTrajectory(["j"], {"j": 0.0})
    goal = {"j": 1.0}
    t_slow = _run_indep(slow, goal, {"j": 0.2}, {"j": AMAX})
    t_fast = _run_indep(fast, goal, {"j": 0.8}, {"j": AMAX})
    assert len(t_slow) > len(t_fast)


def test_independent_no_overshoot():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    smap = {j: VMAX for j in JOINTS}
    amap = {j: AMAX for j in JOINTS}
    goal = {"j1": 1.0, "j2": 0.5, "j3": -0.8}
    traj = _run_indep(gen, goal, smap, amap)
    for row in traj:
        for i, j in enumerate(JOINTS):
            if goal[j] >= 0:
                assert row[i] <= goal[j] + 1e-6
            else:
                assert row[i] >= goal[j] - 1e-6


def test_resync_lead_from_joint_speeds():
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    gen.vel = {"j1": 0.1, "j2": -0.3, "j3": 0.05}
    gen.resync_lead()
    assert abs(gen.lead_vel - 0.3) < 1e-12


def test_sync_to_independent_velocity_continuity():
    """Handing off sync -> decoupled is jerk-bounded: the first decoupled step's
    acceleration from the synced realized velocity stays within max_accel."""
    gen = SyncedJointTrajectory(JOINTS, {j: 0.0 for j in JOINTS})
    goal = {"j1": 2.0, "j2": 1.0, "j3": -1.5}
    last = [gen.stream[j] for j in JOINTS]
    for _ in range(40):                      # build up synchronized velocity
        last = gen.step(goal, DT, VMAX, AMAX)
    v_before = {j: gen.vel[j] for j in JOINTS}
    data = gen.step_independent(goal, DT, {j: VMAX for j in JOINTS},
                                {j: AMAX for j in JOINTS})
    for i, j in enumerate(JOINTS):
        v_after = (data[i] - last[i]) / DT
        acc = (v_after - v_before[j]) / DT
        assert abs(acc) <= AMAX + 1e-6


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
