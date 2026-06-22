"""Time-synchronized joint-space trajectory generation for smooth FPC streaming.

Why this exists
---------------
The previous forward-position-controller (FPC) streamer advanced **each joint
with its own INDEPENDENT trapezoidal velocity profile** toward that joint's IK
goal. Joints with a small angular change reached the goal (and stopped) well
before joints with a large change, so during the move the joint ratios kept
changing: the end-effector did **not** travel directly toward the target, it
curved, and near the goal the tail joints were still settling while re-solved IK
nudged the goal -> the arm "shook" around the target.

The fix (this module) is the principle used by state-of-the-art online
trajectory generators (e.g. Ruckig): **time / phase synchronization**. All
joints advance along the *same* joint-space direction governed by **one** scalar
trapezoidal speed sized for the lead (largest-travel) joint, so:

* the joint-space path is a straight line (phase-synchronized),
* every joint reaches the goal at the **same** instant (no late stragglers,
  no shaking),
* the move automatically slows when the required joint travel is large (e.g.
  when the IK goal demands big joint motion approaching a singularity),
* a moving goal is tracked smoothly (the direction is recomputed each tick and
  the scalar speed carries momentum, so re-aiming is jerk-bounded).

It is pure Python (no ROS, no third-party deps) and deterministic, so it is unit
+ scenario tested in isolation.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence


class SyncedJointTrajectory:
    """Phase-synchronized, acceleration-limited joint trajectory streamer.

    Stateful: call :meth:`step` once per control tick with the current IK goal.
    All joints share one normalized speed profile, so they start and finish
    together and the joint-space path between re-aims is a straight line.

    Parameters
    ----------
    joints:
        Ordered joint names this streamer drives (the controller's full set).
    q0:
        Mapping joint -> initial angle (rad); the stream starts here.
    settle_rad:
        Lead-distance (inf-norm) below which the stream snaps to the goal and
        the speed is zeroed, so the arm parks cleanly without chatter.
    """

    def __init__(self, joints: Sequence[str], q0: Dict[str, float],
                 settle_rad: float = 1e-4) -> None:
        self.joints: List[str] = list(joints)
        self.stream: Dict[str, float] = {j: float(q0[j]) for j in self.joints}
        self.lead_vel: float = 0.0          # scalar speed of the lead coord (rad/s)
        self.settle_rad: float = float(settle_rad)

    def reset(self, q0: Dict[str, float]) -> None:
        """Re-seed the stream position and zero the velocity (on enable)."""
        self.stream = {j: float(q0[j]) for j in self.joints}
        self.lead_vel = 0.0

    def set_position(self, q: Dict[str, float]) -> None:
        """Force the stream position without touching the velocity."""
        for j in self.joints:
            if j in q:
                self.stream[j] = float(q[j])

    def step(self, goal: Dict[str, float], dt: float,
             max_speed: float, max_accel: float) -> List[float]:
        """Advance one tick toward ``goal`` and return the new setpoint list.

        Returns the commanded positions in ``self.joints`` order. All joints
        move by the SAME fraction of their respective remaining deltas, so they
        stay phase-locked (synchronized arrival, straight joint-space segment).

        The shared progress is driven by one scalar speed on the lead
        (largest-travel) coordinate. That speed follows a trapezoidal profile
        (cruise at ``max_speed``, brake by ``sqrt(2*a*d)`` so it stops ON the
        goal) and is bounded so that, every tick, ``|Δspeed| <= max_accel*dt``
        AND the step never overshoots the goal. Because the non-lead joints scale
        by ``delta_i / lead <= 1``, their speed and acceleration are bounded by
        the lead's, so the whole arm respects the limits and lands exactly on the
        goal without overshoot or a settling spike.
        """
        delta = {j: goal[j] - self.stream[j] for j in self.joints}
        # Lead distance = the largest remaining per-joint travel (inf-norm).
        lead = 0.0
        for j in self.joints:
            a = abs(delta[j])
            if a > lead:
                lead = a

        if lead <= 1e-12:
            # Exactly there: hold and rest.
            self.lead_vel = 0.0
            return [self.stream[j] for j in self.joints]

        dv_max = max_accel * dt
        # Brake-curve speed: the largest speed from which, decelerating at
        # ``max_accel`` on a ``dt`` time grid, the lead joint still stops within
        # ``lead``. This is the DISCRETE-time stopping curve
        # ``v = -a*dt/2 + sqrt((a*dt/2)^2 + 2*a*d)`` (the continuous
        # ``sqrt(2*a*d)`` is ~a few % too steep when sampled, which would push the
        # final approach slightly past the acceleration limit). It is used both
        # as the desired speed when braking AND as a hard cap, so the speed never
        # lags the brake schedule and arrives too fast.
        half_adt = 0.5 * dv_max
        v_brake = -half_adt + math.sqrt(half_adt * half_adt
                                        + 2.0 * max_accel * lead)
        if lead <= self.settle_rad:
            # Inside the settle band: ramp the speed down to rest (no position
            # snap, so acceleration stays bounded). The arm parks within
            # ``settle_rad`` of the goal -- a sub-0.1 mrad residual, far below IK
            # and encoder resolution -- and then holds.
            v_des = 0.0
        else:
            # Trapezoidal speed on the lead coordinate: cruise at max_speed, then
            # brake. Capped at lead/dt so a single step never overshoots.
            v_des = min(max_speed, v_brake, lead / dt)
        # Acceleration-limit the scalar lead speed (this bounds every joint's
        # acceleration, since the others scale by delta_i/lead <= 1).
        if v_des > self.lead_vel + dv_max:
            self.lead_vel = self.lead_vel + dv_max
        elif v_des < self.lead_vel - dv_max:
            self.lead_vel = self.lead_vel - dv_max
        else:
            self.lead_vel = v_des
        # Hard-cap onto the brake curve so the final approach decelerates smoothly
        # and the joints never arrive faster than they can stop.
        if self.lead_vel > v_brake:
            self.lead_vel = v_brake
        if self.lead_vel < 0.0:
            self.lead_vel = 0.0

        # One shared fraction of progress => all joints stay synchronized along a
        # straight joint-space segment and reach the goal together.
        frac = self.lead_vel * dt / lead
        if frac > 1.0:
            frac = 1.0
        for j in self.joints:
            self.stream[j] += delta[j] * frac
        return [self.stream[j] for j in self.joints]
