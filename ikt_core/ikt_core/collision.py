"""Coarse self-collision soft penalty (E2).

Approximates arm/torso links as line-segment **capsules** and adds a smooth
repulsive cost between non-adjacent capsule pairs whenever their surface distance
drops below ``min_distance``. The penalty's gradient folds into the solver's
Gauss-Newton RHS as an extra task (so it just discourages bimanual self-contact);
it is **soft and never a guarantee** — use MoveIt for collision-free planning.

Off by default. The capsule set is derived from a few named link frames + radii
(robot-agnostic; configure per robot). Penalty for a pair at surface distance d:

    c(d) = 1/2 * k * max(0, min_distance - d)^2

Its negative gradient w.r.t. q (via the witness-point frame Jacobians) is returned
as a single stacked row the solver treats like any other task error.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple

import numpy as np

from .robot_model import RobotModel


@dataclass
class Capsule:
    """A capsule = segment from frame ``a`` origin to frame ``b`` origin, radius r."""
    frame_a: str
    frame_b: str
    radius: float


def _closest_points_segments(p1, q1, p2, q2):
    """Closest points between segments [p1,q1] and [p2,q2]. Returns (c1, c2)."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)
    eps = 1e-9
    if a <= eps and e <= eps:
        return p1.copy(), p2.copy()
    if a <= eps:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(d1 @ d2)
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)
    return p1 + d1 * s, p2 + d2 * t


def make_self_collision_extra_task(
    model: RobotModel,
    capsules: Sequence[Capsule],
    min_distance: float,
    weight: float,
) -> Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Extra-task callable adding one repulsion row per violating capsule pair.

    The row error drives the witness points apart along their separation axis;
    its Jacobian is the projection of the relative point-Jacobian on that axis.
    Non-violating pairs contribute a zero row (kept for a fixed row count).
    """
    pairs: List[Tuple[int, int]] = []
    for i in range(len(capsules)):
        for j in range(i + 1, len(capsules)):
            ci, cj = capsules[i], capsules[j]
            # skip pairs sharing a frame (adjacent links)
            shared = {ci.frame_a, ci.frame_b} & {cj.frame_a, cj.frame_b}
            if not shared:
                pairs.append((i, j))

    def _point_jacobian(q, frame):
        # translation rows of the frame Jacobian (3 x nq)
        return model.frame_jacobian(q, frame)[:3, :]

    def extra(q: np.ndarray):
        rows_e: List[float] = []
        rows_J: List[np.ndarray] = []
        for (i, j) in pairs:
            ci, cj = capsules[i], capsules[j]
            pa, _ = model.fk(q, ci.frame_a)
            qa, _ = model.fk(q, ci.frame_b)
            pb, _ = model.fk(q, cj.frame_a)
            qb, _ = model.fk(q, cj.frame_b)
            c1, c2 = _closest_points_segments(pa, qa, pb, qb)
            axis = c1 - c2
            dist = float(np.linalg.norm(axis)) - ci.radius - cj.radius
            if dist >= min_distance or np.linalg.norm(axis) < 1e-9:
                rows_e.append(0.0)
                rows_J.append(np.zeros(model.nq))
                continue
            n = axis / np.linalg.norm(axis)
            # error: how far inside the margin (positive => push apart)
            err = (min_distance - dist)
            # approximate witness-point Jacobians by the nearer segment endpoints
            Ja = _point_jacobian(q, ci.frame_a if np.linalg.norm(c1 - pa)
                                 <= np.linalg.norm(c1 - qa) else ci.frame_b)
            Jb = _point_jacobian(q, cj.frame_a if np.linalg.norm(c2 - pb)
                                 <= np.linalg.norm(c2 - qb) else cj.frame_b)
            # d(dist)/dq ~ n^T (Ja - Jb); we want to INCREASE dist, so error row
            # gradient is -n^T(Ja-Jb). Stack as (err, jac).
            J_row = -(n @ (Ja - Jb))
            rows_e.append(err)
            rows_J.append(J_row)
        if not rows_e:
            return np.zeros(0), np.zeros((0, model.nq)), np.zeros(0)
        e = np.asarray(rows_e)
        J = np.vstack(rows_J)
        s = np.full(e.size, float(weight))
        return e, J, s

    return extra
