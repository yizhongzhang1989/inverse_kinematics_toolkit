"""Cartesian safety helpers (pure: numpy + ``RobotModel.fk``; no rclpy).

These back the commander's **30 cm Cartesian safety gate**: the controlled
frame may not leave a sphere of radius ``radius`` centred on the pose captured
when motion was enabled (the *start pose*). Keeping the geometry here — free of
rclpy and Pinocchio internals — lets it be unit-tested offline (table of cases)
and shared by the commander and its tests.

Two operations:

* :func:`frame_displacement` — how far the controlled frame is, for a given
  configuration, from the sphere centre (used by the reject path and status).
* :func:`clamp_config_to_sphere` — scale a commanded configuration's joint step
  so the controlled frame lands on/inside the sphere boundary (the FPC clamp).
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from .robot_model import RobotModel


def frame_displacement(model: RobotModel, q: Sequence[float], frame: str,
                       center_xyz: Sequence[float]) -> float:
    """Euclidean distance of ``frame``'s origin (at ``q``) from ``center_xyz``."""
    xyz, _ = model.fk(np.asarray(q, dtype=float), frame)
    return float(np.linalg.norm(xyz - np.asarray(center_xyz, dtype=float).reshape(3)))


def clamp_config_to_sphere(model: RobotModel, q_seed: Sequence[float],
                           q_target: Sequence[float], frame: str,
                           center_xyz: Sequence[float], radius: float,
                           *, iters: int = 48) -> Tuple[np.ndarray, float, float]:
    """Scale the step ``q_seed -> q_target`` so ``frame`` stays within ``radius``.

    Returns ``(q_out, scale, dist)`` where ``q_out = q_seed + scale*(q_target -
    q_seed)``, ``scale in [0, 1]`` and ``dist`` is the resulting displacement of
    ``frame`` from ``center_xyz``.

    * If the full step already keeps ``frame`` within ``radius`` -> ``scale=1``.
    * If the seed itself is already at/over the boundary -> ``scale=0`` (hold;
      never move further out).
    * Otherwise a bisection on the scalar ``scale`` finds the boundary. The EE
      displacement along the joint-space chord is monotone in practice for the
      small steps this gate sees; bisection is robust and cheap regardless.
    """
    nq = model.nq
    q_seed = np.asarray(q_seed, dtype=float).reshape(nq)
    q_target = np.asarray(q_target, dtype=float).reshape(nq)
    center = np.asarray(center_xyz, dtype=float).reshape(3)
    step = q_target - q_seed
    radius = float(radius)

    def dist(scale: float) -> float:
        xyz, _ = model.fk(q_seed + scale * step, frame)
        return float(np.linalg.norm(xyz - center))

    if not np.any(step):
        return q_seed.copy(), 1.0, dist(0.0)

    d_full = dist(1.0)
    if d_full <= radius:
        return q_target.copy(), 1.0, d_full

    if dist(0.0) >= radius:
        # Already on/over the sphere: do not move outward.
        return q_seed.copy(), 0.0, dist(0.0)

    lo, hi = 0.0, 1.0
    for _ in range(int(iters)):
        mid = 0.5 * (lo + hi)
        if dist(mid) <= radius:
            lo = mid
        else:
            hi = mid
    q_out = q_seed + lo * step
    return q_out, lo, dist(lo)
