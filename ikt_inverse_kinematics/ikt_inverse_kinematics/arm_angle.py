"""S-R-S arm-angle (psi) computation + desired-psi task (requirement R6).

For a 7-DOF spherical-revolute-spherical (S-R-S) arm the Jacobian null space is
1-D: the elbow *swivel* about the shoulder->wrist axis, parametrised by the arm
angle psi. This module:

  * computes psi from the current configuration (always reportable), and
  * provides a desired-psi soft task (scalar error + 1xnq Jacobian) that the
    solver stacks like any other task, so the redundant DOF becomes
    *controllable* without disturbing the 6-DOF pose.

This is the explicit prototype of the FZI ForwardDynamicsSolver null-space fix:
keep the math clean and Eigen-translatable. The psi Jacobian here is computed by
finite differences over the analytic psi(q) — exact enough for the soft task and
trivial to port; a closed-form d(psi)/dq can replace it later if needed.

Definition of psi
-----------------
Let S, E, W be the shoulder, elbow and wrist frame origins (from FK). The plane
through S-E-W has normal proportional to (E - S) x (W - S). psi is the signed
angle of that plane about the shoulder->wrist axis, measured from a reference
plane defined by a fixed world "up" vector projected perpendicular to the axis.
When the arm is near-straight (E on the S-W line) psi is undefined; the caller
falls back to the posture (Wq) term.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .robot_model import RobotModel


@dataclass
class SRSChain:
    """Frame names that define one S-R-S chain for arm-angle computation."""

    name: str
    shoulder: str
    elbow: str
    wrist: str
    base: str = ""
    tip: str = ""


_UP = np.array([0.0, 0.0, 1.0])
_STRAIGHT_TOL = 1e-3   # |elbow offset from S-W axis| below this => psi undefined


def _plane_normal(S: np.ndarray, E: np.ndarray, W: np.ndarray
                  ) -> Tuple[np.ndarray, float, np.ndarray]:
    """Return (axis_unit, elbow_perp_dist, normal_unit) for the S-E-W triangle."""
    axis = W - S
    axis_n = float(np.linalg.norm(axis))
    if axis_n < 1e-9:
        return _UP.copy(), 0.0, _UP.copy()
    axis_u = axis / axis_n
    # elbow vector from shoulder, component perpendicular to the axis
    se = E - S
    perp = se - np.dot(se, axis_u) * axis_u
    perp_n = float(np.linalg.norm(perp))
    if perp_n < 1e-12:
        return axis_u, 0.0, axis_u
    normal = perp / perp_n
    return axis_u, perp_n, normal


def compute_psi(model: RobotModel, q: np.ndarray, chain: SRSChain
                ) -> Optional[float]:
    """Arm angle psi (rad) for ``chain`` at ``q``; None if near-straight/undefined."""
    S, _ = model.fk(q, chain.shoulder)
    E, _ = model.fk(q, chain.elbow)
    W, _ = model.fk(q, chain.wrist)
    axis_u, perp_dist, normal = _plane_normal(np.asarray(S), np.asarray(E),
                                              np.asarray(W))
    if perp_dist < _STRAIGHT_TOL:
        return None
    # reference direction: world up projected perpendicular to the axis. If the
    # axis is ~parallel to up, use world x instead.
    ref = _UP - np.dot(_UP, axis_u) * axis_u
    if np.linalg.norm(ref) < 1e-6:
        ref = np.array([1.0, 0.0, 0.0]) - axis_u[0] * axis_u
    ref /= (np.linalg.norm(ref) + 1e-12)
    # signed angle from ref to normal about axis_u
    cos_a = float(np.clip(np.dot(ref, normal), -1.0, 1.0))
    sin_a = float(np.dot(axis_u, np.cross(ref, normal)))
    return float(np.arctan2(sin_a, cos_a))


def psi_jacobian(model: RobotModel, q: np.ndarray, chain: SRSChain,
                 eps: float = 1e-6) -> Tuple[Optional[float], np.ndarray]:
    """Return (psi, d psi/d q as 1xnq) by finite differences; psi None if undefined.

    Uses the shortest-arc difference so the +/-pi wrap never injects a spurious
    huge gradient.
    """
    psi0 = compute_psi(model, q, chain)
    nq = model.nq
    J = np.zeros((1, nq))
    if psi0 is None:
        return None, J
    for i in range(nq):
        dq = np.zeros(nq)
        dq[i] = eps
        psi1 = compute_psi(model, q + dq, chain)
        if psi1 is None:
            continue
        dpsi = psi1 - psi0
        # wrap to [-pi, pi]
        dpsi = (dpsi + np.pi) % (2.0 * np.pi) - np.pi
        J[0, i] = dpsi / eps
    return psi0, J


def make_arm_angle_extra_task(model: RobotModel, chain: SRSChain,
                              psi_des: float, stiffness: float):
    """Build an extra-task callable for ik_core.solve (scalar psi task).

    The returned callable, given q, yields (error[1], jac[1xnq], stiffness[1])
    where error = shortest-arc (psi_des - psi(q)). If psi is undefined at q
    (near-straight), the task contributes nothing that step.
    """

    def extra(q: np.ndarray):
        psi, J = psi_jacobian(model, q, chain)
        if psi is None:
            return np.zeros(1), np.zeros((1, model.nq)), np.zeros(1)
        err = (float(psi_des) - psi + np.pi) % (2.0 * np.pi) - np.pi
        return np.array([err]), J, np.array([float(stiffness)])

    return extra


def compute_all_psi(model: RobotModel, q: np.ndarray,
                    chains: Dict[str, SRSChain]) -> Dict[str, float]:
    """psi for every chain (skips undefined ones). For ~/status reporting."""
    out: Dict[str, float] = {}
    for name, chain in chains.items():
        psi = compute_psi(model, q, chain)
        if psi is not None:
            out[name] = psi
    return out
