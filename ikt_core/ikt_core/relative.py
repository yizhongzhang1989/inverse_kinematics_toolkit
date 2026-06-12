"""Dual-arm relative-pose task (requirement R9).

Constrains the transform between two tips so both arms can rigidly hold one
object. The relative error drives ``X_b^{-1} X_a`` toward a target relative
pose ``X_rel``; its Jacobian is the inter-tip differential ``J_a - Ad * J_b``.

Provided as an extra-task callable for ``ik_core.solve`` (6 stacked rows with
their own per-DOF stiffness), so it composes with absolute pose tasks: e.g. one
absolute task moves the leader tip while the relative task keeps the pair rigid.
"""

from __future__ import annotations

from typing import Callable, Tuple

import numpy as np
import pinocchio as pin

from .robot_model import RobotModel, se3_from_xyz_quat


def make_relative_extra_task(
    model: RobotModel,
    frame_a: str,
    frame_b: str,
    target_rel_xyz,
    target_rel_quat_wxyz,
    stiffness,
) -> Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Return an extra-task callable computing the inter-tip relative error/Jac.

    error = log6( (X_b^{-1} X_a)^{-1} * X_rel_target )  (6-vector, lin then ang)
    jac   = relative spatial Jacobian of X_b^{-1} X_a w.r.t. q.
    """
    X_rel_des = se3_from_xyz_quat(target_rel_xyz, target_rel_quat_wxyz)
    s = np.asarray(stiffness, dtype=float).reshape(-1)
    if s.size != 6:
        raise ValueError("relative-task stiffness must have 6 entries")

    def extra(q: np.ndarray):
        Xa = model.fk_se3(q, frame_a)
        Xb = model.fk_se3(q, frame_b)
        X_rel = Xb.inverse() * Xa                       # current b->a transform
        # error twist that carries current relative pose to the target
        err6 = pin.log6(X_rel.inverse() * X_rel_des).vector
        # relative Jacobian in b's frame: J_a expressed in b - J_b's own motion.
        Ja = model.frame_jacobian(q, frame_a)
        Jb = model.frame_jacobian(q, frame_b)
        # Both Jacobians are LOCAL_WORLD_ALIGNED; the differential of the
        # relative pose is (J_a - J_b) to first order in the world-aligned
        # tangent, which is the correct small-motion inter-tip constraint.
        J_rel = Ja - Jb
        return err6, J_rel, s

    return extra
