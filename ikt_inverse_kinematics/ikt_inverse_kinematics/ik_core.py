"""Pure-Python weighted LM-DLS inverse-kinematics solver (no rclpy).

Implements the single unified formulation of IMPLEMENTATION_PLAN.md §5:

    min_q  1/2 sum_k || Wt_k^{1/2} e_k(q) ||^2  +  1/2 || Wq^{1/2} (q - q_rest) ||^2
    s.t.   q_min <= q <= q_max

solved by a damped (Levenberg-Marquardt) Gauss-Newton step projected to the box,
with adaptive damping for singularity robustness and a backtracking line search
for guaranteed monotone decrease. Supports:

  * multiple simultaneous tasks (multi-tip / dual-arm) by stacking Jacobians;
  * per-DOF task stiffness Wt (requirement 4);
  * soft joint centering / rest-posture bias Wq (requirement 3);
  * hard box joint limits (requirement 3);
  * active-joint masking (freeze a joint set — e.g. right-arm-only target);
  * extra scalar task rows (arm-angle psi) and relative-pose tasks injected by
    the caller as generic (error, jacobian) providers;
  * a reachability verdict + reason + blocking joints (requirement 7).

The solver is advisory: it returns a Solution; it never commands anything.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from .robot_model import RobotModel
from .tasks import Reason, Solution, Task, TaskResidual


@dataclass
class SolveParams:
    max_iters: int = 200
    tol_pos: float = 1e-3          # metres
    tol_ori: float = 3.5e-3        # rad
    damping: float = 1e-2          # initial LM mu
    damping_min: float = 1e-6
    damping_max: float = 10.0
    joint_centering_weight: float = 1e-2
    step_scale: float = 1.0
    ls_max_steps: int = 20
    ls_shrink: float = 0.5
    sigma_singular: float = 1e-3   # sigma_min below this => near-singular
    # fraction of a joint's range within which it counts as "blocking" / at-limit
    limit_margin: float = 1e-3


# A generic extra task: given q, return (error_vector_r, jacobian_r_by_nq, stiffness_r).
# Used for arm-angle (r=1) and relative-pose (r=6) tasks computed by the caller.
ExtraTask = Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]


def solve(
    model: RobotModel,
    q_seed: np.ndarray,
    tasks: Sequence[Task],
    *,
    params: Optional[SolveParams] = None,
    rest_posture: Optional[np.ndarray] = None,
    centering_weights: Optional[np.ndarray] = None,
    active_joints: Optional[Sequence[str]] = None,
    extra_tasks: Optional[Sequence[ExtraTask]] = None,
) -> Solution:
    """Solve weighted, damped, box-constrained IK. See module docstring.

    Parameters
    ----------
    model:        the RobotModel.
    q_seed:       starting configuration (length nq). Seeding from the current
                  joint state yields the minimal-change behaviour / continuity.
    tasks:        list of Cartesian Tasks (frame + target + stiffness[6]).
    rest_posture: target for the soft centering term (length nq); default zeros.
    centering_weights: per-joint Wq diagonal (length nq); default
                  params.joint_centering_weight on every joint.
    active_joints: names of joints allowed to move; others are frozen. None=all.
    extra_tasks:  optional callables contributing extra stacked rows (psi, rel).
    """
    p = params or SolveParams()
    nq = model.nq
    q = np.asarray(q_seed, dtype=float).reshape(nq).copy()
    q_seed_arr = q.copy()

    q_min, q_max = model.joint_limits()
    mask = model.active_mask(active_joints)

    if rest_posture is None:
        q_rest = np.zeros(nq)
    else:
        q_rest = np.asarray(rest_posture, dtype=float).reshape(nq).copy()
    # clamp rest posture into the box so the bias never fights the limits
    q_rest = np.clip(q_rest, q_min, q_max)

    if centering_weights is None:
        wq = np.full(nq, p.joint_centering_weight, dtype=float)
    else:
        wq = np.asarray(centering_weights, dtype=float).reshape(nq).copy()

    task_list = list(tasks)
    extra_list = list(extra_tasks or [])

    def build_error_and_jac(q_in: np.ndarray):
        """Stack weighted error We (m,) and weighted Jacobian WJ (m, nq).

        Returns (raw_error, sqrt_weight, We, WJ). Rows are the 6-DOF pose error
        of each task followed by any extra-task rows (arm-angle psi, relative
        pose). Inactive joint columns are zeroed so frozen joints never move.
        """
        errs: List[np.ndarray] = []
        jacs: List[np.ndarray] = []
        sqrt_w: List[np.ndarray] = []
        for t in task_list:
            e = model.pose_error(q_in, t.frame,
                                 t.target_xyz_vec(), t.target_quat_vec())
            J = model.frame_jacobian(q_in, t.frame)
            errs.append(e)
            jacs.append(J)
            sqrt_w.append(np.sqrt(t.stiffness_vec()))
        for ex in extra_list:
            e_r, J_r, s_r = ex(q_in)
            e_r = np.asarray(e_r, dtype=float).reshape(-1)
            J_r = np.asarray(J_r, dtype=float).reshape(e_r.size, nq)
            s_r = np.asarray(s_r, dtype=float).reshape(-1)
            errs.append(e_r)
            jacs.append(J_r)
            sqrt_w.append(np.sqrt(s_r))
        if errs:
            e_all = np.concatenate(errs)
            J_all = np.vstack(jacs)
            w_all = np.concatenate(sqrt_w)
        else:
            e_all = np.zeros(0)
            J_all = np.zeros((0, nq))
            w_all = np.zeros(0)
        We = w_all * e_all
        WJ = (w_all[:, None]) * J_all
        WJ[:, ~mask] = 0.0
        return e_all, w_all, We, WJ

    def task_error_norm(q_in: np.ndarray) -> float:
        """Weighted task-error norm (excludes the posture bias) for line search."""
        _, _, We, _ = build_error_and_jac(q_in)
        return float(np.linalg.norm(We))

    mu = float(p.damping)
    err_norm = task_error_norm(q)
    iters = 0

    for it in range(p.max_iters):
        iters = it

        pos_err, ori_err = _primary_residual(model, q, task_list)
        pose_ok = pos_err <= p.tol_pos and ori_err <= p.tol_ori
        # Early exit only when the pose is satisfied AND there is no soft extra
        # task (arm-angle psi / relative pose) still to be resolved. With extra
        # tasks present we keep iterating so the redundant DOF settles; the loop
        # then terminates on "no further improvement" below.
        if pose_ok and not extra_list:
            return _finalize(model, q, q_seed_arr, task_list, extra_list,
                             True, Reason.OK, it, p, q_min, q_max, mask)

        e_all, w_all, We, WJ = build_error_and_jac(q)
        m_rows = We.size

        # Damped least-squares task step (primary): solve in the smaller of
        # joint/task space. dq_task = (J^T J + mu I)^-1 J^T e   (weighted).
        H = WJ.T @ WJ + mu * np.eye(nq)
        try:
            Hinv_Jt = np.linalg.solve(H, WJ.T)          # nq x m
        except np.linalg.LinAlgError:
            mu = min(mu * 10.0, p.damping_max)
            continue
        dq_task = Hinv_Jt @ We                          # nq

        # Soft posture / rest bias projected into the task NULL SPACE so it can
        # never degrade the task (requirement 3 "prefer 0", minimal-change).
        dq_rest = (q_rest - q)
        dq_rest[~mask] = 0.0
        if m_rows > 0:
            N = np.eye(nq) - Hinv_Jt @ WJ               # nq x nq null projector
            dq_null = N @ (wq * dq_rest)
        else:
            dq_null = wq * dq_rest
        dq_task[~mask] = 0.0
        dq_null[~mask] = 0.0

        # Backtracking line search on the task error, projected onto the box.
        # The combined (task + posture) step is tried first; if it fails to
        # reduce the task error, the posture part is dropped and the task-only
        # step is tried, so the secondary posture bias can NEVER stall task
        # convergence (this is what lets the last millimetre close).
        improved = False
        prev_norm = err_norm
        for dq in (p.step_scale * (dq_task + dq_null), dq_task):
            alpha = 1.0
            for _ in range(p.ls_max_steps):
                q_try = np.clip(q + alpha * dq, q_min, q_max)
                n_try = task_error_norm(q_try)
                if n_try < err_norm:
                    q = q_try
                    err_norm = n_try
                    improved = True
                    break
                alpha *= p.ls_shrink
            if improved:
                break

        if improved:
            mu = max(mu * 0.7, p.damping_min)
            # Converged-by-no-progress: the accepted step barely changed the
            # stacked error (pose hit, redundant DOF settled). Stop cleanly.
            if (prev_norm - err_norm) <= 1e-9 * (1.0 + prev_norm):
                pose_ok = (pos_err <= p.tol_pos and ori_err <= p.tol_ori)
                return _finalize(
                    model, q, q_seed_arr, task_list, extra_list, pose_ok,
                    Reason.OK if pose_ok else _why_stuck(
                        model, q, task_list, q_min, q_max, p),
                    it, p, q_min, q_max, mask)
        else:
            # No task improvement available. If the pose is already within tol,
            # we're done (any residual is in the soft extra/posture term).
            if pose_ok:
                return _finalize(model, q, q_seed_arr, task_list, extra_list,
                                 True, Reason.OK, it, p, q_min, q_max, mask)
            mu = min(mu * 2.0, p.damping_max)
            if mu >= p.damping_max:
                return _finalize(model, q, q_seed_arr, task_list, extra_list,
                                 False, _why_stuck(model, q, task_list, q_min,
                                                   q_max, p),
                                 it, p, q_min, q_max, mask)

    # ran out of iterations
    pos_err, ori_err = _primary_residual(model, q, task_list)
    ok = pos_err <= p.tol_pos and ori_err <= p.tol_ori
    reason = Reason.OK if ok else _why_stuck(model, q, task_list, q_min, q_max, p)
    return _finalize(model, q, q_seed_arr, task_list, extra_list, ok,
                     reason if not ok else Reason.OK, iters, p,
                     q_min, q_max, mask, max_iters_hit=not ok)


def _primary_residual(model: RobotModel, q: np.ndarray,
                      tasks: Sequence[Task]) -> Tuple[float, float]:
    """Worst-case raw position (m) and orientation (rad) error over pose tasks.

    DOFs with zero stiffness are excluded from the residual (the user declared
    they don't matter), so e.g. a position-only task isn't failed on orientation.
    """
    pos = 0.0
    ori = 0.0
    for t in tasks:
        e = model.pose_error(q, t.frame, t.target_xyz_vec(), t.target_quat_vec())
        s = t.stiffness_vec()
        pe = np.linalg.norm(e[:3][s[:3] > 0.0]) if np.any(s[:3] > 0) else 0.0
        oe = np.linalg.norm(e[3:][s[3:] > 0.0]) if np.any(s[3:] > 0) else 0.0
        pos = max(pos, float(pe))
        ori = max(ori, float(oe))
    return pos, ori


def _why_stuck(model: RobotModel, q: np.ndarray, tasks: Sequence[Task],
               q_min: np.ndarray, q_max: np.ndarray, p: SolveParams) -> Reason:
    """Classify a non-converged solve: limit vs singular vs task conflict."""
    # at-limit?
    span = np.where(np.isfinite(q_max - q_min), q_max - q_min, 1.0)
    tol = np.maximum(p.limit_margin, p.limit_margin * span)
    at_lo = np.isfinite(q_min) & (q - q_min <= tol)
    at_hi = np.isfinite(q_max) & (q_max - q <= tol)
    if np.any(at_lo | at_hi):
        return Reason.JOINT_LIMIT
    # near-singular?
    for t in tasks:
        s = t.stiffness_vec()
        rows = [i for i in range(6) if s[i] > 0.0]
        if not rows:
            continue
        _, sigma_min = model.manipulability(q, t.frame, rows=rows)
        if sigma_min < p.sigma_singular:
            return Reason.SINGULAR
    # otherwise the (possibly multi-task) objective just can't be met
    return Reason.TASK_CONFLICT


def _blocking_joints(model: RobotModel, q: np.ndarray, q_min: np.ndarray,
                     q_max: np.ndarray, p: SolveParams,
                     mask: np.ndarray) -> List[str]:
    span = np.where(np.isfinite(q_max - q_min), q_max - q_min, 1.0)
    tol = np.maximum(p.limit_margin, p.limit_margin * span)
    at = ((np.isfinite(q_min) & (q - q_min <= tol)) |
          (np.isfinite(q_max) & (q_max - q <= tol))) & mask
    names = []
    for jn in model.joint_names:
        if at[model.q_index(jn)]:
            names.append(jn)
    return names


def _finalize(model, q, q_seed, tasks, extras, success, reason, iters, p,
              q_min, q_max, mask, max_iters_hit=False) -> Solution:
    residuals = []
    worst_rows = None
    for t in tasks:
        e = model.pose_error(q, t.frame, t.target_xyz_vec(), t.target_quat_vec())
        residuals.append(TaskResidual(t.frame, float(np.linalg.norm(e[:3])),
                                      float(np.linalg.norm(e[3:]))))
        s = t.stiffness_vec()
        rows = [i for i in range(6) if s[i] > 0.0]
        if rows:
            worst_rows = rows
    # manipulability/sigma reported for the first pose task (or its active rows)
    manip = 0.0
    sigma_min = 0.0
    if tasks:
        manip, sigma_min = model.manipulability(
            q, tasks[0].frame, rows=worst_rows)

    reachable = success
    if max_iters_hit and reason == Reason.OK:
        reason = Reason.MAX_ITERS
        reachable = False

    blocking = _blocking_joints(model, q, q_min, q_max, p, mask) \
        if reason == Reason.JOINT_LIMIT else []

    active_names = [jn for jn in model.joint_names
                    if mask[model.q_index(jn)]]

    return Solution(
        q=q.copy(),
        joint_names=model.joint_names,
        success=success,
        reachable=reachable,
        reason=reason,
        iters=iters,
        residuals=residuals,
        blocking_joints=blocking,
        manipulability=float(manip),
        sigma_min=float(sigma_min),
        q_seed=q_seed,
        active_joints=active_names,
    )
