"""Task / frame / solution dataclasses for the IK solver.

Pure data (no rclpy, no Pinocchio) so both ``ik_core`` and ``ik_node`` — and the
offline unit tests — share one vocabulary. A *task* binds a frame to a target and
a per-DOF stiffness; a *solution* carries the solved joints plus diagnostics and
the reachability verdict.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


# Full-pose default stiffness: every Cartesian DOF weighted equally.
POSE_STIFFNESS = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


def _as6(stiffness: Sequence[float]) -> np.ndarray:
    s = np.asarray(stiffness, dtype=float).reshape(-1)
    if s.size != 6:
        raise ValueError(f"stiffness must have 6 entries [x y z rx ry rz], got {s.size}")
    if np.any(s < 0.0):
        raise ValueError("stiffness entries must be >= 0")
    return s


@dataclass
class Task:
    """A single Cartesian task: place ``frame`` at ``target`` pose.

    Parameters
    ----------
    frame:
        Name of any frame in the kinematic model (an intermediate link, a tip,
        or a virtual tool frame). Need NOT be the last link of a chain.
    target_xyz:
        Target position (metres) in the solve/base frame.
    target_quat:
        Target orientation as a unit quaternion (w, x, y, z) in the base frame.
    stiffness:
        Per-DOF weight diag(Wt) = [x y z rx ry rz]; relative "how hard to push
        toward this DOF". Zero a rotation entry to let that orientation DOF float.
    """

    frame: str
    target_xyz: Tuple[float, float, float]
    target_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    stiffness: Tuple[float, float, float, float, float, float] = POSE_STIFFNESS

    def stiffness_vec(self) -> np.ndarray:
        return _as6(self.stiffness)

    def target_xyz_vec(self) -> np.ndarray:
        p = np.asarray(self.target_xyz, dtype=float).reshape(-1)
        if p.size != 3:
            raise ValueError("target_xyz must have 3 entries")
        return p

    def target_quat_vec(self) -> np.ndarray:
        q = np.asarray(self.target_quat, dtype=float).reshape(-1)
        if q.size != 4:
            raise ValueError("target_quat must have 4 entries (w, x, y, z)")
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            raise ValueError("target_quat has zero norm")
        return q / n

    # -- E1 task templates: structured ways to fill stiffness[6] ---------
    @classmethod
    def pose(cls, frame, xyz, quat=(1.0, 0.0, 0.0, 0.0), weight=1.0):
        """Full 6-DOF pose task (all DOF active)."""
        w = float(weight)
        return cls(frame, xyz, quat, (w, w, w, w, w, w))

    @classmethod
    def point(cls, frame, xyz, weight=1.0):
        """Position-only task: orientation free (rotation stiffness = 0)."""
        w = float(weight)
        return cls(frame, xyz, (1.0, 0.0, 0.0, 0.0), (w, w, w, 0.0, 0.0, 0.0))

    @classmethod
    def position_yaw(cls, frame, xyz, quat, weight=1.0, yaw_weight=None):
        """Position + a single orientation DOF (e.g. yaw); pitch/roll free."""
        w = float(weight)
        yw = w if yaw_weight is None else float(yaw_weight)
        return cls(frame, xyz, quat, (w, w, w, 0.0, 0.0, yw))


@dataclass
class RelativeTask:
    """Constrain the transform between two tips (dual-arm rigid hold, R9).

    The error drives ``X_b^{-1} X_a`` toward ``target_rel`` (a, b are frames).
    """

    frame_a: str
    frame_b: str
    target_rel_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_rel_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    stiffness: Tuple[float, float, float, float, float, float] = POSE_STIFFNESS

    def stiffness_vec(self) -> np.ndarray:
        return _as6(self.stiffness)


@dataclass
class ArmAngleTask:
    """Desired arm-angle psi for one S-R-S chain (R6, redundancy knob)."""

    chain: str
    psi_des: float
    stiffness: float = 0.5


@dataclass
class VirtualFrame:
    """A fixed 6-DOF offset link attached to ``parent`` (tool frame, R2)."""

    name: str
    parent: str
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def as_aux_frame(self) -> Dict:
        """Mapping accepted by ikt_common.urdf_loader.augment_urdf."""
        return {"name": self.name, "parent": self.parent,
                "xyz": list(self.xyz), "rpy": list(self.rpy)}


class Reason(str, Enum):
    """Machine-readable reachability verdict reason (R7)."""

    OK = "ok"
    JOINT_LIMIT = "joint_limit"
    SINGULAR = "singular"
    TASK_CONFLICT = "task_conflict"
    MAX_ITERS = "max_iters"
    TF_UNAVAILABLE = "tf_unavailable"


@dataclass
class TaskResidual:
    frame: str
    pos_err: float          # metres
    ori_err: float          # rad


@dataclass
class Solution:
    """Result of an IK solve. Advisory only — NOT a command."""

    q: np.ndarray                                   # solved joint vector (rad)
    joint_names: List[str]
    success: bool
    reachable: bool
    reason: Reason
    iters: int
    residuals: List[TaskResidual] = field(default_factory=list)
    blocking_joints: List[str] = field(default_factory=list)
    manipulability: float = 0.0
    sigma_min: float = 0.0
    arm_angles: Dict[str, float] = field(default_factory=dict)   # chain -> psi
    rel_residual: Optional[TaskResidual] = None
    self_collision_min_dist: Optional[float] = None
    q_seed: Optional[np.ndarray] = None
    active_joints: Optional[List[str]] = None      # joints allowed to move

    def max_pos_err(self) -> float:
        return max((r.pos_err for r in self.residuals), default=0.0)

    def max_ori_err(self) -> float:
        return max((r.ori_err for r in self.residuals), default=0.0)

    def delta_norm(self) -> float:
        """L2 norm of joint change from the seed (continuity / safety metric)."""
        if self.q_seed is None:
            return 0.0
        return float(np.linalg.norm(self.q - self.q_seed))

    def joint_dict(self) -> Dict[str, float]:
        """Solved configuration as a ``{joint_name: value}`` mapping (all DOF)."""
        return {jn: float(self.q[i]) for i, jn in enumerate(self.joint_names)}

    def q_active(self) -> List[float]:
        """Solved values for the active joints (all DOF if none were masked),
        in configuration-vector order."""
        names = self.active_joints or self.joint_names
        idx = {jn: i for i, jn in enumerate(self.joint_names)}
        return [float(self.q[idx[jn]]) for jn in names if jn in idx]
